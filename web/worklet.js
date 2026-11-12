// Audio thread: hosts the firmware (a WebAssembly module) and nothing else.
//
// It moves bytes and numbers between the page and the firmware. It does
// not know what any control means, what a note is, or what is on the display.

class SynthProcessor extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const o = options.processorOptions;
    const module = new WebAssembly.Module(o.wasm);
    const x = new WebAssembly.Instance(module, {}).exports;
    this.x = x;

    // Saved state goes in before start-up, as a real synth would read its flash.
    if (o.state && o.state.length > 0 && o.state.length <= x.fw_storage_capacity()) {
      new Uint8Array(x.memory.buffer, x.fw_storage_ptr(), o.state.length).set(o.state);
      x.fw_storage_set_len(o.state.length);
    }
    // ...and its stored patches.
    this.slots = x.fw_patch_slots();
    (o.patches || []).forEach((bytes, slot) => {
      if (!bytes || slot >= this.slots || bytes.length > x.fw_patch_capacity()) return;
      new Uint8Array(x.memory.buffer, x.fw_patch_ptr(slot), bytes.length).set(bytes);
      x.fw_patch_set_len(slot, bytes.length);
    });
    x.fw_init(sampleRate);

    const mem = x.memory.buffer; // never grows: the firmware does not allocate
    this.maxBlock = x.fw_max_block();
    this.outL = new Float32Array(mem, x.fw_out_left_ptr(), this.maxBlock);
    this.outR = new Float32Array(mem, x.fw_out_right_ptr(), this.maxBlock);
    this.midiCap = x.fw_midi_capacity();
    this.midi = new Uint8Array(mem, x.fw_midi_ptr(), this.midiCap);
    this.width = x.fw_display_width();
    this.height = x.fw_display_height();
    this.frameBuf = new Uint16Array(mem, x.fw_display_ptr(), this.width * this.height);
    this.count = x.fw_control_count();
    this.positions = new Uint16Array(this.count);
    this.leds = new Uint32Array(this.count);       // the light behind each control's label
    this.saveCount = x.fw_storage_save_count();
    this.patchSaveCount = x.fw_patch_save_count();
    this.pool = [];            // display buffers handed back by the page, reused to avoid garbage
    this.first = true;
    this.frameSamples = sampleRate / 30;
    this.sinceFrame = 0;

    const bytes = new Uint8Array(mem);
    const controls = [];
    for (let i = 0; i < this.count; i++) {
      let p = x.fw_control_name(i), name = '';
      while (bytes[p]) name += String.fromCharCode(bytes[p++]);
      controls.push({ name, kind: x.fw_control_kind(i), steps: x.fw_control_steps(i) });
    }

    this.port.onmessage = (e) => this.onMessage(e.data);
    this.port.postMessage({ t: 'ready', controls, display: { w: this.width, h: this.height }, sampleRate });
  }

  onMessage(m) {
    const x = this.x;
    switch (m.t) {
      case 'set': x.fw_control_set(m.i, m.v); break;       // pot position, selector stop, button down/up
      case 'enc': x.fw_encoder_turn(m.i, m.d); break;      // encoder clicks
      case 'midi':
        for (let i = 0; i < m.bytes.length; i += this.midiCap) {
          const n = Math.min(this.midiCap, m.bytes.length - i);
          for (let k = 0; k < n; k++) this.midi[k] = m.bytes[i + k];
          x.fw_midi(n);
        }
        break;
      case 'buf': this.pool.push(m.buf); break;
    }
  }

  process(inputs, outputs) {
    const out = outputs[0];
    const n = out[0].length;
    for (let done = 0; done < n; done += this.maxBlock) {
      const len = Math.min(this.maxBlock, n - done);
      this.x.fw_process(len);
      const whole = len === this.maxBlock;
      out[0].set(whole ? this.outL : this.outL.subarray(0, len), done);
      if (out.length > 1) out[1].set(whole ? this.outR : this.outR.subarray(0, len), done);
    }

    this.sinceFrame += n;
    if (this.sinceFrame >= this.frameSamples) {
      const dt = Math.round((this.sinceFrame * 1000) / sampleRate);
      this.sinceFrame = 0;
      this.frame(dt);
    }
    return true;
  }

  // About 30 times a second: advance the firmware's clock and pass on
  // whatever changed (display pixels, control positions, label lights, saved state, patches).
  frame(dtMs) {
    const x = this.x;
    const changed = x.fw_tick(dtMs);
    const msg = { t: 'frame' };
    const transfer = [];

    if (changed) {
      const buf = this.pool.pop() || new ArrayBuffer(this.frameBuf.byteLength);
      new Uint16Array(buf).set(this.frameBuf);
      msg.fb = buf;
      transfer.push(buf);
    }

    // Sent when something moved, and twice a second regardless, so the page
    // always converges on what the firmware holds.
    this.ticks = (this.ticks || 0) + 1;
    let moved = this.first || this.ticks % 15 === 0;
    for (let i = 0; i < this.count; i++) {
      const v = x.fw_control_get(i);
      if (v !== this.positions[i]) {
        this.positions[i] = v;
        moved = true;
      }
    }
    if (moved) msg.pos = this.positions.slice();

    let lit = this.first || this.ticks % 15 === 0;
    for (let i = 0; i < this.count; i++) {
      const v = x.fw_led_rgb(i);
      if (v !== this.leds[i]) {
        this.leds[i] = v;
        lit = true;
      }
    }
    if (lit) msg.led = this.leds.slice();
    this.first = false;

    const saves = x.fw_storage_save_count();
    if (saves !== this.saveCount) {
      this.saveCount = saves;
      msg.state = new Uint8Array(x.memory.buffer, x.fw_storage_ptr(), x.fw_storage_len()).slice();
    }

    const patchSaves = x.fw_patch_save_count();
    if (patchSaves !== this.patchSaveCount) {
      this.patchSaveCount = patchSaves;
      msg.patches = [];
      for (let s = 0; s < this.slots; s++) {
        const n = x.fw_patch_len(s);
        msg.patches.push(n ? new Uint8Array(x.memory.buffer, x.fw_patch_ptr(s), n).slice() : null);
      }
    }

    if (msg.fb || msg.pos || msg.led || msg.state || msg.patches) this.port.postMessage(msg, transfer);
  }
}

registerProcessor('synth', SynthProcessor);
