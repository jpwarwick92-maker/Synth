// Audio thread: hosts the firmware (a WebAssembly module) and nothing else.
//
// It moves bytes and numbers between the page and the firmware. It does
// not know what any control means, what a note is, or what is on the display.
// It also stands in for the hardware round the firmware that the page
// cannot be: rough pots, the audio input's preamp and converter, and a
// clock sender for testing.

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
    this.inL = new Float32Array(mem, x.fw_in_left_ptr(), this.maxBlock);
    this.inR = new Float32Array(mem, x.fw_in_right_ptr(), this.maxBlock);
    this.midiCap = x.fw_midi_capacity();
    this.midi = new Uint8Array(mem, x.fw_midi_ptr(), this.midiCap);
    this.midiOut = new Uint8Array(mem, x.fw_midi_out_ptr(), 256);
    // The audio input's hardware, which the worklet stands in for: a preamp
    // set to the gain the firmware asks for, then a converter that cannot
    // go past full scale. A test tone can take the place of a microphone.
    this.preampDb = -1;
    this.preamp = 1;
    this.tone = { level: 0, step: 0, phase: 0 };
    this.inLight = 0;
    // A stand-in for a clock sender (a DAW, or the clock hub): MIDI clock
    // at a set tempo into one of the MIDI inputs, timed by the sample.
    this.clock = { bpm: 0, port: 0, phase: 0 };
    this.width = x.fw_display_width();
    this.height = x.fw_display_height();
    this.frameBuf = new Uint16Array(mem, x.fw_display_ptr(), this.width * this.height);
    this.count = x.fw_control_count();
    this.positions = new Uint16Array(this.count);
    this.leds = new Uint32Array(this.count);       // the light behind each control's label
    this.saveCount = x.fw_storage_save_count();
    this.patchSaveCount = x.fw_patch_save_count();
    // "Rough pots": the worklet stands in for a real panel's converter and
    // multiplexers. It keeps each pot's true position and feeds the
    // firmware readings of it with jitter and a little of the neighbouring
    // pot mixed in, a few pots at a time, as a scan would.
    this.rough = false;
    this.truePos = new Int32Array(this.count).fill(-1);
    this.scanAt = 0;
    this.noise = 0x1234567;
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

    this.kinds = controls.map((c) => c.kind);
    this.pots = controls.map((c, i) => (c.kind === 0 ? i : -1)).filter((i) => i >= 0);   // kind 0 = pot or fader
    this.port.onmessage = (e) => this.onMessage(e.data);
    this.port.postMessage({ t: 'ready', controls, display: { w: this.width, h: this.height }, sampleRate });
  }

  onMessage(m) {
    const x = this.x;
    switch (m.t) {
      case 'set':                                          // pot position, selector stop, button down/up
        if (this.rough && this.kinds[m.i] === 0) this.truePos[m.i] = m.v;   // the scan will find it there
        else x.fw_control_set(m.i, m.v);
        break;
      case 'rough':
        if (this.rough && !m.on) {
          // Back to perfect hardware: each pot is reported exactly where it is,
          // so the firmware is not left holding its last rough reading of it.
          for (const i of this.pots) if (this.truePos[i] >= 0) x.fw_control_set(i, this.truePos[i]);
        }
        this.rough = !!m.on;
        if (this.rough) for (const i of this.pots) this.truePos[i] = x.fw_control_get(i);
        break;
      case 'enc': x.fw_encoder_turn(m.i, m.d); break;      // encoder clicks
      case 'midi':                                         // bytes arriving at a MIDI input (0 = the jack, 1 = USB)
        for (let i = 0; i < m.bytes.length; i += this.midiCap) {
          const n = Math.min(this.midiCap, m.bytes.length - i);
          for (let k = 0; k < n; k++) this.midi[k] = m.bytes[i + k];
          x.fw_midi_port(n, m.port | 0);
        }
        break;
      case 'tone':                                         // a sine wave plugged into the audio input (level 0 = unplugged)
        this.tone.level = m.level || 0;
        this.tone.step = (2 * Math.PI * (m.hz || 440)) / sampleRate;
        break;
      case 'clock':                                        // the clock sender: start (bpm), continue, or stop
        this.clock.port = m.port | 0;
        if (m.bpm > 0 && this.clock.bpm > 0) {
          this.clock.bpm = m.bpm;                          // already running: a change of tempo
        } else if (m.bpm > 0) {
          this.midiByte(m.resume ? 0xfb : 0xfa, this.clock.port);
          if (!m.resume || !(this.clock.phase > 0)) this.clock.phase = 1;   // the first tick follows at once
          this.clock.bpm = m.bpm;
        } else if (this.clock.bpm > 0) {
          this.clock.bpm = 0;
          this.midiByte(0xfc, this.clock.port);
        }
        break;
      case 'buf': this.pool.push(m.buf); break;
    }
  }

  midiByte(b, port) {
    this.midi[0] = b;
    this.x.fw_midi_port(1, port);
  }

  // The audio input for the next `len` samples, as the converter delivers
  // it. Returns false when nothing is plugged in.
  readInput(input, done, len) {
    const tone = this.tone, mic = input && input.length > 0 ? input : null;
    if (!mic && !(tone.level > 0)) return false;
    const db = this.x.fw_input_gain_db();
    if (db !== this.preampDb) {
      this.preampDb = db;
      this.preamp = Math.pow(10, db / 20);
    }
    const g = this.preamp, l = mic ? mic[0] : null, r = mic ? mic[mic.length > 1 ? 1 : 0] : null;
    for (let i = 0; i < len; i++) {
      let a = l ? l[done + i] : 0, b = r ? r[done + i] : 0;
      if (tone.level > 0) {
        const t = tone.level * Math.sin(tone.phase);
        tone.phase += tone.step;
        if (tone.phase > 2 * Math.PI) tone.phase -= 2 * Math.PI;
        a += t;
        b += t;
      }
      this.inL[i] = Math.max(-1, Math.min(1, a * g));
      this.inR[i] = Math.max(-1, Math.min(1, b * g));
    }
    return true;
  }

  process(inputs, outputs) {
    const out = outputs[0];
    const n = out[0].length;
    const clock = this.clock;
    for (let done = 0; done < n; done += this.maxBlock) {
      const len = Math.min(this.maxBlock, n - done);
      if (clock.bpm > 0) {
        while (clock.phase >= 1) {
          clock.phase -= 1;
          this.midiByte(0xf8, clock.port);
        }
        clock.phase += (len * clock.bpm * 24) / (60 * sampleRate);
      }
      if (this.readInput(inputs[0], done, len)) this.x.fw_process_in(len);
      else this.x.fw_process(len);
      const whole = len === this.maxBlock;
      out[0].set(whole ? this.outL : this.outL.subarray(0, len), done);
      if (out.length > 1) out[1].set(whole ? this.outR : this.outR.subarray(0, len), done);
    }

    // Whatever is waiting to leave by the MIDI outputs goes to the page.
    for (let port = 0; port < 2; port++) {
      let bytes = null;
      for (let k = this.x.fw_midi_out(port); k > 0; k = this.x.fw_midi_out(port)) {
        if (!bytes) bytes = [];
        for (let i = 0; i < k; i++) bytes.push(this.midiOut[i]);
      }
      if (bytes) this.port.postMessage({ t: 'midiout', port, bytes });
    }

    if (this.rough) this.scanPots(8);

    this.sinceFrame += n;
    if (this.sinceFrame >= this.frameSamples) {
      const dt = Math.round((this.sinceFrame * 1000) / sampleRate);
      this.sinceFrame = 0;
      this.frame(dt);
    }
    return true;
  }

  // Reads the next few pots as rough hardware would: the true position,
  // plus or minus 8 counts of jitter (about what a Teensy 4's converter
  // gives), plus 0.15% of the difference from the pot read just before it
  // (a multiplexer that has been given time to settle).
  scanPots(howMany) {
    const pots = this.pots;
    for (let k = 0; k < howMany; k++) {
      const i = pots[this.scanAt], before = pots[(this.scanAt + pots.length - 1) % pots.length];
      this.scanAt = (this.scanAt + 1) % pots.length;
      this.noise = (Math.imul(this.noise, 1664525) + 1013904223) | 0;
      const jitter = ((this.noise >>> 8) % 17) - 8;
      const truth = this.truePos[i];
      const reading = truth + jitter + Math.round(0.0015 * (this.truePos[before] - truth));
      this.x.fw_pot_reading(i, Math.min(4095, Math.max(0, reading)));
    }
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

    const inLight = x.fw_input_light();                    // the light by the input jack
    if (inLight !== this.inLight || this.first) {
      this.inLight = inLight;
      msg.inLight = inLight;
    }
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

    if (msg.fb || msg.pos || msg.led || msg.state || msg.patches || msg.inLight !== undefined) this.port.postMessage(msg, transfer);
  }
}

registerProcessor('synth', SynthProcessor);
