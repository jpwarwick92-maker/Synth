// Long run of the browser build, with no browser: loads the WebAssembly
// module the page uses and plays it exactly as the audio thread does, for
// many simulated minutes, with every delay line in use.
//
// The native tests cannot see everything about this build: its memory is
// laid out differently, so a stray read that is harmless there can pick up
// wild values here. (Found by hand on 2026-11-14: about once in three
// minutes the chorus read one sample past its buffer, and the sound
// collapsed into a roar.) This plays long enough for such a fault to show.
//
// Usage: node soak.mjs <site dir> [minutes]

import { readFile } from 'node:fs/promises';
import path from 'node:path';

const siteDir = process.argv[2] || 'build/site';
const fullMinutes = Number(process.argv[3] || 15);
const wasm = await readFile(path.join(siteDir, 'firmware.wasm'));
let fails = 0;

// 44.1 kHz is the Teensy's rate and 48 kHz the page's. 96 kHz is run for a shorter time: the firmware is
// not made for it, but a sound device set that high must not be able to break it.
for (const [sampleRate, minutes] of [[44100, fullMinutes], [48000, fullMinutes], [96000, Math.min(5, fullMinutes)]]) {
  const x = (await WebAssembly.instantiate(wasm, {})).instance.exports;
  x.fw_init(sampleRate);
  const block = x.fw_max_block();
  const midi = (bytes) => {
    new Uint8Array(x.memory.buffer, x.fw_midi_ptr(), bytes.length).set(bytes);
    x.fw_midi(bytes.length);
  };
  // chorus, delay mix and feedback, reverb mix; oscillator 2 plucked; the test chord held
  midi([0xb0, 50, 127, 0xb0, 49, 80, 0xb0, 48, 80, 0xb0, 53, 80, 0xb0, 116, 127, 0xb0, 27, 90]);
  midi([0x90, 48, 100, 0x90, 55, 100, 0x90, 60, 100, 0x90, 63, 100]);

  const perSecond = Math.round(sampleRate / block);
  let normal = 0, worst = 0, worstAt = 0, notNumber = -1, sinceTick = 0;
  for (let sec = 0; sec < minutes * 60; sec++) {
    if (sec % 3 === 0) midi([0x80, 72, 0, 0x90, 72, 90, 0xb0, 46, 30 + ((sec * 7) % 60)]);   // a fresh pluck; the delay time moves
    let sum = 0;
    for (let b = 0; b < perSecond; b++) {
      x.fw_process(block);
      const l = new Float32Array(x.memory.buffer, x.fw_out_left_ptr(), block);
      const r = new Float32Array(x.memory.buffer, x.fw_out_right_ptr(), block);
      for (let i = 0; i < block; i++) {
        if (!Number.isFinite(l[i]) || !Number.isFinite(r[i])) { if (notNumber < 0) notNumber = sec; continue; }
        sum += l[i] * l[i] + r[i] * r[i];
      }
      sinceTick += block;
      if (sinceTick >= sampleRate / 30) {            // the display clock, as the page runs it
        x.fw_tick(Math.round((sinceTick * 1000) / sampleRate));
        sinceTick = 0;
      }
    }
    const rms = Math.sqrt(sum / (2 * perSecond * block));
    if (sec >= 3 && sec < 13) normal = Math.max(normal, rms);
    else if (sec >= 13 && rms > worst) { worst = rms; worstAt = sec; }
  }
  const ok = notNumber < 0 && normal > 0.01 && worst < 2 * normal;
  if (!ok) fails++;
  console.log(`  ${ok ? 'ok  ' : 'FAIL'}  ${minutes} minutes at ${sampleRate} Hz: level ${normal.toFixed(3)} at the start, never above ${worst.toFixed(3)}`
    + (ok ? '' : ` (at ${worstAt} s)${notNumber >= 0 ? `, output stopped being a number at ${notNumber} s` : ''}`));
}

// The same again with the newer paths busy: a tone at the audio input going
// through the effects, and the arpeggiator following MIDI clock whose ticks
// arrive unevenly. The settings are made the way a player makes them, with
// the Patch button and the encoders.
{
  const sampleRate = 48000, minutes = Math.min(5, fullMinutes);
  const x = (await WebAssembly.instantiate(wasm, {})).instance.exports;
  x.fw_init(sampleRate);
  const block = x.fw_max_block();
  const midi = (bytes) => {
    new Uint8Array(x.memory.buffer, x.fw_midi_ptr(), bytes.length).set(bytes);
    x.fw_midi(bytes.length);
  };
  const control = (name) => {
    const mem = new Uint8Array(x.memory.buffer);
    for (let i = 0; i < x.fw_control_count(); i++) {
      let p = x.fw_control_name(i), s = '';
      while (mem[p]) s += String.fromCharCode(mem[p++]);
      if (s === name) return i;
    }
    throw new Error(`no control named ${name}`);
  };
  const press = (name) => { x.fw_control_set(control(name), 1); x.fw_control_set(control(name), 0); };
  for (let i = 0; i < 4; i++) press('btn_patch');           // the MIDI page
  x.fw_encoder_turn(control('enc1'), 1);                    // Clock: MIDI
  press('btn_patch');                                       // the Input page
  x.fw_encoder_turn(control('enc3'), 2);                    // Send To: FX
  press('btn_patch');
  x.fw_control_set(control('arp_mode'), 1);
  midi([0xb0, 50, 127, 0xb0, 49, 80, 0xb0, 48, 80, 0xb0, 53, 80]);   // chorus, delay, reverb
  midi([0x90, 48, 100, 0x90, 55, 100, 0x90, 60, 100, 0x90, 63, 100, 0xfa]);

  const perSecond = Math.round(sampleRate / block), tickEvery = (sampleRate * 60) / (126 * 24);
  let normal = 0, worst = 0, notNumber = -1, sinceTick = 0, now = 0, nextTick = 0, ideal = 0, rng = 7, phase = 0;
  for (let sec = 0; sec < minutes * 60; sec++) {
    let sum = 0;
    for (let b = 0; b < perSecond; b++) {
      while (nextTick <= now) {
        midi([0xf8]);
        ideal += tickEvery;
        rng = (Math.imul(rng, 1664525) + 1013904223) | 0;
        nextTick = ideal + (((rng >>> 8) / 16777216) * 2 - 1) * 0.004 * sampleRate;   // up to 4 ms early or late
      }
      const il = new Float32Array(x.memory.buffer, x.fw_in_left_ptr(), block);
      const ir = new Float32Array(x.memory.buffer, x.fw_in_right_ptr(), block);
      for (let i = 0; i < block; i++) {
        il[i] = ir[i] = 0.2 * Math.sin(phase);
        phase += (2 * Math.PI * 330) / sampleRate;
      }
      x.fw_process_in(block);
      now += block;
      const l = new Float32Array(x.memory.buffer, x.fw_out_left_ptr(), block);
      const r = new Float32Array(x.memory.buffer, x.fw_out_right_ptr(), block);
      for (let i = 0; i < block; i++) {
        if (!Number.isFinite(l[i]) || !Number.isFinite(r[i])) { if (notNumber < 0) notNumber = sec; continue; }
        sum += l[i] * l[i] + r[i] * r[i];
      }
      sinceTick += block;
      if (sinceTick >= sampleRate / 30) {
        x.fw_tick(Math.round((sinceTick * 1000) / sampleRate));
        sinceTick = 0;
      }
    }
    const rms = Math.sqrt(sum / (2 * perSecond * block));
    if (sec >= 3 && sec < 13) normal = Math.max(normal, rms);
    else if (sec >= 13) worst = Math.max(worst, rms);
  }
  const ok = notNumber < 0 && normal > 0.01 && worst < 2 * normal;
  if (!ok) fails++;
  console.log(`  ${ok ? 'ok  ' : 'FAIL'}  ${minutes} minutes at ${sampleRate} Hz with the audio input and a MIDI clock in use: level ${normal.toFixed(3)} at the start, never above ${worst.toFixed(3)}`
    + (notNumber >= 0 ? `, output stopped being a number at ${notNumber} s` : ''));
}

console.log(`long run of the browser build: ${fails ? 'FAILED' : 'passed'}`);
process.exit(fails ? 1 : 0);
