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

console.log(`long run of the browser build: ${fails ? 'FAILED' : 'passed'}`);
process.exit(fails ? 1 : 0);
