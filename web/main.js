// The page: fake hardware around the firmware.
//
// It draws the panel from the layout file, reports raw control positions
// and MIDI bytes to the firmware, shows the firmware's display buffer and
// plays its audio. There is no synthesis, parameter or menu logic here.

import { buildPanel, makeControl, setKnobMode, panelProblems } from './panel.js';

const LS_STATE = 'synth.state.v1';     // the firmware's saved state, as it would sit in flash
const LS_PATCHES = 'synth.patches.v1'; // the patch slots, likewise
const LS_PX_PER_MM = 'synth.pxPerMm';  // screen calibration for the 1:1 view
const LS_VIEW = 'synth.view';
const LS_KNOBS = 'synth.knobs';        // 'turn' or 'updown'
const LS_ZOOM = 'synth.zoom';
const LS_ACRYLIC = 'synth.acrylic';    // '1' = show the display as through amber acrylic
const LS_ROUGH = 'synth.rough';        // '1' = feed the firmware jittery pot readings, as real hardware would

const $ = (id) => document.getElementById(id);

let layout = null;
let ctx = null;
let node = null;
let fwControls = [];             // [{name, kind, steps}] as the firmware lists them
const fwIndex = new Map();       // control name -> firmware index
let controls = new Map();        // control id -> on-screen control
// The firmware's frame lives in an off-screen canvas at its true 320x240.
// The visible display and the zoomed copy are both painted from it.
let lcdSrc = null, lcdSrcCtx = null, lcdImage = null, lcdPixels = null;
let lcdCtx = null, zoomCtx = null;
const problems = [];

// ---------------------------------------------------------------- hardware -> firmware

function send(id, kind, value) {
  const i = fwIndex.get(id);
  if (i === undefined || !node) return;
  node.port.postMessage(kind === 'enc' ? { t: 'enc', i, d: value } : { t: 'set', i, v: value });
}

// Bytes arriving at one of the synth's MIDI inputs: 0 = the MIDI jack, 1 = USB.
function sendMidi(bytes, port = 0) {
  if (node) node.port.postMessage({ t: 'midi', bytes: Array.from(bytes), port });
}

// The test aids stand in for a keyboard set to the channel the synth plays
// (channel 1 when it plays them all). They are written for channel 1 and
// moved to that channel here.
let aidChannel = 0;
let aidsReleased = () => {};
function sendAid(bytes) {
  sendMidi(Array.from(bytes, (b) => (b >= 0x80 && b < 0xf0 ? (b & 0xf0) | aidChannel : b)));
}

// Bytes leaving by the synth's MIDI outputs. The MIDI out jack (port 0)
// goes to whichever device is picked in the bar at the top; the tests read the log.
let midiOutDevice = null;
let outDotTimer = 0;
function onMidiOut(port, bytes) {
  const log = window.__synth.midiOut;
  log.push({ port, bytes });
  if (log.length > 4000) log.splice(0, log.length - 4000);
  if (port !== 0) return;
  $('out-dot').classList.add('on');
  clearTimeout(outDotTimer);
  outDotTimer = setTimeout(() => $('out-dot').classList.remove('on'), 120);
  if (midiOutDevice) {
    try { midiOutDevice.send(bytes); } catch (_) { /* the device went away */ }
  }
}

// The light beside the input jack, in the colour the firmware gives it.
function showInputLight(rgb) {
  window.__synth.inputLight = rgb;
  $('in-light').style.background = rgb ? `#${rgb.toString(16).padStart(6, '0')}` : '';
}

// ---------------------------------------------------------------- firmware -> screen

function onFirmwareMessage(e) {
  const m = e.data;
  if (m.t === 'ready') {
    fwControls = m.controls;
    fwControls.forEach((c, i) => fwIndex.set(c.name, i));
    checkLayoutAgainstFirmware();
    window.__synth.sampleRate = m.sampleRate;
    return;
  }
  if (m.t === 'midiout') return onMidiOut(m.port, m.bytes);
  if (m.t !== 'frame') return;
  if (m.inLight !== undefined) showInputLight(m.inLight);
  if (m.channel !== undefined) {
    // The synth lets go of everything when its channel changes: the aids' buttons follow.
    if (window.__synth.channel !== undefined) aidsReleased();
    aidChannel = m.channel > 0 ? m.channel - 1 : 0;
    window.__synth.channel = m.channel;
  }

  if (m.fb) {
    drawDisplay(new Uint16Array(m.fb));
    node.port.postMessage({ t: 'buf', buf: m.fb }, [m.fb]);   // hand the buffer back for reuse
    window.__synth.frames++;
  }
  if (m.pos) {
    for (let i = 0; i < m.pos.length; i++) {
      const c = controls.get(fwControls[i].name);
      if (c) c.set(m.pos[i]);
    }
    window.__synth.ready = true;
  }
  if (m.led) {
    for (let i = 0; i < m.led.length; i++) {
      const c = controls.get(fwControls[i].name);
      if (c) c.led(m.led[i]);
    }
  }
  if (m.state) {
    try { localStorage.setItem(LS_STATE, btoa(String.fromCharCode(...m.state))); } catch (_) { /* storage may be off */ }
    window.__synth.saves++;
  }
  if (m.patches) {
    const slots = m.patches.map((bytes) => (bytes ? btoa(String.fromCharCode(...bytes)) : null));
    try { localStorage.setItem(LS_PATCHES, JSON.stringify(slots)); } catch (_) { /* storage may be off */ }
    window.__synth.patchSaves++;
  }
}

// RGB565, exactly as the real display takes it -> the canvases.
function drawDisplay(src) {
  const dst = lcdPixels;
  for (let i = 0; i < src.length; i++) {
    const p = src[i];
    const r = (p >> 11) & 31, g = (p >> 5) & 63, b = p & 31;
    dst[i] = 0xff000000 | (((b << 3) | (b >> 2)) << 16) | (((g << 2) | (g >> 4)) << 8) | ((r << 3) | (r >> 2));
  }
  lcdSrcCtx.putImageData(lcdImage, 0, 0);
  paintDisplay();
}

// The on-panel display is drawn at the screen's own pixel grid. When it is
// shown smaller than 320x240 screen pixels (the usual case in the fit view)
// the frame is averaged down, so thin lines and small letters fade evenly
// instead of losing whole rows and columns. At 2x or more, pixels stay crisp.
function paintDisplay() {
  const lcd = lcdCtx.canvas;
  const scale = lcd.width / lcdSrc.width;
  lcdCtx.imageSmoothingEnabled = scale < 2;
  lcdCtx.imageSmoothingQuality = 'high';
  lcdCtx.drawImage(lcdSrc, 0, 0, lcd.width, lcd.height);
  tint(lcdCtx);
  if (!$('zoom').hidden) {
    zoomCtx.drawImage(lcdSrc, 0, 0, zoomCtx.canvas.width, zoomCtx.canvas.height);
    tint(zoomCtx);
  }
}

// The amber acrylic that will sit over the real display, as a sheet of
// colour laid over the picture: it lets red through, most of the green and
// little of the blue. This is the panel's doing, not the firmware's, so the
// firmware's own frame (which the tests read) is untouched.
const ACRYLIC = 'rgb(255, 176, 56)';
function tint(c) {
  if (localStorage.getItem(LS_ACRYLIC) !== '1') return;
  c.save();
  c.globalCompositeOperation = 'multiply';
  c.fillStyle = ACRYLIC;
  c.fillRect(0, 0, c.canvas.width, c.canvas.height);
  c.restore();
}

// Keeps the visible display's pixel grid equal to the screen's.
function fitDisplayToScreen() {
  const lcd = lcdCtx.canvas;
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(lcd.clientWidth * dpr)), h = Math.max(1, Math.round(lcd.clientHeight * dpr));
  if (lcd.width !== w || lcd.height !== h) {
    lcd.width = w;
    lcd.height = h;
  }
  paintDisplay();
}

// The layout file and the firmware must agree on which controls exist.
function checkLayoutAgainstFirmware() {
  const inLayout = new Set([...layout.controls, ...(layout.rear || [])].map((d) => d.id));
  for (const id of inLayout) {
    if (!fwIndex.has(id)) {
      problems.push(`layout control "${id}" is unknown to the firmware`);
      const c = controls.get(id);
      if (c) c.el.classList.add('unknown');
    }
  }
  for (const c of fwControls) if (!inLayout.has(c.name)) problems.push(`firmware control "${c.name}" is not in the layout`);
  for (const d of layout.controls) {
    const i = fwIndex.get(d.id);
    if (i !== undefined && (d.type === 'selector' || d.type === 'toggle') && d.stops.length !== fwControls[i].steps)
      problems.push(`"${d.id}" has ${d.stops.length} stops in the layout, ${fwControls[i].steps} in the firmware`);
  }
  if (problems.length) console.warn('Layout problems:\n' + problems.join('\n'));
}

// ---------------------------------------------------------------- panel

async function loadPanel() {
  layout = await (await fetch('layouts/a3.json')).json();
  controls = buildPanel($('panel'), layout, send);
  problems.push(...panelProblems);

  // the display sits where the layout file says, at its true size
  const { width: W, height: H } = layout.panel, d = layout.display, lcd = $('lcd');
  lcd.style.left = `${(d.x / W) * 100}%`;
  lcd.style.top = `${(d.y / H) * 100}%`;
  lcd.style.width = `${(d.w / W) * 100}%`;
  lcd.style.height = `${(d.h / H) * 100}%`;
  lcdSrc = document.createElement('canvas');
  lcdSrc.width = 320;
  lcdSrc.height = 240;
  lcdSrcCtx = lcdSrc.getContext('2d');
  lcdImage = lcdSrcCtx.createImageData(lcdSrc.width, lcdSrc.height);
  lcdPixels = new Uint32Array(lcdImage.data.buffer);
  lcdPixels.fill(0xff000000);
  lcdCtx = lcd.getContext('2d');
  zoomCtx = $('lcd-zoom').getContext('2d');
  zoomCtx.imageSmoothingEnabled = false;
  new ResizeObserver(fitDisplayToScreen).observe(lcd);
  window.addEventListener('resize', fitDisplayToScreen);   // also catches a change of browser zoom
  fitDisplayToScreen();

  // rear-panel controls get their own small strip
  const rear = $('rear');
  (layout.rear || []).forEach((def, k) => {
    controls.set(def.id, makeControl(rear, { ...def, x: 15 + k * 30, y: 9.5 }, layout.sizes, send));
  });
  applyView();
}

// ---------------------------------------------------------------- view: fit / 1:1

function pxPerMm() { return parseFloat(localStorage.getItem(LS_PX_PER_MM)) || 0; }

function applyView() {
  const oneToOne = localStorage.getItem(LS_VIEW) === '1:1' && pxPerMm() > 0;
  const wrap = $('panelwrap');
  if (oneToOne) {
    wrap.style.width = `${layout.panel.width * pxPerMm()}px`;
    wrap.style.maxWidth = 'none';
  } else {
    wrap.style.width = '';
    wrap.style.maxWidth = '';
  }
  $('view-toggle').setAttribute('aria-pressed', String(oneToOne));
}

const HINTS = {
  turn: 'Knobs, selectors and encoders: press and circle the pointer round the control; it turns with you. Further out is finer.',
  updown: 'Knobs, selectors and encoders: press and drag up to turn clockwise, down to turn back.',
};

function applyKnobMode() {
  const mode = localStorage.getItem(LS_KNOBS) === 'updown' ? 'updown' : 'turn';
  setKnobMode(mode);
  $('knob-mode').textContent = mode === 'turn' ? 'Knobs: turn' : 'Knobs: drag up/down';
  $('hint').textContent = `${HINTS[mode]} Scrolling over any control also moves it (two fingers on a trackpad). Hold Shift for fine. `
    + 'Selectors: or tap a stop\'s label. Encoders: double-tap to push. Faders follow the pointer. Switches: tap to flip.';
  window.__synth.knobMode = mode;
}

function applyZoom() {
  const on = localStorage.getItem(LS_ZOOM) === '1';
  $('zoom').hidden = !on;
  $('zoom-toggle').setAttribute('aria-pressed', String(on));
  if (on && lcdCtx) paintDisplay();
}

function applyAcrylic() {
  $('acrylic-toggle').setAttribute('aria-pressed', String(localStorage.getItem(LS_ACRYLIC) === '1'));
  if (lcdCtx) paintDisplay();
}

function setupView() {
  $('acrylic-toggle').addEventListener('click', () => {
    localStorage.setItem(LS_ACRYLIC, localStorage.getItem(LS_ACRYLIC) === '1' ? '0' : '1');
    applyAcrylic();
  });
  $('knob-mode').addEventListener('click', () => {
    localStorage.setItem(LS_KNOBS, localStorage.getItem(LS_KNOBS) === 'updown' ? 'turn' : 'updown');
    applyKnobMode();
  });
  $('zoom-toggle').addEventListener('click', () => {
    localStorage.setItem(LS_ZOOM, localStorage.getItem(LS_ZOOM) === '1' ? '0' : '1');
    applyZoom();
  });
  $('view-toggle').addEventListener('click', () => {
    const on = localStorage.getItem(LS_VIEW) === '1:1';
    if (!on && !pxPerMm()) return openCalibration(true);   // 1:1 means nothing until the screen is measured
    localStorage.setItem(LS_VIEW, on ? 'fit' : '1:1');
    applyView();
  });
  $('calibrate').addEventListener('click', () => openCalibration(false));

  const slider = $('cal-slider'), card = $('cal-card');
  const show = () => {
    const v = parseFloat(slider.value);
    card.style.width = `${85.6 * v}px`;     // ISO bank card: 85.60 x 53.98 mm
    card.style.height = `${53.98 * v}px`;
    $('cal-value').textContent = `${v.toFixed(2)} pixels per mm`;
  };
  slider.addEventListener('input', show);
  let thenOneToOne = false;
  function openCalibration(switchAfter) {
    thenOneToOne = switchAfter;
    slider.value = pxPerMm() || 5;
    show();
    $('cal').showModal();
  }
  $('cal-save').addEventListener('click', () => {
    localStorage.setItem(LS_PX_PER_MM, slider.value);
    if (thenOneToOne) localStorage.setItem(LS_VIEW, '1:1');
    $('cal').close();
    applyView();
  });
}

// ---------------------------------------------------------------- test aids

function setupWheel(elem, onChange) {
  const thumb = elem.querySelector('.thumb');
  const spring = elem.dataset.spring === '1';
  let value = spring ? 0.5 : 0;                      // 0 = bottom, 1 = top
  const draw = () => {
    thumb.style.top = `calc(${(1 - value) * 100}% + ${12 * value - 6}px)`;   // keeps the 12 px thumb inside the track
    elem.dataset.value = value.toFixed(3);
  };
  const set = (v) => { value = Math.min(1, Math.max(0, v)); draw(); onChange(value); };
  const fromEvent = (e) => {
    const r = elem.getBoundingClientRect();
    set(1 - (e.clientY - r.top) / r.height);
  };
  elem.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    try { elem.setPointerCapture(e.pointerId); } catch (_) { /* synthetic pointer */ }
    fromEvent(e);
    const move = (ev) => { if (ev.pointerId === e.pointerId) fromEvent(ev); };
    const end = (ev) => {
      if (ev.pointerId !== e.pointerId) return;
      elem.removeEventListener('pointermove', move);
      elem.removeEventListener('pointerup', end);
      elem.removeEventListener('pointercancel', end);
      if (spring) set(0.5);                          // a bend wheel springs back to centre
    };
    elem.addEventListener('pointermove', move);
    elem.addEventListener('pointerup', end);
    elem.addEventListener('pointercancel', end);
  });
  draw();
  return { reset: () => set(spring ? 0.5 : 0) };
}

function setupAids() {
  // These send the same MIDI a keyboard would; they are not part of the panel.
  const bend = setupWheel($('bend'), (v) => {
    const v14 = Math.min(16383, Math.max(0, Math.round(v * 16383 + 0.5)));
    sendAid([0xe0, v14 & 0x7f, v14 >> 7]);
  });
  const mod = setupWheel($('mod'), (v) => sendAid([0xb0, 1, Math.round(v * 127)]));

  const chord = [48, 55, 60, 63];
  const toggle = (btn, on, off) => {
    btn.addEventListener('click', () => {
      const now = btn.getAttribute('aria-pressed') !== 'true';
      btn.setAttribute('aria-pressed', String(now));
      (now ? on : off)();
    });
  };
  toggle($('test-note'),
    () => sendAid(chord.flatMap((n) => [0x90, n, 100])),
    () => sendAid(chord.flatMap((n) => [0x80, n, 0])));
  toggle($('sustain'), () => sendAid([0xb0, 64, 127]), () => sendAid([0xb0, 64, 0]));

  // Real pots jitter and bleed into each other a little. This switches the
  // stand-in hardware from perfect readings to rough ones.
  const applyRough = () => {
    const on = localStorage.getItem(LS_ROUGH) === '1';
    $('rough').setAttribute('aria-pressed', String(on));
    if (node) node.port.postMessage({ t: 'rough', on });
  };
  $('rough').addEventListener('click', () => {
    localStorage.setItem(LS_ROUGH, localStorage.getItem(LS_ROUGH) === '1' ? '0' : '1');
    applyRough();
  });
  window.__synth.applyRough = applyRough;
  applyRough();

  // A microphone into the audio input. The browser's own echo and noise
  // processing is switched off: the firmware should get what a real input would.
  let micStream = null, micNode = null, micAsking = false;
  const micOff = () => {
    if (micNode) micNode.disconnect();
    if (micStream) micStream.getTracks().forEach((t) => t.stop());
    micNode = micStream = null;
    $('mic').setAttribute('aria-pressed', 'false');
  };
  $('mic').addEventListener('click', async () => {
    if (micStream) return micOff();
    if (!ctx || !node || micAsking) return;   // a second click while the browser is still asking must not open a second microphone
    micAsking = true;
    try {
      micStream = await navigator.mediaDevices.getUserMedia({
        audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false },
      });
      micNode = ctx.createMediaStreamSource(micStream);
      micNode.connect(node);
      $('mic').setAttribute('aria-pressed', 'true');
    } catch (_) {
      micOff();
      $('mic').title = 'The microphone was not allowed for this page.';
    }
    micAsking = false;
  });

  // A clock sender: MIDI clock into the MIDI in jack, with start and stop.
  const clockBpm = () => Math.min(300, Math.max(30, parseFloat($('clock-bpm').value) || 96));
  const sendClock = (bpm, resume = false, port = 0) => { if (node) node.port.postMessage({ t: 'clock', bpm, resume, port }); };
  toggle($('clock'), () => sendClock(clockBpm()), () => sendClock(0));
  $('clock-bpm').addEventListener('change', () => {
    if ($('clock').getAttribute('aria-pressed') === 'true') sendClock(clockBpm());
  });
  window.__synth.clock = sendClock;
  window.__synth.inputTone = (level, hz = 440) => { if (node) node.port.postMessage({ t: 'tone', level, hz }); };

  aidsReleased = () => {
    $('test-note').setAttribute('aria-pressed', 'false');
    $('sustain').setAttribute('aria-pressed', 'false');
  };
  $('panic').addEventListener('click', () => {
    // all sound off and all notes off, on every channel: whichever one the synth plays, it hears them
    const bytes = [];
    for (let ch = 0; ch < 16; ch++) bytes.push(0xb0 | ch, 120, 0, 0xb0 | ch, 123, 0);
    sendMidi(bytes);
    aidsReleased();
    bend.reset();
    mod.reset();
  });
}

// ---------------------------------------------------------------- MIDI keyboard

async function setupMidi() {
  const label = $('midi-status'), dot = $('midi-dot');
  const show = (text, on) => { label.textContent = text; dot.classList.toggle('on', !!on); window.__synth.midiStatus = text; };
  if (!navigator.requestMIDIAccess) {
    show('Keyboard: this browser has no MIDI (use Chrome on the Mac)');
    return;
  }
  try {
    const access = await navigator.requestMIDIAccess({ sysex: false });
    const refresh = () => {
      // the devices the MIDI out jack could be plugged into
      const pick = $('midi-out'), chosen = pick.value;
      while (pick.options.length > 1) pick.remove(1);
      for (const output of access.outputs.values()) {
        if (output.state === 'connected') pick.add(new Option(`MIDI out: ${output.name || 'MIDI output'}`, output.id));
      }
      pick.value = [...pick.options].some((o) => o.value === chosen) ? chosen : '';
      midiOutDevice = pick.value ? access.outputs.get(pick.value) : null;
      // A device that is both an input and the chosen output (a virtual bus
      // shared with a DAW, say) would hand the synth its own output straight
      // back, round and round. While it is the MIDI out, it is not listened to.
      const names = [], deaf = [];
      for (const input of access.inputs.values()) {
        const loops = midiOutDevice !== null && (input.name || '') === (midiOutDevice.name || '');
        input.onmidimessage = loops ? null : (e) => sendMidi(e.data);   // notes, controllers and clock alike
        if (input.state !== 'connected') continue;
        (loops ? deaf : names).push(input.name || 'MIDI input');
      }
      show((names.length ? `Keyboard: ${names.join(', ')}` : 'Keyboard: none connected')
        + (deaf.length ? ` (not listening to ${deaf.join(', ')}: it is the MIDI out)` : ''), names.length > 0);
      window.__synth.midiDeaf = deaf;
    };
    $('midi-out').addEventListener('change', refresh);
    access.onstatechange = refresh;                  // fires when a keyboard is plugged in or pulled out
    refresh();
  } catch (_) {
    show('Keyboard: MIDI was not allowed for this page');
  }
}

// ---------------------------------------------------------------- start-up

function savedState() {
  try {
    const s = localStorage.getItem(LS_STATE);
    return s ? Uint8Array.from(atob(s), (ch) => ch.charCodeAt(0)) : null;
  } catch (_) {
    return null;
  }
}

function savedPatches() {
  try {
    const slots = JSON.parse(localStorage.getItem(LS_PATCHES) || '[]');
    return slots.map((s) => (s ? Uint8Array.from(atob(s), (ch) => ch.charCodeAt(0)) : null));
  } catch (_) {
    return [];
  }
}

function showAudioDelay() {
  if (!ctx) return;
  const ms = ((ctx.baseLatency || 0) + (ctx.outputLatency || 0)) * 1000;
  $('audio-delay').textContent = `${ms.toFixed(0)} ms`;
  window.__synth.audioDelayMs = ms;
}

async function start() {
  const err = $('start-error');
  err.textContent = '';
  $('start-btn').disabled = true;
  try {
    if (!window.AudioWorkletNode) throw new Error('This browser cannot run the audio engine (no AudioWorklet).');
    const wasm = await (await fetch('firmware.wasm')).arrayBuffer();
    // The firmware's delay and reverb lines are sized for 48 kHz. Asking for that rate, whatever the
    // sound device is set to, keeps the page sounding the same on every machine; the browser converts.
    try {
      ctx = new AudioContext({ latencyHint: 'interactive', sampleRate: 48000 });
    } catch (_) {
      ctx = new AudioContext({ latencyHint: 'interactive' });   // a browser that cannot: the firmware is safe at any rate
    }
    await ctx.audioWorklet.addModule('worklet.js');
    node = new AudioWorkletNode(ctx, 'synth', {
      numberOfInputs: 1,                               // the audio input: nothing is plugged in until Mic in is pressed
      numberOfOutputs: 1,
      outputChannelCount: [2],
      processorOptions: { wasm, state: savedState(), patches: savedPatches() },
    });
    node.port.onmessage = onFirmwareMessage;
    window.__synth.applyRough();
    node.onprocessorerror = () => { err.textContent = 'The audio engine stopped with an error.'; $('start').hidden = false; };
    node.connect(ctx.destination);
    await ctx.resume();
    $('start').hidden = true;
    showAudioDelay();
    setInterval(showAudioDelay, 1000);
    setupMidi();
  } catch (e) {
    err.textContent = e.message || String(e);
    $('start-btn').disabled = false;
  }
}

// Hooks for the automated page tests and for poking at the page from the console.
window.__synth = {
  ready: false,
  frames: 0,
  saves: 0,
  patchSaves: 0,
  midiOut: [],                  // what has left by the MIDI outputs: [{port, bytes}]
  inputLight: 0,
  problems,
  midi: sendMidi,
  position: (id) => controls.get(id)?.value,
  led: (id) => controls.get(id)?.ledRgb ?? 0,   // the label light's colour as the firmware last set it
  controlIds: () => [...controls.keys()],
  firmwareControls: () => fwControls,
  // Display checks read the firmware's frame itself, not the scaled copy on screen.
  lcdCount: (rgb, y0 = 0, y1 = 240) => {
    let n = 0;
    for (let i = y0 * 320; i < y1 * 320; i++) {
      const p = lcdPixels[i];
      if (Math.abs((p & 255) - rgb[0]) < 12 && Math.abs(((p >> 8) & 255) - rgb[1]) < 12 && Math.abs(((p >> 16) & 255) - rgb[2]) < 12) n++;
    }
    return n;
  },
  lcdHash: () => {
    let h = 2166136261;
    for (let i = 0; i < lcdPixels.length; i++) h = Math.imul(h ^ lcdPixels[i], 16777619);
    return h >>> 0;
  },
};

(async function boot() {
  setupView();
  setupAids();
  applyKnobMode();
  await loadPanel();
  applyZoom();
  applyAcrylic();
  fetch('build.json').then((r) => r.json()).then((b) => { $('build').textContent = `${b.build} (${b.date})`; })
    .catch(() => { $('build').textContent = 'local'; });
  $('start-btn').addEventListener('click', start);
  window.__synth.loaded = true;
})();
