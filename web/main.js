// The page: fake hardware around the firmware.
//
// It draws the panel from the layout file, reports raw control positions
// and MIDI bytes to the firmware, shows the firmware's display buffer and
// plays its audio. There is no synthesis, parameter or menu logic here.

import { buildPanel, makeControl, setKnobMode } from './panel.js';

const LS_STATE = 'synth.state.v1';     // the firmware's saved state, as it would sit in flash
const LS_PX_PER_MM = 'synth.pxPerMm';  // screen calibration for the 1:1 view
const LS_VIEW = 'synth.view';
const LS_KNOBS = 'synth.knobs';        // 'turn' or 'updown'
const LS_ZOOM = 'synth.zoom';

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

function sendMidi(bytes) {
  if (node) node.port.postMessage({ t: 'midi', bytes: Array.from(bytes) });
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
  if (m.t !== 'frame') return;

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
  if (m.state) {
    try { localStorage.setItem(LS_STATE, btoa(String.fromCharCode(...m.state))); } catch (_) { /* storage may be off */ }
    window.__synth.saves++;
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
  if (!$('zoom').hidden) zoomCtx.drawImage(lcdSrc, 0, 0, zoomCtx.canvas.width, zoomCtx.canvas.height);
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

function setupView() {
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
    sendMidi([0xe0, v14 & 0x7f, v14 >> 7]);
  });
  const mod = setupWheel($('mod'), (v) => sendMidi([0xb0, 1, Math.round(v * 127)]));

  const chord = [48, 55, 60, 63];
  const toggle = (btn, on, off) => {
    btn.addEventListener('click', () => {
      const now = btn.getAttribute('aria-pressed') !== 'true';
      btn.setAttribute('aria-pressed', String(now));
      (now ? on : off)();
    });
  };
  toggle($('test-note'),
    () => sendMidi(chord.flatMap((n) => [0x90, n, 100])),
    () => sendMidi(chord.flatMap((n) => [0x80, n, 0])));
  toggle($('sustain'), () => sendMidi([0xb0, 64, 127]), () => sendMidi([0xb0, 64, 0]));

  $('panic').addEventListener('click', () => {
    sendMidi([0xb0, 120, 0, 0xb0, 123, 0]);          // all sound off, all notes off
    $('test-note').setAttribute('aria-pressed', 'false');
    $('sustain').setAttribute('aria-pressed', 'false');
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
      const names = [];
      for (const input of access.inputs.values()) {
        input.onmidimessage = (e) => sendMidi(e.data);
        if (input.state === 'connected') names.push(input.name || 'MIDI input');
      }
      show(names.length ? `Keyboard: ${names.join(', ')}` : 'Keyboard: none connected', names.length > 0);
    };
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
    ctx = new AudioContext({ latencyHint: 'interactive' });
    await ctx.audioWorklet.addModule('worklet.js');
    node = new AudioWorkletNode(ctx, 'synth', {
      numberOfInputs: 0,
      numberOfOutputs: 1,
      outputChannelCount: [2],
      processorOptions: { wasm, state: savedState() },
    });
    node.port.onmessage = onFirmwareMessage;
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
  problems,
  midi: sendMidi,
  position: (id) => controls.get(id)?.value,
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
  fetch('build.json').then((r) => r.json()).then((b) => { $('build').textContent = `${b.build} (${b.date})`; })
    .catch(() => { $('build').textContent = 'local'; });
  $('start-btn').addEventListener('click', start);
  window.__synth.loaded = true;
})();
