// Draws the panel described by a layout file and turns mouse and touch
// into raw hardware readings: a pot position (0..4095), a selector stop,
// encoder clicks, a button going down or up.
//
// This file knows shapes and gestures only. It has no idea what any control
// does; labels come from the layout file, as silk-screen on a real panel.

const NS = 'http://www.w3.org/2000/svg';
const POT_MAX = 4095;

// How knobs, selectors and encoders answer a drag:
//   'turn'   - the control turns with the pointer as it circles the control
//   'updown' - dragging up turns it clockwise, down anticlockwise
// Faders always follow the pointer along their slot.
let knobMode = 'turn';
export function setKnobMode(mode) { knobMode = mode === 'updown' ? 'updown' : 'turn'; }

const MIN_TURN_RADIUS = 8;   // px: closer to the centre than this, an angle means nothing

function el(name, attrs, parent, text) {
  const e = document.createElementNS(NS, name);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (text !== undefined) e.textContent = text;
  if (parent) parent.appendChild(e);
  return e;
}

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

// One finger or the mouse dragging on one control. Every control captures
// its own pointer, so several can be moved at once on a touch screen.
function onDrag(target, handlers) {
  target.addEventListener('pointerdown', (e) => {
    if (e.button > 0) return;
    e.preventDefault();
    try { target.setPointerCapture(e.pointerId); } catch (_) { /* synthetic pointers cannot be captured */ }
    const box = target.getBoundingClientRect();
    const st = {
      id: e.pointerId, x0: e.clientX, y0: e.clientY, lastY: e.clientY, moved: false, acc: 0,
      cx: box.left + box.width / 2, cy: box.top + box.height / 2,   // the control's centre on screen
      angle: null,
    };
    if (handlers.down) handlers.down(e, st);
    const move = (ev) => {
      if (ev.pointerId !== st.id) return;
      if (Math.abs(ev.clientY - st.y0) > 3 || Math.abs(ev.clientX - st.x0) > 3) st.moved = true;
      const dy = ev.clientY - st.lastY;
      st.lastY = ev.clientY;
      if (handlers.move) handlers.move(ev, st, dy);
    };
    const end = (ev) => {
      if (ev.pointerId !== st.id) return;
      target.removeEventListener('pointermove', move);
      target.removeEventListener('pointerup', end);
      target.removeEventListener('pointercancel', end);
      if (handlers.up) handlers.up(ev, st);
    };
    target.addEventListener('pointermove', move);
    target.addEventListener('pointerup', end);
    target.addEventListener('pointercancel', end);
  });
}

// Pointer direction seen from the control's centre, in degrees: 0 = straight
// up, clockwise positive. Null when the pointer is too close to the centre.
function angleOf(e, st) {
  const dx = e.clientX - st.cx, dy = e.clientY - st.cy;
  if (dx * dx + dy * dy < MIN_TURN_RADIUS * MIN_TURN_RADIUS) return null;
  return (Math.atan2(dx, -dy) * 180) / Math.PI;
}

// How many degrees the pointer has swept round the centre since the last call.
function sweep(e, st) {
  const a = angleOf(e, st);
  if (a === null || st.angle === null) {
    st.angle = a;
    return 0;
  }
  let d = a - st.angle;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  st.angle = a;
  return d;
}

// Scrolling over a control (two fingers on a trackpad, or a mouse wheel).
// `fn` gets the scrolled distance in pixels, positive = wheel up; a trackpad
// sends many small amounts, a wheel a few large ones.
function onWheel(target, fn) {
  target.addEventListener('wheel', (e) => {
    if (e.ctrlKey) return;                       // pinch-zoom, not a scroll
    e.preventDefault();
    const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 240 : 1;
    fn(-e.deltaY * unit, e);
  }, { passive: false });
}

const WHEEL_SWEEP = 1200;   // px of scrolling for a knob's full travel
const WHEEL_STEP = 40;      // px of scrolling per selector stop or encoder click

// On-screen pixels per millimetre of panel, for 1:1 feeling drags.
function pxPerMm(svg) {
  const vb = svg.viewBox.baseVal;
  return svg.getBoundingClientRect().width / vb.width;
}

// ---- the control types ----

function makeKnob(g, def, sizes, c, send) {
  const large = def.size === 'large';
  const r = (large ? sizes.knobLarge : sizes.knob) / 2;
  el('circle', { r, class: large ? 'cap large' : 'cap' }, g);
  const ptr = el('line', { x1: 0, y1: -r + 0.9, x2: 0, y2: -r + 0.9 + (large ? 6 : 4), class: large ? 'ptr large' : 'ptr' }, g);
  if (def.label) el('text', { y: r + 4.4, class: 'label' }, g, def.label);
  const hit = el('circle', { r: r + 2.5, class: 'hit' }, g);

  c.draw = () => ptr.setAttribute('transform', `rotate(${-135 + (270 * c.value) / POT_MAX})`);
  const nudge = (dv) => {
    c.fine = clamp(c.fine + dv, 0, POT_MAX);
    const v = Math.round(c.fine);
    if (v !== c.value) {
      c.value = v;
      c.touched();
      c.draw();
      send(def.id, 'set', v);
    }
  };
  onDrag(hit, {
    down: (e, st) => { c.held = true; c.fine = c.value; st.angle = angleOf(e, st); },
    move: (e, st, dy) => {
      const fine = e.shiftKey ? 0.1 : 1;
      if (knobMode === 'turn') nudge((sweep(e, st) / 270) * POT_MAX * fine);   // the pointer line follows the cursor
      else nudge(-dy * (POT_MAX / 180) * fine);                                // full travel = 180 px
    },
    up: () => { c.held = false; },
  });
  onWheel(hit, (px, e) => { c.fine = c.value; nudge((px / WHEEL_SWEEP) * POT_MAX * (e.shiftKey ? 0.1 : 1)); });
}

function makeFader(g, def, sizes, c, send) {
  const travel = def.travel;
  const [capW, capH] = sizes.faderCap;
  el('rect', { x: -sizes.faderSlot / 2, y: -travel / 2 - capH / 2, width: sizes.faderSlot, height: travel + capH, rx: 0.9, class: 'slot' }, g);
  const cap = el('g', {}, g);
  el('rect', { x: -capW / 2, y: -capH / 2, width: capW, height: capH, rx: 0.9, class: 'fcap' }, cap);
  el('line', { x1: -capW / 2 + 1.2, y1: 0, x2: capW / 2 - 1.2, y2: 0, class: 'fline' }, cap);
  if (def.label) el('text', { y: travel / 2 + capH / 2 + 5.6, class: 'label' }, g, def.label);
  const hit = el('rect', { x: -6.5, y: -travel / 2 - 5, width: 13, height: travel + 10, class: 'hit' }, g);

  c.draw = () => cap.setAttribute('transform', `translate(0 ${travel / 2 - (travel * c.value) / POT_MAX})`);
  const nudge = (dv) => {
    c.fine = clamp(c.fine + dv, 0, POT_MAX);
    const v = Math.round(c.fine);
    if (v !== c.value) {
      c.value = v;
      c.touched();
      c.draw();
      send(def.id, 'set', v);
    }
  };
  onDrag(hit, {
    down: () => { c.held = true; c.fine = c.value; },
    // the cap follows the finger: one millimetre of drag is one millimetre of travel
    move: (e, st, dy) => nudge((-dy / (travel * pxPerMm(g.ownerSVGElement))) * POT_MAX * (e.shiftKey ? 0.1 : 1)),
    up: () => { c.held = false; },
  });
  onWheel(hit, (px, e) => { c.fine = c.value; nudge((px / WHEEL_SWEEP) * POT_MAX * (e.shiftKey ? 0.1 : 1)); });
}

function makeSelector(g, def, sizes, c, send) {
  const stops = def.stops;
  const n = stops.length;
  const spacing = n === 3 ? 45 : 40;
  const angle = (i) => (i - (n - 1) / 2) * spacing;
  const r = sizes.selector / 2;
  const ticks = [], labels = [];
  for (let i = 0; i < n; i++) {
    const a = (angle(i) * Math.PI) / 180;
    const sx = Math.sin(a), cy = -Math.cos(a);
    ticks.push(el('line', { x1: sx * (r + 1.6), y1: cy * (r + 1.6), x2: sx * (r + 3.4), y2: cy * (r + 3.4), class: 'tick' }, g));
    const anchor = angle(i) < -10 ? 'end' : angle(i) > 10 ? 'start' : 'middle';
    const t = el('text', { x: sx * (r + 5), y: cy * (r + 5) + (anchor === 'middle' ? -0.2 : 0.9), 'text-anchor': anchor, class: 'stop' }, g, stops[i]);
    labels.push(t);
    // tapping a stop's label turns the switch straight to it
    const w = Math.max(5, stops[i].length * 1.9 + 2);
    const hx = sx * (r + 5) - (anchor === 'end' ? w - 1 : anchor === 'start' ? 1 : w / 2);
    const stopHit = el('rect', { x: hx, y: cy * (r + 5) - 3.2, width: w, height: 5, class: 'hit' }, g);
    stopHit.addEventListener('pointerdown', (e) => { e.preventDefault(); setStop(i); });
  }
  el('circle', { r, class: 'cap' }, g);
  const ptr = el('line', { x1: 0, y1: 0, x2: 0, y2: -r + 0.2, class: 'ptr' }, g);
  if (def.label) el('text', { y: r + 4.8, class: 'label' }, g, def.label);
  const hit = el('circle', { r: r + 2.2, class: 'hit' }, g);

  c.draw = () => {
    ptr.setAttribute('transform', `rotate(${angle(c.value)})`);
    for (let i = 0; i < n; i++) {
      ticks[i].classList.toggle('on', i === c.value);
      labels[i].classList.toggle('on', i === c.value);
    }
  };
  const setStop = (i) => {
    i = clamp(i, 0, n - 1);
    if (i === c.value) return;
    c.value = i;
    c.touched();
    c.draw();
    send(def.id, 'set', i);
  };
  onDrag(hit, {
    down: () => { c.held = true; },
    move: (e, st, dy) => {
      if (knobMode === 'turn') {              // the switch points at the stop nearest the cursor
        const a = angleOf(e, st);
        if (a !== null && Math.abs(a) < 150) setStop(Math.round(a / spacing + (n - 1) / 2));
        return;
      }
      st.acc -= dy;                           // one stop per 22 px, up = clockwise
      while (st.acc >= 22) { st.acc -= 22; setStop(c.value + 1); }
      while (st.acc <= -22) { st.acc += 22; setStop(c.value - 1); }
    },
    up: () => { c.held = false; },
  });
  let wheelAcc = 0;
  onWheel(hit, (px) => {
    wheelAcc += px;
    while (wheelAcc >= WHEEL_STEP) { wheelAcc -= WHEEL_STEP; setStop(c.value + 1); }
    while (wheelAcc <= -WHEEL_STEP) { wheelAcc += WHEEL_STEP; setStop(c.value - 1); }
  });
}

function makeEncoder(g, def, sizes, c, send) {
  const r = sizes.encoder / 2;
  el('circle', { r, class: 'enc' }, g);
  const ptr = el('line', { x1: 0, y1: -r + 0.9, x2: 0, y2: -r + 4.9, class: 'ptr' }, g);
  const hit = el('circle', { r: r + 2.5, class: 'hit' }, g);
  let lastTap = 0;

  c.draw = () => {
    const clicks = c.value > 32767 ? c.value - 65536 : c.value;   // firmware reports a 16-bit running count
    ptr.setAttribute('transform', `rotate(${(clicks * 15) % 360})`);
  };
  const turn = (d) => {
    c.value = (c.value + d + 65536) % 65536;
    c.touched();
    c.draw();
    send(def.id, 'enc', d);
  };
  onDrag(hit, {
    down: (e, st) => { st.angle = angleOf(e, st); },
    move: (e, st, dy) => {
      const per = knobMode === 'turn' ? 15 : 12;   // 15 degrees, or 12 px, per click
      st.acc += knobMode === 'turn' ? sweep(e, st) : -dy;
      while (st.acc >= per) { st.acc -= per; turn(1); }
      while (st.acc <= -per) { st.acc += per; turn(-1); }
    },
    up: (e, st) => {                          // double tap = push
      if (st.moved) return;
      const now = performance.now();
      if (now - lastTap < 350) {
        send(def.id, 'set', 1);
        send(def.id, 'set', 0);
        g.classList.add('pushed');
        setTimeout(() => g.classList.remove('pushed'), 150);
        lastTap = 0;
      } else {
        lastTap = now;
      }
    },
  });
  let wheelAcc = 0;
  onWheel(hit, (px) => {
    wheelAcc += px;
    while (wheelAcc >= WHEEL_STEP) { wheelAcc -= WHEEL_STEP; turn(1); }
    while (wheelAcc <= -WHEEL_STEP) { wheelAcc += WHEEL_STEP; turn(-1); }
  });
}

function makeButton(g, def, sizes, c, send) {
  const rect = el('rect', { x: -def.w / 2, y: -def.h / 2, width: def.w, height: def.h, rx: 1.1, class: 'btn' }, g);
  if (def.label) el('text', { y: 0.9, class: 'btnlabel' }, g, def.label);
  if (def.caption) el('text', { x: def.w / 2 + 2, y: 1, class: 'caption' }, g, def.caption);
  c.draw = () => g.classList.toggle('lit', c.value > 0);   // the light is the firmware's, not the finger's
  onDrag(rect, {
    down: () => { g.classList.add('down'); send(def.id, 'set', 1); },
    up: () => { g.classList.remove('down'); send(def.id, 'set', 0); },
  });
}

const MAKERS = { knob: makeKnob, fader: makeFader, selector: makeSelector, encoder: makeEncoder, button: makeButton };

// Creates one control inside `parent` (an <svg> or <g>). `send(id, kind, value)`
// reports hardware activity; the returned object's set() shows a position
// reported back by the firmware.
export function makeControl(parent, def, sizes, send) {
  const g = el('g', { transform: `translate(${def.x} ${def.y})`, class: `ctl ${def.type}`, 'data-id': def.id }, parent);
  const c = {
    id: def.id, type: def.type, def, el: g, value: 0, fine: 0, held: false, movedAt: -1e9,
    draw() {},
    touched() { this.movedAt = performance.now(); },
    // A position reported back by the firmware. A control under a finger
    // stays where the finger put it, and for a moment afterwards too: the
    // firmware's reports run a frame or two behind and would tug it back.
    set(v) {
      if (this.held || v === this.value || performance.now() - this.movedAt < 250) return;
      this.value = v;
      this.draw();
    },
  };
  MAKERS[def.type](g, def, sizes, c, send);
  c.draw();
  return c;
}

// Draws the whole panel. Returns a Map of control id -> control.
export function buildPanel(svg, layout, send) {
  const { width: W, height: H } = layout.panel;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.replaceChildren();
  el('rect', { x: 0, y: 0, width: W, height: H, rx: 3, class: 'plate' }, svg);

  for (const s of layout.sections || []) {
    el('rect', { x: s.x, y: s.y, width: s.w, height: s.h, rx: 3, class: 'section' }, svg);
    el('text', { x: s.x + 5.5, y: s.y + 7.5, class: s.accent === 'soft' ? 'stitle soft' : 'stitle' }, svg, s.title.toUpperCase());
  }
  for (const t of layout.texts || []) {
    el('text', { x: t.x, y: t.y, 'font-size': t.size, 'text-anchor': t.anchor || 'start', class: `ptext ${t.style || ''}` }, svg, t.text);
  }
  if (layout.display) {
    const d = layout.display, b = d.bezel || 0;
    el('rect', { x: d.x - b, y: d.y - b, width: d.w + 2 * b, height: d.h + 2 * b, rx: 1, class: 'bezel' }, svg);
  }

  const controls = new Map();
  for (const def of layout.controls) controls.set(def.id, makeControl(svg, def, layout.sizes, send));
  return controls;
}
