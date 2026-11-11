// Page tests: load the built site in a headless browser and use it the way
// a person would (mouse, touch, MIDI bytes), checking what the page shows.
//
//   node tests/page/run.mjs build/site [screenshot-dir]

import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { createRequire } from 'node:module';
import path from 'node:path';

const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const siteDir = path.resolve(process.argv[2] || 'build/site');
const shotDir = path.resolve(process.argv[3] || 'build/out');

const TYPES = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json', '.wasm': 'application/wasm' };

const server = createServer(async (req, res) => {
  try {
    let p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
    if (p.endsWith('/')) p += 'index.html';
    const file = path.join(siteDir, path.normalize(p));
    if (!file.startsWith(siteDir)) throw new Error('outside site');
    const body = await readFile(file);
    res.writeHead(200, { 'content-type': TYPES[path.extname(file)] || 'application/octet-stream', 'cache-control': 'no-store' });
    res.end(body);
  } catch {
    res.writeHead(404);
    res.end('not found');
  }
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const base = `http://127.0.0.1:${server.address().port}/`;

let checks = 0, fails = 0;
function check(ok, what, detail = '') {
  checks++;
  if (!ok) {
    fails++;
    console.log(`    FAIL  ${what}${detail ? '  -- ' + detail : ''}`);
  }
}
const section = (s) => console.log(s);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const browser = await chromium.launch({ args: ['--autoplay-policy=no-user-gesture-required'] });

async function open(context, { start = true, tweak } = {}) {
  const page = await context.newPage();
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  if (tweak) await tweak(page);
  await page.goto(base);
  await page.waitForFunction(() => window.__synth && window.__synth.loaded);
  if (start) {
    await page.click('#start-btn');
    await page.waitForFunction(() => window.__synth.ready, null, { timeout: 15000 });
  }
  return { page, errors };
}

const pos = (page, id) => page.evaluate((i) => window.__synth.position(i), id);
async function centre(page, id) {
  // the grab area: the round one on knobs, selectors and encoders, else the fader strip or the button
  const round = page.locator(`[data-id="${id}"] circle.hit`);
  const target = (await round.count()) ? round : page.locator(`[data-id="${id}"] .hit, [data-id="${id}"] .btn`);
  const box = await target.first().boundingBox();
  return { x: box.x + box.width / 2, y: box.y + box.height / 2 };
}
async function drag(page, id, dy, opts = {}) {
  const c = await centre(page, id);
  await page.mouse.move(c.x, c.y);
  await page.mouse.down();
  for (let i = 1; i <= 8; i++) await page.mouse.move(c.x, c.y + (dy * i) / 8);
  if (opts.hold) await opts.hold();
  await page.mouse.up();
}
async function press(page, id) {
  const c = await centre(page, id);
  await page.mouse.move(c.x, c.y);
  await page.mouse.down();
  await page.mouse.up();
}
// Counts pixels close to a colour in the firmware's own 320x240 frame, to tell screens apart.
const lcdCount = (page, rgb, y0 = 0, y1 = 240) => page.evaluate(([c, a, b]) => window.__synth.lcdCount(c, a, b), [rgb, y0, y1]);
// Moves the pressed mouse round a control's centre from one clock angle to another (degrees, 0 = up).
async function arc(page, id, fromDeg, toDeg, radius = 20) {
  const c = await centre(page, id);
  const at = (deg) => ({ x: c.x + radius * Math.sin((deg * Math.PI) / 180), y: c.y - radius * Math.cos((deg * Math.PI) / 180) });
  const p0 = at(fromDeg);
  await page.mouse.move(p0.x, p0.y);
  await page.mouse.down();
  const steps = Math.max(4, Math.round(Math.abs(toDeg - fromDeg) / 6));
  for (let i = 1; i <= steps; i++) {
    const p = at(fromDeg + ((toDeg - fromDeg) * i) / steps);
    await page.mouse.move(p.x, p.y);
  }
  await page.mouse.up();
}
const setKnobs = async (page, mode) => {
  if (await page.evaluate(() => window.__synth.knobMode) !== mode) await page.click('#knob-mode');
};
const AMBER = [0xe8, 0xa3, 0x3d], TEAL = [0x7f, 0xe0, 0xc4], WHITE = [0xed, 0xed, 0xed];

try {
  await mkdir(shotDir, { recursive: true });
  const context = await browser.newContext({ viewport: { width: 1500, height: 900 } });

  // ------------------------------------------------------------------ AC1
  section('AC1  the panel is drawn from the layout file');
  {
    const { page, errors } = await open(context, { start: false });
    const count = (sel) => page.locator(sel).count();
    check(await count('#panel .ctl.knob') === 12, '12 knobs', String(await count('#panel .ctl.knob')));
    check(await count('#panel .ctl.selector') === 7, '7 rotary selectors', String(await count('#panel .ctl.selector')));
    check(await count('#panel .ctl.fader') === 12, '12 faders', String(await count('#panel .ctl.fader')));
    check(await count('#panel .ctl.encoder') === 4, '4 push-encoders', String(await count('#panel .ctl.encoder')));
    check(await count('#panel .ctl.button') === 8, '8 buttons', String(await count('#panel .ctl.button')));
    check(await count('#rear .ctl.knob') === 1, 'rear-panel volume knob');

    // the display sits where the layout says, at 4:3
    const lcd = await page.locator('#lcd').boundingBox(), panel = await page.locator('#panel').boundingBox();
    const mm = panel.width / 464;
    check(Math.abs((lcd.x - panel.x) / mm - 365) < 0.5 && Math.abs((lcd.y - panel.y) / mm - 106) < 0.5, 'display position', JSON.stringify(lcd));
    check(Math.abs(lcd.width / mm - 48.96) < 0.3 && Math.abs(lcd.height / mm - 36.72) < 0.3, 'display size');
    check(errors.length === 0, 'no page errors before start', errors.join(' | '));
    await page.close();

    // change one position in the layout file only: the control moves
    const moved = await open(context, {
      start: false,
      tweak: (p) => p.route('**/layouts/a2.json', async (route) => {
        const res = await route.fetch();
        const json = await res.json();
        json.controls.find((c) => c.id === 'osc1_macro').x = 70;
        await route.fulfill({ response: res, json });
      }),
    });
    const t = await moved.page.locator('[data-id="osc1_macro"]').getAttribute('transform');
    check(t === 'translate(70 50)', 'editing the layout file moves the control', t);
    await moved.page.close();
  }

  // ------------------------------------------------------------------ start-up
  section('start-up: firmware loads, layout and firmware agree, display runs');
  const { page, errors } = await open(context);
  {
    const problems = await page.evaluate(() => window.__synth.problems);
    check(problems.length === 0, 'every layout control exists in the firmware and the reverse', problems.join(' | '));
    check((await page.evaluate(() => window.__synth.firmwareControls().length)) === 44, '44 firmware controls');
    const f0 = await page.evaluate(() => window.__synth.frames);
    await sleep(500);
    const f1 = await page.evaluate(() => window.__synth.frames);
    check(f1 - f0 >= 8, 'display updates about 30 times a second', `${f1 - f0} frames in 0.5 s`);
    check(await lcdCount(page, AMBER, 0, 30) > 100, 'AC5 home screen shows the patch name');
    check(await page.locator('#start').isHidden(), 'start overlay closes');
    const delay = await page.locator('#audio-delay').textContent();
    check(/^\d+ ms$/.test(delay), 'AC15 the page shows its audio delay', delay);
    check((await page.locator('#build').textContent()).length > 3, 'AC14 the page shows which build it is');
    check((await page.locator('#midi-status').textContent()).startsWith('Keyboard:'), 'AC10 keyboard status is shown');
  }

  // ------------------------------------------------------------------ display rendering
  section('display: drawn on the screen\'s own pixel grid, with a zoomed view');
  {
    const grid = await page.evaluate(() => {
      const c = document.getElementById('lcd');
      return { w: c.width, h: c.height, cw: c.clientWidth, ch: c.clientHeight, dpr: devicePixelRatio };
    });
    check(grid.w === Math.round(grid.cw * grid.dpr) && grid.h === Math.round(grid.ch * grid.dpr),
      'the visible display has one canvas pixel per screen pixel (nothing dropped by scaling)', JSON.stringify(grid));
    // Hands-on finding: shown smaller than 320 px wide, nearest-pixel scaling
    // lost whole columns. Every lit column of the title must survive.
    const lost = await page.evaluate(() => {
      const c = document.getElementById('lcd'), ctx = c.getContext('2d');
      const d = ctx.getImageData(0, 0, c.width, Math.round(c.height * 30 / 240)).data;
      const lit = new Array(c.width).fill(false);
      for (let i = 0; i < d.length; i += 4) if (d[i] > 40) lit[(i / 4) % c.width] = true;
      // title "001 INIT" spans frame columns 114..206; map to canvas columns
      const a = Math.ceil(116 * c.width / 320), b = Math.floor(204 * c.width / 320);
      let dark = 0, run = 0, longest = 0;
      for (let x = a; x <= b; x++) { if (!lit[x]) { dark++; run++; longest = Math.max(longest, run); } else run = 0; }
      return { dark, longest, width: b - a + 1, canvas: c.width };
    });
    check(lost.canvas < 320, 'this check runs with the display shown smaller than its 320 pixels', JSON.stringify(lost));
    check(lost.dark < lost.width * 0.5, 'the title is still legible when the display is shown small', JSON.stringify(lost));
    await page.click('#zoom-toggle');
    const zoom = await page.evaluate(() => {
      const z = document.getElementById('lcd-zoom'), d = z.getContext('2d').getImageData(0, 0, 640, 60).data;
      let amber = 0;
      for (let i = 0; i < d.length; i += 4) if (Math.abs(d[i] - 0xe8) < 12 && Math.abs(d[i + 1] - 0xa3) < 12) amber++;
      return { shown: !document.getElementById('zoom').hidden, amber };
    });
    check(zoom.shown && zoom.amber > 400, 'Zoom display shows the frame at 2x with crisp pixels', JSON.stringify(zoom));
    await page.screenshot({ path: path.join(shotDir, 'page_zoom.png') });
    await page.click('#zoom-toggle');
    check(await page.locator('#zoom').isHidden(), 'Zoom display toggles off');
  }

  // ------------------------------------------------------------------ turning with the pointer
  section('AC12  knobs, selectors and encoders turn with the pointer (default)');
  {
    check(await page.evaluate(() => window.__synth.knobMode) === 'turn', 'turning is the default');
    const v0 = await pos(page, 'filter_resonance');
    await arc(page, 'filter_resonance', -90, 0);          // a quarter turn clockwise
    const v1 = await pos(page, 'filter_resonance');
    check(Math.abs(v1 - v0 - 1365) < 70, 'knob: a quarter turn of the pointer is a quarter turn of the knob', `${v0} -> ${v1}`);
    await arc(page, 'filter_resonance', 0, -90);          // and back
    const v2 = await pos(page, 'filter_resonance');
    check(Math.abs(v2 - v0) < 70, 'knob: turning back returns to where it started', `${v0} -> ${v1} -> ${v2}`);
    await arc(page, 'filter_resonance', 0, 720, 24);      // two full circles: stops at the end, does not wrap
    check(await pos(page, 'filter_resonance') === 4095, 'knob: stops at the end of its travel', String(await pos(page, 'filter_resonance')));
    await page.evaluate(() => window.__synth.midi([0xb0, 71, 26]));   // put it back near its default

    await arc(page, 'osc1_octave', 0, 80, 18);            // point at +2
    check(await pos(page, 'osc1_octave') === 4, 'selector: points at the stop the cursor is nearest', String(await pos(page, 'osc1_octave')));
    await arc(page, 'osc1_octave', 80, 0, 18);            // back to 0
    check(await pos(page, 'osc1_octave') === 2, 'selector: follows the cursor back', String(await pos(page, 'osc1_octave')));

    const e0 = await pos(page, 'enc2');
    await arc(page, 'enc2', -45, 45);                     // 90 degrees = 6 clicks
    const turned = ((await pos(page, 'enc2')) - e0 + 65536) % 65536;
    check(turned === 5 || turned === 6, 'encoder: one click per 15 degrees of turn', String(turned));

    // scrolling over a control: a trackpad sends many small amounts
    const s0 = await pos(page, 'lfo_rate');
    const c = await centre(page, 'lfo_rate');
    await page.mouse.move(c.x, c.y);
    for (let i = 0; i < 30; i++) await page.mouse.wheel(0, -4);
    const s1 = await pos(page, 'lfo_rate');
    check(Math.abs(s1 - s0 - 410) < 40, 'scrolling 120 px over a knob moves it a tenth of its travel, however the scroll is chopped up', `${s0} -> ${s1}`);
    await page.mouse.wheel(0, 120);
    check(Math.abs((await pos(page, 'lfo_rate')) - s0) < 40, 'scrolling back returns it');
    await sleep(1800);
  }

  // ------------------------------------------------------------------ dragging up and down
  section('AC12  the same controls with "drag up/down" chosen; AC6 moving one shows its visual');
  {
    await setKnobs(page, 'updown');
    check((await page.locator('#knob-mode').textContent()).includes('up/down'), 'the button says which way is active');
    const before = await pos(page, 'filter_cutoff');
    await drag(page, 'filter_cutoff', -60, {
      hold: async () => {
        await sleep(150);
        check(await lcdCount(page, WHITE, 0, 44) > 60, 'AC6 a moved control shows its name and value on the display');
        await page.screenshot({ path: path.join(shotDir, 'page_popup.png') });
      },
    });
    const after = await pos(page, 'filter_cutoff');
    check(after > before + 800, 'knob: dragging up turns it up', `${before} -> ${after}`);

    const f0 = await pos(page, 'mix_sub');
    await drag(page, 'mix_sub', -40);
    check(await pos(page, 'mix_sub') > f0 + 500, 'fader: dragging up raises it', `${f0} -> ${await pos(page, 'mix_sub')}`);

    await page.locator('[data-id="osc1_wave"] rect.hit').nth(1).dispatchEvent('pointerdown');
    check(await pos(page, 'osc1_wave') === 1, 'selector: tapping a stop label turns to it', String(await pos(page, 'osc1_wave')));
    await drag(page, 'osc1_wave', -50);
    check(await pos(page, 'osc1_wave') === 3, 'selector: dragging steps through the stops', String(await pos(page, 'osc1_wave')));

    await sleep(1800);
    check(await lcdCount(page, AMBER, 0, 30) > 100 && await lcdCount(page, WHITE, 0, 44) === 0, 'AC6 the home screen returns after the visual');
  }

  // ------------------------------------------------------------------ soft controls
  section('AC7  Engine page: button lights, encoders change values, placeholders');
  {
    await press(page, 'btn_engine');
    await sleep(150);
    check(await page.locator('[data-id="btn_engine"]').evaluate((e) => e.classList.contains('lit')), 'Engine button lights');
    const shown = await lcdCount(page, WHITE, 40, 70);
    check(shown > 100, 'Engine page shows encoder values');
    await page.screenshot({ path: path.join(shotDir, 'page_engine.png') });
    const snap = () => page.evaluate(() => window.__synth.lcdHash());
    const a = await snap();
    await drag(page, 'enc1', -60);                      // five clicks clockwise
    await sleep(150);
    const b = await snap();
    check(a !== b, 'turning encoder 1 changes the display');
    check(await pos(page, 'enc1') === 5, 'encoder pointer follows the clicks', String(await pos(page, 'enc1')));
    const c = await centre(page, 'enc1');
    await page.mouse.click(c.x, c.y);
    await page.mouse.click(c.x, c.y);                   // double tap = push = back to default
    await sleep(200);
    // (the scope is flat with no notes sounding, so equal frames mean equal values)
    await drag(page, 'enc1', 60);
    await press(page, 'btn_fx1');
    await sleep(150);
    check(await page.locator('[data-id="btn_fx1"]').evaluate((e) => e.classList.contains('lit')), 'FX 1 lights');
    check(!(await page.locator('[data-id="btn_engine"]').evaluate((e) => e.classList.contains('lit'))), 'Engine light goes out');
    await press(page, 'btn_fx1');
    await sleep(150);
    check(!(await page.locator('[data-id="btn_fx1"]').evaluate((e) => e.classList.contains('lit'))), 'pressing the lit button returns home');
    await press(page, 'btn_osc1_engine');
    await sleep(150);
    check(await lcdCount(page, WHITE, 90, 125) > 500, 'oscillator engine button shows "VA only"');
    await sleep(1700);
  }

  // ------------------------------------------------------------------ MIDI
  section('AC9  a MIDI controller number moves the on-screen control');
  {
    await page.evaluate(() => window.__synth.midi([0xb0, 71, 100]));
    await page.waitForFunction(() => window.__synth.position('filter_resonance') === 3224, null, { timeout: 3000 }).catch(() => {});
    check(await pos(page, 'filter_resonance') === 3224, 'CC 71 at 100 puts Resonance at 3224', String(await pos(page, 'filter_resonance')));
    await page.evaluate(() => window.__synth.midi([0xb0, 89, 127]));
    await page.waitForFunction(() => window.__synth.position('voice_mode') === 2, null, { timeout: 3000 }).catch(() => {});
    check(await pos(page, 'voice_mode') === 2, 'CC 89 at 127 turns Mode to UNI', String(await pos(page, 'voice_mode')));
    await page.evaluate(() => window.__synth.midi([0xb0, 89, 0]));
  }

  section('AC8  test aids: Test note sounds, wheels and Panic send MIDI');
  {
    await sleep(1700);
    const quiet = await lcdCount(page, TEAL, 40, 208);
    await page.click('#test-note');
    await sleep(500);
    const loud = await lcdCount(page, TEAL, 40, 208);
    check(loud > quiet + 150, 'Test note: the scope shows a waveform', `${quiet} -> ${loud}`);
    check(await lcdCount(page, TEAL, 222, 236) === 4 * 64, 'Test note: four voices light in the status strip', String(await lcdCount(page, TEAL, 222, 236)));
    await page.screenshot({ path: path.join(shotDir, 'page_playing.png') });
    const box = await page.locator('#mod').boundingBox();
    await page.mouse.move(box.x + box.width / 2, box.y + box.height - 4);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width / 2, box.y + 4);
    await page.mouse.up();
    const modValue = parseFloat(await page.locator('#mod').getAttribute('data-value'));
    check(modValue > 0.9, 'mod wheel stays where it is put', String(modValue));
    const bbox = await page.locator('#bend').boundingBox();
    await page.mouse.move(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    await page.mouse.down();
    await page.mouse.move(bbox.x + bbox.width / 2, bbox.y + 4);
    await page.mouse.up();
    const bendValue = parseFloat(await page.locator('#bend').getAttribute('data-value'));
    check(bendValue === 0.5, 'bend wheel springs back to centre', String(bendValue));
    await page.click('#panic');
    await sleep(300);
    check(await lcdCount(page, TEAL, 222, 236) === 0, 'AC10 Panic silences every voice', String(await lcdCount(page, TEAL, 222, 236)));
    check(await page.locator('#test-note').getAttribute('aria-pressed') === 'false', 'Panic clears Test note');
  }

  // ------------------------------------------------------------------ persistence
  section('AC11  positions and encoder values survive a reload');
  {
    await press(page, 'btn_engine');
    await drag(page, 'enc3', -40);                      // bend range up
    await press(page, 'btn_engine');
    const saves = await page.evaluate(() => window.__synth.saves);
    await page.waitForFunction((s) => window.__synth.saves > s, saves, { timeout: 5000 }).catch(() => {});
    check(await page.evaluate(() => window.__synth.saves) > saves, 'the firmware saves about a second after the last change');
    const ids = await page.evaluate(() => window.__synth.controlIds().filter((i) => !i.startsWith('enc') && !i.startsWith('btn')));
    const want = {};
    for (const id of ids) want[id] = await pos(page, id);
    const blob = await page.evaluate(() => localStorage.getItem('synth.state.v1'));
    check(ids.length === 32 && blob && blob.length > 50, 'state is stored for the next visit');
    await page.close();

    const again = await open(context);
    let wrong = [];
    for (const id of ids) if (await pos(again.page, id) !== want[id]) wrong.push(id);
    check(wrong.length === 0, 'all 32 controls are where they were left', wrong.join(', '));
    check(want.filter_cutoff > 3000 && want.osc1_wave === 3, 'and those are the moved positions, not defaults');
    check(await again.page.evaluate(() => localStorage.getItem('synth.state.v1')) === blob, 'saved state is unchanged by the reload');
    check(again.errors.length === 0, 'no page errors', again.errors.join(' | '));
    await again.page.screenshot({ path: path.join(shotDir, 'page.png') });

    // ---------------------------------------------------------------- AC2
    section('AC2  1:1 view uses the calibrated scale');
    await again.page.click('#view-toggle');
    check(await again.page.locator('#cal').evaluate((d) => d.open), 'first use of 1:1 asks for calibration');
    await again.page.locator('#cal-slider').fill('4.25');
    const card = await again.page.locator('#cal-card').boundingBox();
    check(Math.abs(card.width - 85.6 * 4.25) < 1.5 && Math.abs(card.height - 53.98 * 4.25) < 1.5, 'calibration outline is one bank card at the chosen scale', JSON.stringify(card));
    await again.page.click('#cal-save');
    const w = (await again.page.locator('#panel').boundingBox()).width;
    check(Math.abs(w - 464 * 4.25) < 1, '1:1 panel is 464 mm at the calibrated scale', String(w));
    const a = await again.page.locator('[data-id="lfo_rate"] .cap').boundingBox(), b = await again.page.locator('[data-id="lfo_depth"] .cap').boundingBox();
    check(Math.abs((b.x - a.x) / 4.25 - 24) < 0.1, 'two knobs 24 mm apart in the file are 24 mm apart on screen', String((b.x - a.x) / 4.25));
    check(await again.page.evaluate(() => { const s = document.getElementById('stage'); return s.scrollWidth > s.clientWidth; }), 'the 1:1 panel can be panned');
    await again.page.click('#view-toggle');
    check((await again.page.locator('#panel').boundingBox()).width < 1500, 'toggling back fits the panel to the window');
    await again.page.close();
  }
  check(errors.length === 0, 'no page errors during the session', errors.join(' | '));
  await context.close();

  // ------------------------------------------------------------------ touch
  section('AC12  touch: two controls can be moved at once');
  {
    const touch = await browser.newContext({ viewport: { width: 1180, height: 820 }, hasTouch: true, isMobile: true });
    const { page: tp, errors: terrors } = await open(touch);
    const a = await centre(tp, 'fenv_attack'), b = await centre(tp, 'fenv_release');
    const a0 = await pos(tp, 'fenv_attack'), b0 = await pos(tp, 'fenv_release');
    const cdp = await touch.newCDPSession(tp);
    const pts = (dy1, dy2) => [{ x: a.x, y: a.y + dy1, id: 1 }, { x: b.x, y: b.y + dy2, id: 2 }];
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: pts(0, 0) });
    for (let i = 1; i <= 6; i++) await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: pts(-5 * i, 4 * i) });
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await sleep(200);
    const a1 = await pos(tp, 'fenv_attack'), b1 = await pos(tp, 'fenv_release');
    check(a1 > a0 + 300, 'first finger raised one fader', `${a0} -> ${a1}`);
    check(b1 < b0 - 300, 'second finger lowered another at the same time', `${b0} -> ${b1}`);
    check(terrors.length === 0, 'no page errors', terrors.join(' | '));
    await tp.screenshot({ path: path.join(shotDir, 'page_touch.png') });
    await touch.close();
  }
} catch (e) {
  fails++;
  console.log('    FAIL  test run stopped: ' + (e.stack || e));
} finally {
  await browser.close();
  server.close();
}

console.log(`\n${checks} page checks, ${fails} failed`);
process.exit(fails ? 1 : 0);
