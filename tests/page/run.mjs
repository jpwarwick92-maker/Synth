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
// The layout file the page draws: positions and sizes below are checked against it.
const layout = JSON.parse(await readFile(path.join(siteDir, 'layouts/a3.json'), 'utf8'));
const PANEL_MM = layout.panel.width;

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

// (the last two flags give the browser a made-up microphone and let the page use it without asking)
const browser = await chromium.launch({
  args: ['--autoplay-policy=no-user-gesture-required', '--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream'],
});

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
  // press on the control itself, slide out along the same bearing, then go round
  const grab = { x: c.x + 10 * Math.sin((fromDeg * Math.PI) / 180), y: c.y - 10 * Math.cos((fromDeg * Math.PI) / 180) };
  const p0 = at(fromDeg);
  await page.mouse.move(grab.x, grab.y);
  await page.mouse.down();
  await page.mouse.move(p0.x, p0.y);
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
// Measured on the drawn panel: every piece of lettering and every control body, in millimetres.
// Returns a description of each overlap, or of lettering that leaves its section or the panel.
const findClashes = (page) => page.evaluate(() => {
  const svg = document.getElementById('panel');
  const mm = svg.viewBox.baseVal.width / svg.getBoundingClientRect().width;
  const box = (e) => { const r = e.getBoundingClientRect(); return { l: r.left * mm, t: r.top * mm, r: r.right * mm, b: r.bottom * mm }; };
  const owner = (e) => e.closest('.ctl')?.dataset.id || '';
  const hits = (a, b, gap) => a.l < b.r - gap && b.l < a.r - gap && a.t < b.b - gap && b.t < a.b - gap;
  const texts = [...svg.querySelectorAll('text')].map((e) => ({ what: `"${e.textContent}"`, id: owner(e), ...box(e) }));
  const bodies = [...svg.querySelectorAll('.cap, .enc, .btn, .tslot, .fcap, .bezel')].map((e) => ({ what: owner(e) || 'display', id: owner(e), ...box(e) }));
  const frame = box(svg.querySelector('.plate'));
  const out = [];
  for (let i = 0; i < texts.length; i++) {
    const t = texts[i];
    if (t.l < frame.l || t.r > frame.r || t.t < frame.t || t.b > frame.b) out.push(`${t.what} runs off the panel`);
    for (let j = i + 1; j < texts.length; j++) if (hits(t, texts[j], 0.15)) out.push(`${t.what} on ${texts[j].what}`);
    for (const b of bodies) if (b.id !== t.id && hits(t, b, 0.15)) out.push(`${t.what} on ${b.what}`);
  }
  for (let i = 0; i < bodies.length; i++)
    for (let j = i + 1; j < bodies.length; j++)
      if (bodies[i].id !== bodies[j].id && hits(bodies[i], bodies[j], 0)) out.push(`${bodies[i].what} on ${bodies[j].what}`);
  // lettering must also stay inside the section outline it belongs to
  const sections = [...svg.querySelectorAll('.section')].map(box);
  for (const t of texts) {
    if (!t.id) continue;
    const s = sections.find((q) => (t.l + t.r) / 2 > q.l && (t.l + t.r) / 2 < q.r && (t.t + t.b) / 2 > q.t && (t.t + t.b) / 2 < q.b);
    if (!s || t.l < s.l + 0.5 || t.r > s.r - 0.5 || t.t < s.t + 0.5 || t.b > s.b - 0.5) out.push(`${t.what} crosses its section's outline`);
  }
  return out;
});

// The display's colours (the amber palette): the accent for names and titles, the line colour for curves, the ink for values.
const ACCENT = [0xff, 0x6a, 0x1f], LINE = [0xff, 0xb0, 0x00], INK = [0xff, 0xe9, 0xc4];

try {
  await mkdir(shotDir, { recursive: true });
  const context = await browser.newContext({ viewport: { width: 1500, height: 900 } });

  // ------------------------------------------------------------------ AC1
  section('AC1  the panel is drawn from the layout file');
  {
    const { page, errors } = await open(context, { start: false });
    const count = (sel) => page.locator(sel).count();
    check(await count('#panel .ctl.knob') === 17, '17 knobs', String(await count('#panel .ctl.knob')));
    check(await count('#panel .ctl.selector') === 12, '12 rotary selectors', String(await count('#panel .ctl.selector')));
    check(await count('#panel .ctl.toggle') === 2, '2 lever switches', String(await count('#panel .ctl.toggle')));
    check(await count('#panel .ctl') === layout.controls.length, 'one drawn control for every entry in the layout file');
    check(await count('#panel .ctl.fader') === 12, '12 faders', String(await count('#panel .ctl.fader')));
    check(await count('#panel .ctl.encoder') === 4, '4 push-encoders', String(await count('#panel .ctl.encoder')));
    check(await count('#panel .ctl.button') === 9, '9 buttons', String(await count('#panel .ctl.button')));
    check(await count('#rear .ctl.knob') === 1, 'rear-panel volume knob');

    // the display sits where the layout says, at 4:3
    const lcd = await page.locator('#lcd').boundingBox(), panel = await page.locator('#panel').boundingBox();
    const mm = panel.width / PANEL_MM;
    check(Math.abs((lcd.x - panel.x) / mm - layout.display.x) < 0.5 && Math.abs((lcd.y - panel.y) / mm - layout.display.y) < 0.5, 'display position', JSON.stringify(lcd));
    check(Math.abs(lcd.width / mm - 48.96) < 0.3 && Math.abs(lcd.height / mm - 36.72) < 0.3, 'display size');
    check(errors.length === 0, 'no page errors before start', errors.join(' | '));

    section('layout rules: spacing between controls, and no lettering on top of anything');
    {
      const round = layout.controls.filter((c) => ['knob', 'selector', 'encoder', 'toggle'].includes(c.type));
      const faders = layout.controls.filter((c) => c.type === 'fader');
      const tight = [];
      const minKnob = layout.rules.knobSpacingMin, minFader = layout.rules.faderSpacingMin;
      for (let i = 0; i < round.length; i++)
        for (let j = i + 1; j < round.length; j++) {
          const d = Math.hypot(round[i].x - round[j].x, round[i].y - round[j].y);
          if (d < minKnob) tight.push(`${round[i].id}/${round[j].id} ${d.toFixed(1)} mm`);
        }
      for (let i = 0; i < faders.length; i++)
        for (let j = i + 1; j < faders.length; j++) {
          const d = Math.hypot(faders[i].x - faders[j].x, faders[i].y - faders[j].y);
          if (d < minFader) tight.push(`${faders[i].id}/${faders[j].id} ${d.toFixed(1)} mm`);
        }
      for (const f of faders)
        for (const r of round)
          if (Math.abs(f.x - r.x) < 18 && Math.abs(f.y - r.y) < f.travel / 2 + 12) tight.push(`${f.id}/${r.id} too close`);
      check(tight.length === 0, `knob, selector, encoder and switch centres at least ${minKnob} mm apart; faders at least ${minFader} mm`, tight.join(', '));

      // The sections sit on a grid: rows of equal height, the same columns in every row, one gap throughout.
      const rows = new Map();
      for (const sec of layout.sections) {
        if (!rows.has(sec.y)) rows.set(sec.y, []);
        rows.get(sec.y).push(sec);
      }
      const edges = [...rows.values()].map((r) => r.sort((p, q) => p.x - q.x).flatMap((sec) => [sec.x, sec.x + sec.w]).join(' '));
      check(rows.size === 2 && new Set(edges).size === 1, 'every upright edge of a section in one row lines up with one in the other', edges.join(' | '));
      check(new Set(layout.sections.map((sec) => sec.h)).size === 1, 'the sections are all the same height');
      const gaps = new Set();
      for (const r of rows.values()) for (let k = 1; k < r.length; k++) gaps.add(r[k].x - (r[k - 1].x + r[k - 1].w));
      const ys = [...rows.keys()].sort((p, q) => p - q);
      gaps.add(ys[1] - (ys[0] + layout.sections[0].h));
      check(gaps.size === 1, 'the gap between neighbouring sections is the same everywhere, across and down', [...gaps].join(', '));
      const first = [...rows.values()][0];
      check(first[0].x === layout.panel.width - (first[first.length - 1].x + first[first.length - 1].w), 'the margin is the same at both ends of the panel');

      // every control, with its lettering, sits inside one section and inside the panel
      const outside = layout.controls.filter((c) => !layout.sections.some((s) => c.x > s.x + 3 && c.x < s.x + s.w - 3 && c.y > s.y + 8 && c.y < s.y + s.h - 3)).map((c) => c.id);
      check(outside.length === 0, 'every control is inside a section', outside.join(', '));

      // measured on the drawn panel: boxes of all lettering and all control bodies
      const clashes = await findClashes(page);
      check(clashes.length === 0, 'no label overlaps another label, a control or a section outline', clashes.slice(0, 12).join('; '));
    }
    await page.close();

    // the overlap check must be able to fail: push one knob's label onto its neighbour
    const broken = await open(context, {
      start: false,
      tweak: (p) => p.route('**/layouts/a3.json', async (route) => {
        const res = await route.fetch();
        const json = await res.json();
        json.controls.find((c) => c.id === 'filter_env_amount').x -= 14;
        await route.fulfill({ response: res, json });
      }),
    });
    const found = await findClashes(broken.page);
    check(found.length > 0, 'the overlap check notices a knob moved onto its neighbour', found.slice(0, 3).join('; '));
    await broken.page.close();

    // change one position in the layout file only: the control moves
    const moved = await open(context, {
      start: false,
      tweak: (p) => p.route('**/layouts/a3.json', async (route) => {
        const res = await route.fetch();
        const json = await res.json();
        json.controls.find((c) => c.id === 'osc1_macro').x = 70;
        await route.fulfill({ response: res, json });
      }),
    });
    const t = await moved.page.locator('[data-id="osc1_macro"]').getAttribute('transform');
    const macroY = layout.controls.find((c) => c.id === 'osc1_macro').y;
    check(t === `translate(70 ${macroY})`, 'editing the layout file moves the control', t);
    await moved.page.close();
  }

  // ------------------------------------------------------------------ start-up
  section('start-up: firmware loads, layout and firmware agree, display runs');
  const { page, errors } = await open(context);
  {
    const problems = await page.evaluate(() => window.__synth.problems);
    check(problems.length === 0, 'every layout control exists in the firmware and the reverse', problems.join(' | '));
    check((await page.evaluate(() => window.__synth.firmwareControls().length)) === 57, '57 firmware controls');
    const f0 = await page.evaluate(() => window.__synth.frames);
    await sleep(500);
    const f1 = await page.evaluate(() => window.__synth.frames);
    check(f1 - f0 >= 8, 'display updates about 30 times a second', `${f1 - f0} frames in 0.5 s`);
    check(await lcdCount(page, ACCENT, 0, 30) > 100, 'AC5 home screen shows the patch name');
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
      // the title ("-- INIT") sits in the middle of the frame, within columns 114..206; map to canvas columns
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
      let title = 0;   // pixels in the title's colour, the display's accent
      for (let i = 0; i < d.length; i += 4) if (Math.abs(d[i] - 0xff) < 12 && Math.abs(d[i + 1] - 0x6a) < 12) title++;
      return { shown: !document.getElementById('zoom').hidden, title };
    });
    check(zoom.shown && zoom.title > 400, 'Zoom display shows the frame at 2x with crisp pixels', JSON.stringify(zoom));
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
    await page.evaluate(() => window.__synth.midi([0xb0, 71, 26]));   // the sound's resonance back near its default (the knob stays at the top, following)

    await arc(page, 'osc1_octave', 0, 80, 18);            // point at +2
    check(await pos(page, 'osc1_octave') === 4, 'selector: points at the stop the cursor is nearest', String(await pos(page, 'osc1_octave')));
    await arc(page, 'osc1_octave', 80, 0, 18);            // back to 0
    check(await pos(page, 'osc1_octave') === 2, 'selector: follows the cursor back', String(await pos(page, 'osc1_octave')));

    const e0 = await pos(page, 'enc2');
    await arc(page, 'enc2', -45, 45);                     // 90 degrees = 6 clicks
    const turned = ((await pos(page, 'enc2')) - e0 + 65536) % 65536;
    check(turned === 5 || turned === 6, 'encoder: one click per 15 degrees of turn', String(turned));

    // the arpeggiator's own panel controls, and the Sync switch
    await arc(page, 'arp_mode', -85, -51, 18);            // from OFF to UP
    check(await pos(page, 'arp_mode') === 1, 'arp Mode selector: six stops, turned to UP', String(await pos(page, 'arp_mode')));
    await arc(page, 'arp_mode', -51, -85, 18);
    check(await pos(page, 'arp_mode') === 0, 'arp Mode back to OFF', String(await pos(page, 'arp_mode')));
    const sw = await centre(page, 'osc_sync');
    await page.mouse.click(sw.x, sw.y);
    check(await pos(page, 'osc_sync') === 1, 'Sync switch: a tap flips it on', String(await pos(page, 'osc_sync')));
    {
      // the LFO's Sync selector: Free, Key, Voice
      const stop3 = async (name) => {
        const box = await page.locator('[data-id="lfo_sync"] text.stop', { hasText: new RegExp(`^${name}$`) }).boundingBox();
        await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2);
        await sleep(200);
      };
      await stop3('VOICE');
      check(await pos(page, 'lfo_sync') === 2, 'LFO Sync selector: three stops, turned to VOICE', String(await pos(page, 'lfo_sync')));
      check(await lcdCount(page, INK, 0, 44) > 60, 'and the display names the setting');
      await stop3('KEY');
      check(await pos(page, 'lfo_sync') === 1, 'then to KEY', String(await pos(page, 'lfo_sync')));
      await stop3('FREE');
      check(await pos(page, 'lfo_sync') === 0, 'and back to FREE', String(await pos(page, 'lfo_sync')));
    }
    check(await page.locator('[data-id="osc_sync"]').evaluate((e) => e.classList.contains('lit')), 'Sync switch shows it is on');
    await page.mouse.click(sw.x, sw.y);
    check(await pos(page, 'osc_sync') === 0, 'Sync switch: a second tap flips it off', String(await pos(page, 'osc_sync')));
    await sleep(1700);

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
        check(await lcdCount(page, INK, 0, 44) > 60, 'AC6 a moved control shows its name and value on the display');
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
    check(await lcdCount(page, ACCENT, 0, 30) > 100 && await lcdCount(page, INK, 0, 44) === 0, 'AC6 the home screen returns after the visual');
  }

  // ------------------------------------------------------------------ soft controls
  section('AC7  Engine page: button lights, encoders change values');
  {
    await press(page, 'btn_engine');
    await sleep(150);
    check(await page.locator('[data-id="btn_engine"]').evaluate((e) => e.classList.contains('lit')), 'Engine button lights');
    const shown = await lcdCount(page, INK, 40, 70);
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
    check(await lcdCount(page, INK, 44, 70) > 100, 'FX 1 is a live page with values on the encoders');
    await page.screenshot({ path: path.join(shotDir, 'page_fx1.png') });
    await press(page, 'btn_fx1');
    await sleep(150);
    check(!(await page.locator('[data-id="btn_fx1"]').evaluate((e) => e.classList.contains('lit'))), 'pressing the lit button returns home');

    const lit = (id) => page.locator(`[data-id="${id}"]`).evaluate((e) => e.classList.contains('lit'));
    await press(page, 'btn_osc1_engine');
    await sleep(150);
    check(await lit('btn_osc1_engine'), 'an oscillator engine button lights when its engine is no longer VA');
    check(await lcdCount(page, INK, 0, 44) > 60, 'and the display names the engine');
    for (let i = 0; i < 3; i++) { await press(page, 'btn_osc1_engine'); await sleep(60); }
    await sleep(150);
    check(!(await lit('btn_osc1_engine')), 'three more presses come back round to VA');

    await page.evaluate(() => window.__synth.midi([0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100]));   // hold a chord...
    await sleep(150);
    await press(page, 'btn_chord');                                                                 // ...and have it remembered
    await sleep(150);
    check(await lit('btn_chord'), 'Chord lights once a held chord has been learned');
    await page.evaluate(() => window.__synth.midi([0x80, 60, 0, 0x80, 64, 0, 0x80, 67, 0]));
    await sleep(1700);
    await page.evaluate(() => window.__synth.midi([0x90, 50, 100]));                                 // one key
    await sleep(300);
    check(await lcdCount(page, LINE, 222, 236) === 3 * 64, 'with Chord on, one key sounds three voices', String(await lcdCount(page, LINE, 222, 236)));
    await page.evaluate(() => window.__synth.midi([0x80, 50, 0]));
    await press(page, 'btn_chord');
    await sleep(1700);
    check(!(await lit('btn_chord')), 'Chord switches off again');

    await press(page, 'btn_hold');
    await sleep(150);
    check(await lit('btn_hold'), 'Hold lights when pressed');
    await page.evaluate(() => window.__synth.midi([0x90, 60, 100, 0x80, 60, 0]));   // a key pressed and let go
    await sleep(1700);
    check(await lcdCount(page, LINE, 222, 236) === 64, 'with Hold on, a note keeps sounding after its key is released', String(await lcdCount(page, LINE, 222, 236)));
    await press(page, 'btn_hold');
    await sleep(900);
    check(!(await lit('btn_hold')) && await lcdCount(page, LINE, 222, 236) === 0, 'Hold off lets it go');
    await sleep(900);
  }

  // ------------------------------------------------------------------ MIDI
  section('AC9  a MIDI controller number sets the value shown; the real control stays put and its label lights');
  const IDLE = 0x96876a;                                // the firmware's plain label light
  const led = (pg, id) => pg.evaluate((i) => window.__synth.led(i), id);
  const labelFill = (pg, id) => pg.locator(`[data-id="${id}"] text.label`).evaluate((e) => getComputedStyle(e).fill.match(/\d+/g).map(Number));
  const push = async (pg, id) => {                      // double tap = encoder push
    const c = await centre(pg, id);
    await pg.mouse.click(c.x, c.y);
    await pg.mouse.click(c.x, c.y);
    await sleep(250);
  };
  {
    const before = await pos(page, 'filter_keytrack');
    check(await led(page, 'filter_keytrack') === IDLE, 'an untouched control has a plain label light', (await led(page, 'filter_keytrack')).toString(16));
    const plain = await labelFill(page, 'filter_keytrack');
    check(plain[0] > 150 && plain[0] > plain[2], 'and its label is painted warm white', plain.join(','));
    const shown = await page.evaluate(() => window.__synth.lcdHash());
    await page.evaluate(() => window.__synth.midi([0xb0, 86, 100]));
    await sleep(300);
    check(await page.evaluate(() => window.__synth.lcdHash()) !== shown, 'CC 86 brings up the Keytrack picture');
    check(await pos(page, 'filter_keytrack') === before, 'the knob itself has not moved', `${before} -> ${await pos(page, 'filter_keytrack')}`);
    const seen = new Set();
    let warm = true;
    for (let i = 0; i < 12; i++) {
      const v = await led(page, 'filter_keytrack');
      seen.add(v);
      warm = warm && ((v >> 16) & 255) > 200 && (v & 255) < 30;
      await sleep(70);
    }
    check(!seen.has(IDLE) && warm, 'its label light turns red to amber: move up to reach the value', [...seen].map((v) => v.toString(16)).join(' '));
    check(seen.size === 1, 'and holds steady: no blinking', String(seen.size));
    const lit = await labelFill(page, 'filter_keytrack');
    check(lit[0] > 2 * lit[2], 'the label is painted in the light\'s colour', lit.join(','));
    await drag(page, 'filter_keytrack', -80);            // up through the value sent (knobs are in drag mode here)
    await sleep(350);
    check(await led(page, 'filter_keytrack') === IDLE && await pos(page, 'filter_keytrack') > 3224, 'turning the knob through the value takes over and the light goes plain',
      `${(await led(page, 'filter_keytrack')).toString(16)} at ${await pos(page, 'filter_keytrack')}`);

    await page.evaluate(() => window.__synth.midi([0xb0, 89, 127]));   // voice mode to UNI
    await sleep(300);
    check(await pos(page, 'voice_mode') === 0 && await led(page, 'voice_mode') !== IDLE, 'CC 89 leaves the Mode selector on POLY, with its light on');
    const stop = async (id, name) => {
      const box = await page.locator(`[data-id="${id}"] text.stop`, { hasText: new RegExp(`^${name}$`) }).boundingBox();
      await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2);
      await sleep(300);
    };
    await stop('voice_mode', 'MONO');
    check(await pos(page, 'voice_mode') === 1 && await led(page, 'voice_mode') !== IDLE, 'one click: the selector is live but still off the value, light stays on');
    await stop('voice_mode', 'UNI');
    check(await led(page, 'voice_mode') === IDLE, 'on the value\'s stop the light goes plain');
    await stop('voice_mode', 'POLY');
  }

  section('rough pots: jittery readings with a trace of crosstalk change nothing that can be seen');
  {
    await sleep(1800);                                  // any picture has gone: home screen
    const saves = await page.evaluate(() => window.__synth.saves);
    await page.click('#rough');
    check(await page.locator('#rough').getAttribute('aria-pressed') === 'true', 'Rough pots switches on');
    await sleep(2500);                                  // some 250 scans of every pot
    check(await lcdCount(page, INK, 0, 44) === 0, 'no control\'s picture comes up by itself', String(await lcdCount(page, INK, 0, 44)));
    const quiet = await page.evaluate(() => window.__synth.saves);
    check(quiet - saves <= 1, 'and the firmware does not keep saving as if controls were moving', `${saves} -> ${quiet}`);
    const before = await pos(page, 'lfo_depth');
    await drag(page, 'lfo_depth', -60);                 // a real turn still gets through
    await sleep(400);
    check(await pos(page, 'lfo_depth') > before + 1000, 'a knob that is turned still moves', `${before} -> ${await pos(page, 'lfo_depth')}`);
    check(await lcdCount(page, INK, 0, 44) > 60, 'and its picture comes up');
    const turned = await pos(page, 'lfo_depth');
    await sleep(2200);
    check(await pos(page, 'lfo_depth') === turned, 'the on-screen knob is not tugged about by the firmware\'s rough reading of it', `${turned} -> ${await pos(page, 'lfo_depth')}`);
    check(await lcdCount(page, INK, 0, 44) === 0, 'and the picture goes away again once the knob is still');
    await drag(page, 'lfo_depth', 60);
    await sleep(300);
    await page.click('#rough');
    await sleep(1800);
  }

  section('patches: saved from the Patch page, loaded without moving a control, lights guide the panel back');
  {
    const lit = (id) => page.locator(`[data-id="${id}"]`).evaluate((e) => e.classList.contains('lit'));
    await sleep(1700);
    await press(page, 'btn_patch');
    await sleep(200);
    check(await lit('btn_patch'), 'Patch button lights');
    await page.screenshot({ path: path.join(shotDir, 'page_patch.png') });
    await push(page, 'enc4');                           // Manual: every control live where it stands
    const labelled = layout.controls.filter((d) => ['knob', 'fader', 'selector', 'toggle'].includes(d.type)).map((d) => d.id);
    const offs = [];
    for (const id of labelled) if (await led(page, id) !== IDLE) offs.push(id);
    check(labelled.length === 43 && offs.length === 0, 'after Manual every label light is plain', offs.join(', '));

    const saves = await page.evaluate(() => window.__synth.patchSaves);
    await drag(page, 'enc1', -24);                      // two clicks: slot 3
    await push(page, 'enc2');                           // save here
    await page.waitForFunction((n) => window.__synth.patchSaves > n, saves, { timeout: 3000 }).catch(() => {});
    const stored = await page.evaluate(() => JSON.parse(localStorage.getItem('synth.patches.v1') || '[]'));
    check(stored.length === 16 && typeof stored[2] === 'string' && stored.filter(Boolean).length === 1, 'the patch is stored in slot 3 of 16', JSON.stringify(stored.map((x) => !!x)));

    const cutSaved = await pos(page, 'filter_cutoff');
    await drag(page, 'filter_cutoff', 60);              // turn Cutoff well down
    await sleep(300);
    const cutMoved = await pos(page, 'filter_cutoff');
    check(cutMoved < cutSaved - 800, 'set-up: Cutoff turned down', `${cutSaved} -> ${cutMoved}`);
    await push(page, 'enc1');                           // load slot 3
    await sleep(300);
    check(await pos(page, 'filter_cutoff') === cutMoved, 'loading does not move the knob');
    const v = await led(page, 'filter_cutoff');
    check(v !== IDLE && ((v >> 16) & 255) > 200 && (v & 255) < 30, 'Cutoff\'s label lights red to amber: turn up to reach the patch', v.toString(16));
    await page.screenshot({ path: path.join(shotDir, 'page_patch_guiding.png') });
    await drag(page, 'filter_cutoff', -10);             // a little way, not there yet
    await sleep(300);
    const nearer = await led(page, 'filter_cutoff');
    check(nearer !== IDLE && ((nearer >> 8) & 255) > ((v >> 8) & 255), 'part of the way back, the light is still on and has warmed towards amber', `${v.toString(16)} -> ${nearer.toString(16)}`);
    check(await lcdCount(page, INK, 100, 130) > 80, 'and the display shows the pick-up picture', String(await lcdCount(page, INK, 100, 130)));
    await drag(page, 'filter_cutoff', -80);             // through the stored value
    await sleep(300);
    check(await led(page, 'filter_cutoff') === IDLE && await pos(page, 'filter_cutoff') >= cutSaved, 'through the patch\'s value the knob takes over and the light goes plain',
      `${(await led(page, 'filter_cutoff')).toString(16)} at ${await pos(page, 'filter_cutoff')} (patch ${cutSaved})`);
    await sleep(1700);
    await press(page, 'btn_patch');                     // the Name page
    await press(page, 'btn_patch');                     // the Setup page
    await sleep(200);
    await page.screenshot({ path: path.join(shotDir, 'page_setup.png') });
    // the amber acrylic is the panel's, not the firmware's: it tints what is shown, not the firmware's frame
    const shown = () => page.evaluate(() => { const c = document.getElementById('lcd'); const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let blue = 0; for (let i = 2; i < d.length; i += 4) blue += d[i]; return blue; });
    const frame = await page.evaluate(() => window.__synth.lcdHash());
    const blueBare = await shown();
    await page.click('#acrylic-toggle');
    await sleep(200);
    check(await shown() < blueBare * 0.5, 'Amber acrylic takes most of the blue out of the display as shown', `${blueBare} -> ${await shown()}`);
    check(await page.evaluate(() => window.__synth.lcdHash()) === frame, 'and leaves the firmware\'s own frame alone');
    await page.click('#acrylic-toggle');
    for (let i = 0; i < 3; i++) await press(page, 'btn_patch');     // MIDI, Input, home
    await sleep(200);
    check(!(await lit('btn_patch')), 'five more presses return home');
  }

  section('lights: Dance, chosen on the Setup page, moves the labels with the sound');
  {
    await sleep(1700);
    for (let i = 0; i < 3; i++) await press(page, 'btn_patch');     // Patch, Name, Setup
    await drag(page, 'enc3', -12);                                  // one click: Lights to Dance
    await sleep(200);
    await page.screenshot({ path: path.join(shotDir, 'page_setup_dance.png') });
    for (let i = 0; i < 3; i++) await press(page, 'btn_patch');     // MIDI, Input, home
    await sleep(600);
    const fills = async (id, ms) => {
      const seen = new Set();
      for (let t = 0; t < ms; t += 60) { seen.add((await labelFill(page, id)).join(',')); await sleep(60); }
      return seen;
    };
    check((await fills('lfo_rate', 1200)).size >= 4, 'with nothing playing, the LFO labels rise and fall with the LFO');
    check((await fills('filter_cutoff', 600)).size === 1, 'and a label with nothing to follow holds still');
    const restLight = await led(page, 'filter_cutoff');
    check(restLight !== IDLE && ((restLight >> 16) & 255) < 120, 'at rest a label is a dim amber', restLight.toString(16));
    await page.click('#test-note');
    const moving = await fills('volume', 900);
    await page.screenshot({ path: path.join(shotDir, 'page_dance.png') });
    check(moving.size >= 3, 'with notes sounding, the Volume label follows the level', String(moving.size));
    const seenColours = [];
    for (const id of ['filter_cutoff', 'volume', 'mix_osc1', 'aenv_sustain']) seenColours.push(await labelFill(page, id));
    check(seenColours.every((c) => c[0] >= c[1] && c[1] >= c[2]), 'every colour is an amber: no blue, no green', JSON.stringify(seenColours));
    await page.click('#test-note');
    await sleep(300);
    for (let i = 0; i < 3; i++) await press(page, 'btn_patch');
    await drag(page, 'enc3', 12);                                   // back to Steady
    for (let i = 0; i < 3; i++) await press(page, 'btn_patch');
    await sleep(500);
    check(await led(page, 'filter_cutoff') === IDLE && await led(page, 'lfo_rate') === IDLE, 'Steady puts every label back to plain');
  }

  section('connections: MIDI thru, clock in and out, and the audio input, set on the MIDI and Input pages');
  {
    await sleep(1700);
    const sent = (port, match) => page.evaluate(([p, m]) => window.__synth.midiOut.filter((e) => e.port === p).flatMap((e) => e.bytes).filter((b) => m.includes(b)).length, [port, match]);
    const clear = () => page.evaluate(() => { window.__synth.midiOut.length = 0; });
    const quietScope = await lcdCount(page, LINE, 40, 208);         // the home page's scope with nothing sounding
    await page.evaluate(() => window.__synth.midi([0x90, 60, 100, 0x80, 60, 0]));
    await sleep(300);
    check(await sent(0, [0x90, 0x80]) === 0, 'with Thru off, notes are not passed to MIDI out');
    for (let i = 0; i < 4; i++) await press(page, 'btn_patch');     // Patch, Name, Setup, MIDI
    await sleep(200);
    check(await lcdCount(page, ACCENT, 96, 112) === 0, 'the MIDI page starts out on the synth\'s own clock');
    await drag(page, 'enc1', -12);                                  // Clock: MIDI
    await drag(page, 'enc3', -12);                                  // Thru: on
    await drag(page, 'enc4', -12);                                  // Clock Out: on
    await page.evaluate(() => window.__synth.midi([0x90, 60, 100, 0x80, 60, 0]));
    await sleep(300);
    check(await sent(0, [0x90, 0x80]) === 2 && await sent(1, [0x90, 0x80]) === 2, 'with Thru on, they leave by MIDI out and by USB');
    await page.evaluate(() => window.__synth.midi([0x91, 60, 100, 0x81, 60, 0], 1));
    await sleep(300);
    check(await sent(0, [0x91, 0x81]) === 2 && await sent(1, [0x91, 0x81]) === 0, 'what arrives by USB goes to MIDI out but not back to USB');
    check(await page.locator('#midi-out option').count() >= 1, 'the bar offers somewhere to plug MIDI out into');
    await clear();
    await page.click('#clock');                                     // the test clock: 96 BPM into MIDI in
    await sleep(2500);
    check(await lcdCount(page, ACCENT, 96, 112) > 200, 'Test clock: the page says the synth is following MIDI clock', String(await lcdCount(page, ACCENT, 96, 112)));
    await page.screenshot({ path: path.join(shotDir, 'page_midi.png') });
    await clear();
    await sleep(2000);
    const ticks = await sent(0, [0xf8]);
    check(ticks >= 66 && ticks <= 88, 'Clock Out passes the clock on: two seconds at 96 BPM is about 77 ticks', String(ticks));
    await page.click('#clock');
    await sleep(400);
    check(await sent(0, [0xfc]) === 1, 'stopping the test clock sends stop, and that is passed on too');
    check(await lcdCount(page, ACCENT, 96, 112) === 0, 'and the page no longer says it is following', String(await lcdCount(page, ACCENT, 96, 112)));
    await drag(page, 'enc1', 12);                                   // everything back as it was
    await drag(page, 'enc3', 12);
    await drag(page, 'enc4', 12);

    // the test aids are a keyboard on the synth's own channel, and Panic is heard on every channel
    const voices = () => lcdCount(page, LINE, 222, 236);            // the status strip: 64 pixels a voice
    for (let i = 0; i < 3; i++) await drag(page, 'enc2', -12);      // Channel: 3
    await sleep(300);
    check(await page.evaluate(() => window.__synth.channel) === 3, 'the Channel encoder sets channel 3', String(await page.evaluate(() => window.__synth.channel)));
    await page.click('#test-note');
    await sleep(500);
    check(await voices() === 4 * 64, 'on channel 3, Test note still plays its four notes', String(await voices()));
    await page.click('#panic');
    await sleep(400);
    check(await voices() === 0, 'and Panic still silences them', String(await voices()));
    await page.click('#test-note');
    await sleep(400);
    await drag(page, 'enc2', 12);                                   // Channel: 2, with the chord held
    await sleep(600);
    check(await voices() === 0, 'changing Channel with notes held lets them go', String(await voices()));
    check(await page.locator('#test-note').getAttribute('aria-pressed') === 'false', 'and the Test note button shows it');
    for (let i = 0; i < 2; i++) await drag(page, 'enc2', 12);       // back to ALL
    await sleep(300);
    check(await page.evaluate(() => window.__synth.channel) === 0, 'Channel is back on ALL');

    await press(page, 'btn_patch');                                 // the Input page
    await sleep(300);
    check(await page.evaluate(() => window.__synth.inputLight) === 0, 'nothing plugged into the input: its light is dark');
    await page.evaluate(() => window.__synth.inputTone(0.3));       // a test tone into the audio input
    await sleep(500);
    check(await page.evaluate(() => window.__synth.inputLight) === 0xff9a00, 'a signal at a healthy level: the light is amber',
      (await page.evaluate(() => window.__synth.inputLight)).toString(16));
    check(await lcdCount(page, LINE, 98, 114) > 1500, 'and the Input page shows its level', String(await lcdCount(page, LINE, 98, 114)));
    await page.screenshot({ path: path.join(shotDir, 'page_input.png') });
    await drag(page, 'enc2', -150);                                 // gain up: the stand-in preamp overloads its converter
    await sleep(500);
    check(await page.evaluate(() => window.__synth.inputLight) === 0xe00000, 'too much gain: the light goes red',
      (await page.evaluate(() => window.__synth.inputLight)).toString(16));
    check(await lcdCount(page, ACCENT, 98, 114) > 1500, 'and the page shows the clip', String(await lcdCount(page, ACCENT, 98, 114)));
    for (let i = 0; i < 6; i++) await drag(page, 'enc2', 150);      // gain back to 0 dB
    await drag(page, 'enc3', -12);                                  // Send To: MIX
    await press(page, 'btn_patch');                                 // home
    await sleep(1700);
    const heardScope = await lcdCount(page, LINE, 40, 208);
    check(heardScope > quietScope + 150, 'sent to MIX, the input is heard: the scope shows it', `${quietScope} -> ${heardScope}`);
    await page.evaluate(() => window.__synth.inputTone(0));
    for (let i = 0; i < 5; i++) await press(page, 'btn_patch');
    await drag(page, 'enc3', 12);                                   // Send To: off
    await press(page, 'btn_patch');
    await sleep(2200);
    check(await lcdCount(page, LINE, 40, 208) <= quietScope + 20 && await page.evaluate(() => window.__synth.inputLight) === 0,
      'with the tone unplugged the output and the light go quiet', String(await lcdCount(page, LINE, 40, 208)));
    // the vocoder: the held chord is heard only while something arrives at the input
    await page.evaluate(() => window.__synth.midi([0xb0, 106, 127]));   // vocoder amount to full
    await page.click('#test-note');
    await sleep(1900);
    check(await lcdCount(page, LINE, 40, 208) <= quietScope + 20, 'vocoder on, nothing at the input: the held chord is silent', String(await lcdCount(page, LINE, 40, 208)));
    await page.evaluate(() => window.__synth.inputTone(0.3, 700));
    await sleep(600);
    const spoken = await lcdCount(page, LINE, 40, 208);
    check(spoken > quietScope + 150, 'a tone at the input opens it: the chord sounds', `${quietScope} -> ${spoken}`);
    await press(page, 'btn_fx2');
    await press(page, 'btn_fx2');                                   // the Vocoder page: the bands, live
    await sleep(300);
    check(await lcdCount(page, LINE, 96, 152) > 300, 'the Vocoder page shows the bands the tone falls in', String(await lcdCount(page, LINE, 96, 152)));
    await page.screenshot({ path: path.join(shotDir, 'page_vocoder.png') });
    await press(page, 'btn_fx2');
    await page.evaluate(() => { window.__synth.inputTone(0); window.__synth.midi([0xb0, 106, 0]); });
    await page.click('#test-note');
    await sleep(2200);
    // the Mic button plugs in the browser's microphone (a made-up one here)
    await page.click('#mic');
    await page.waitForFunction(() => document.getElementById('mic').getAttribute('aria-pressed') === 'true', null, { timeout: 5000 }).catch(() => {});
    check(await page.locator('#mic').getAttribute('aria-pressed') === 'true', 'Mic in: the microphone is plugged in');
    const heard = await page.waitForFunction(() => window.__synth.inputLight !== 0, null, { timeout: 6000 }).then(() => true, () => false);
    check(heard, 'and its signal reaches the firmware\'s input');
    await page.click('#mic');
    check(await page.locator('#mic').getAttribute('aria-pressed') === 'false', 'pressing again unplugs it');
    // a double click must not leave a second microphone plugged in behind the button's back
    await page.waitForFunction(() => window.__synth.inputLight === 0, null, { timeout: 6000 }).catch(() => {});
    await page.dblclick('#mic');
    await sleep(1500);
    if (await page.locator('#mic').getAttribute('aria-pressed') === 'true') await page.click('#mic');
    const dark = await page.waitForFunction(() => window.__synth.inputLight === 0, null, { timeout: 6000 }).then(() => true, () => false);
    check(await page.locator('#mic').getAttribute('aria-pressed') === 'false' && dark, 'after a double click and switching off, no microphone is left feeding the input');
  }

  section('AC8  test aids: Test note sounds, wheels and Panic send MIDI');
  {
    await sleep(1700);
    const quiet = await lcdCount(page, LINE, 40, 208);
    await page.click('#test-note');
    await sleep(500);
    const loud = await lcdCount(page, LINE, 40, 208);
    check(loud > quiet + 150, 'Test note: the scope shows a waveform', `${quiet} -> ${loud}`);
    check(await lcdCount(page, LINE, 222, 236) === 4 * 64, 'Test note: four voices light in the status strip', String(await lcdCount(page, LINE, 222, 236)));
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
    check(await lcdCount(page, LINE, 222, 236) === 0, 'AC10 Panic silences every voice', String(await lcdCount(page, LINE, 222, 236)));
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
    check(ids.length === 44 && blob && blob.length > 50, 'state is stored for the next visit');
    await page.close();

    const again = await open(context);
    let wrong = [];
    for (const id of ids) if (await pos(again.page, id) !== want[id]) wrong.push(id);
    check(wrong.length === 0, 'all 44 controls are where they were left', wrong.join(', '));
    check(want.filter_cutoff > 3000 && want.osc1_wave === 3, 'and those are the moved positions, not defaults');
    check(await again.page.evaluate(() => localStorage.getItem('synth.state.v1')) === blob, 'saved state is unchanged by the reload');
    check(again.errors.length === 0, 'no page errors', again.errors.join(' | '));
    await again.page.screenshot({ path: path.join(shotDir, 'page.png') });
    // the stored patch is still in its slot, and still the one loaded
    await press(again.page, 'btn_patch');
    await sleep(300);
    check(await lcdCount(again.page, ACCENT, 78, 104) > 300, 'after a reload the Patch page still shows the stored patch as loaded', String(await lcdCount(again.page, ACCENT, 78, 104)));
    for (let i = 0; i < 5; i++) await press(again.page, 'btn_patch');
    await sleep(200);

    // ---------------------------------------------------------------- AC2
    section('AC2  1:1 view uses the calibrated scale');
    await again.page.click('#view-toggle');
    check(await again.page.locator('#cal').evaluate((d) => d.open), 'first use of 1:1 asks for calibration');
    await again.page.locator('#cal-slider').fill('4.25');
    const card = await again.page.locator('#cal-card').boundingBox();
    check(Math.abs(card.width - 85.6 * 4.25) < 1.5 && Math.abs(card.height - 53.98 * 4.25) < 1.5, 'calibration outline is one bank card at the chosen scale', JSON.stringify(card));
    await again.page.click('#cal-save');
    const w = (await again.page.locator('#panel').boundingBox()).width;
    check(Math.abs(w - PANEL_MM * 4.25) < 1, `1:1 panel is ${PANEL_MM} mm at the calibrated scale`, String(w));
    const a = await again.page.locator('[data-id="lfo_rate"] .cap').boundingBox(), b = await again.page.locator('[data-id="lfo_depth"] .cap').boundingBox();
    const xOf = (id) => layout.controls.find((c) => c.id === id).x;
    const apart = xOf('lfo_depth') - xOf('lfo_rate');
    check(Math.abs((b.x - a.x) / 4.25 - apart) < 0.1, `two knobs ${apart} mm apart in the file are ${apart} mm apart on screen`, String((b.x - a.x) / 4.25));
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
