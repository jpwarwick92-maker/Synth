# Synth

Firmware for a DIY 8-voice subtractive poly synth, and a browser prototype that runs
that same firmware with an on-screen control panel, so the sound engine, the display
and the panel layout can be played before any hardware exists.

## What is here

| Path | What it is |
|---|---|
| `firmware/` | The firmware core in portable C++: sound engine, MIDI handling, the parameter table, control logic, soft-control modes and display drawing. No heap, no standard library, nothing platform-specific. |
| `targets/web/` | Thin wrapper that builds the core as a WebAssembly module for the browser. |
| `targets/teensy/` | Thin wrapper that builds the core for the Teensy 4.1 (PlatformIO). Compile-checked on every push; not yet run on a board. |
| `web/` | The page. It is fake hardware only: it draws the panel, reports raw control positions and MIDI bytes, shows the firmware's display buffer and plays its audio. |
| `layouts/a2.json` | The panel layout in millimetres. The page draws whatever this file says. |
| `tests/native/` | Engine and display tests that render audio and display frames with no browser. |
| `tests/page/` | Page tests in a headless browser (mouse, touch, MIDI). |
| `docs/controls.md` | Every control with its MIDI controller number, generated from the firmware's table. |

## The rule that keeps the code portable

Everything the real synth will run lives in `firmware/` and is built three ways from
the same files: natively for the tests, as WebAssembly for the page, and for the Teensy.
The page never decides what a control does. It reports "pot 10 is at 2731" or "button 36
went down", exactly as panel hardware would, and shows the pixels the firmware draws.

Rules for code in `firmware/`:

- no memory allocation, at start-up or while playing;
- audio in blocks of at most 128 samples, at any sample rate;
- only `<stdint.h>` and `<stddef.h>`; maths comes from `fw_math.h`;
- the display is drawn with pixels, lines, fills and a bitmap font only.

## Build and run

Needs `make`, `g++`, `clang++` with the WebAssembly linker (`lld`), Node 22 and Python 3.

```sh
make test        # engine and display tests
make site        # build the page into build/site
make serve       # serve it on http://localhost:8080
make page-test   # page tests (needs Playwright: cd tests/page && npm install && npx playwright install chromium)
make docs        # regenerate docs/controls.md
```

Open the page in Chrome, plug in a MIDI keyboard, press Start and allow MIDI.
The page must be served from `localhost` or over HTTPS; browsers do not allow MIDI
or the audio engine on a plain file.

Teensy compile check: `pip install platformio && pio run -d targets/teensy`.

## Using the page

- Knobs and faders: drag up and down. Hold Shift for fine movement.
- Selectors: drag, or tap a stop's label.
- Encoders: drag to turn, double-tap to push.
- **1:1 size** shows the panel at true physical size. It asks once for calibration
  against a bank card held to the screen.
- Below the panel are test aids that are not part of the synth: stand-in pitch-bend
  and mod wheels, a Test note button that holds a chord, a sustain-pedal switch and Panic.

## What the prototype covers

The whole A2 panel driving the core voice: two oscillators (saw, square with pulse
width, triangle, sine) plus sub and noise, mixer, multimode filter (12 or 24 dB) with
drive, filter and amp envelopes, one LFO, glide, and poly, mono and unison modes.
The display shows a home screen with a live scope and a visual for each control as it
moves. Of the six soft-control modes only **Engine** is live; the rest are placeholders.

Not built yet: effects, arpeggiator, the FM, wavetable and pluck engines, patch storage,
and the iPad version (Safari on iPad has no Web MIDI, so it needs a small wrapper app).

## Publishing

`.github/workflows/build.yml` runs the tests and the Teensy compile on every push, and
publishes `build/site` to GitHub Pages on pushes to `main`. Pages must be switched on
once in the repository settings, with **GitHub Actions** as the source.

Third-party material is listed in `THIRD_PARTY.md`.
