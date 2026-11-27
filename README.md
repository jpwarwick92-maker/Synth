# Synth

Firmware for a DIY 8-voice subtractive poly synth, and a browser prototype that runs
that same firmware with an on-screen control panel, so the sound engine, the display
and the panel layout can be played before any hardware exists.

## What is here

| Path | What it is |
|---|---|
| `firmware/` | The firmware core in portable C++: sound engine, oscillator engines, arpeggiator, vocoder, effects, MIDI handling, the parameter table, control logic, patches and panel take-over, soft-control modes and display drawing. No heap, no standard library, nothing platform-specific. |
| `targets/web/` | Thin wrapper that builds the core as a WebAssembly module for the browser. |
| `targets/teensy/` | Thin wrapper that builds the core for the Teensy 4.1 (PlatformIO). Compile-checked on every push; not yet run on a board. |
| `web/` | The page. It is fake hardware only: it draws the panel, reports raw control positions and MIDI bytes, shows the firmware's display buffer, lights each control's label in the colour the firmware asks for, and plays its audio. It can also plug a microphone into the firmware's audio input, send it MIDI clock, and pass on what leaves by its MIDI out. |
| `layouts/a3.json` | The panel layout in millimetres. The page draws whatever this file says. |
| `tests/native/` | Engine and display tests that render audio and display frames with no browser. `make test` leaves the frames, and a few of the renders as sound files, in `build/out`. |
| `tests/page/` | Page tests in a headless browser (mouse, touch, MIDI). |
| `docs/controls.md` | Every control with its MIDI controller number, generated from the firmware's table. |
| `hardware/` | KiCad 10 projects for the circuit boards (clock hub main board, one-LED tile) and the shared `Synth` parts library. See `hardware/README.md`. |

## The rule that keeps the code portable

Everything the real synth will run lives in `firmware/` and is built three ways from
the same files: natively for the tests, as WebAssembly for the page, and for the Teensy.
The page never decides what a control does. It reports "pot 10 is at 2731" or "button 36
went down", exactly as panel hardware would, and shows the pixels the firmware draws. An
on-screen knob never moves by itself, because a real one cannot.

Rules for code in `firmware/`:

- no memory allocation, at start-up or while playing;
- audio in blocks of at most 128 samples. Safe at any sample rate; made for 44.1 kHz
  (the Teensy) and 48 kHz (the page asks the browser for it). Above 48 kHz the longest
  delay is shorter than its setting says and the reverb's lines are scaled down;
- only `<stdint.h>` and `<stddef.h>`; maths comes from `fw_math.h`;
- the display is drawn with pixels, lines, fills and a bitmap font only, in a fixed
  set of colours named by what they are for (the frame is 4 bits per pixel), and no
  text smaller than 2 mm tall on the real display (font scale 2);
- large tables and delay lines are 16-bit where that is inaudible, to fit the
  Teensy's 1 MB of RAM. They stay in RAM: reading them from flash in the audio loop
  is too slow.

## Build and run

Needs `make`, `g++`, `clang++` with the WebAssembly linker (`lld`), Node 22 and Python 3.

```sh
make test        # engine and display tests
make site        # build the page into build/site
make serve       # serve it on http://localhost:8080
make sanitize    # the same tests with the compiler's memory checker on (slower)
make page-test   # a long run of the browser build, then the page tests
                 # (needs Playwright: cd tests/page && npm install && npx playwright install chromium)
make docs        # regenerate docs/controls.md
```

Open the page in Chrome, plug in a MIDI keyboard, press Start and allow MIDI.
The page must be served from `localhost` or over HTTPS; browsers do not allow MIDI
or the audio engine on a plain file.

Teensy compile check: `pip install platformio && pio run -d targets/teensy`.

## Using the page

- Knobs, selectors and encoders: press and circle the pointer round the control and it
  turns with you. The **Knobs** button switches to dragging up and down instead.
- Scrolling over any control also moves it (two fingers on a trackpad, or a mouse wheel).
- Hold Shift for fine movement.
- Selectors: you can also tap a stop's label. Encoders: double-tap to push.
- Faders follow the pointer along their slot.
- **Zoom display** shows the display enlarged with every pixel visible. On the panel it
  is drawn at its true physical size, which on most screens is fewer pixels than the real
  320x240 display has.
- **1:1 size** shows the panel at true physical size. It asks once for calibration
  against a bank card held to the screen.
- Each control's label stands in for a back-lit label on the real panel. It is plain
  warm white normally, and glows red to amber or blue when the control is off a loaded
  patch (see Patches below).
- **Rough pots** (among the test aids) feeds the firmware pot readings as real hardware
  gives them: a few counts of jitter and a trace of the neighbouring pot. The firmware
  smooths them, so nothing should look or sound different with it on.
- Below the panel are test aids that are not part of the synth: stand-in pitch-bend
  and mod wheels, a Test note button that holds a chord, a sustain-pedal switch and Panic.
  They behave as a keyboard set to the channel the synth plays, and Panic is sent on
  every channel.
- **Mic in** plugs this computer's microphone into the synth's audio input (use
  headphones, or the speakers feed back). The dot beside it is the light by the input
  jack. **Test clock** sends MIDI clock into the synth's MIDI in at the tempo in the box
  beside it, as a DAW or a clock hub would; pressing it again sends stop.
- **MIDI out** in the top bar picks the device the synth's MIDI out jack is plugged
  into. A MIDI keyboard, or a DAW sending notes and clock, arrives at the synth's MIDI in.

## What the prototype covers

The whole panel (layout A3) driving the voice: two oscillators plus sub and noise,
mixer, multimode filter (12 or 24 dB) with drive, filter and amp envelopes, one LFO,
glide, and poly, mono and unison modes. The display shows a home screen with a live
scope and a visual for each control as it moves.

**Panel features added while prototyping.**

- **Drift** makes each voice slightly and slowly out of tune with the others, in pitch
  and filter cutoff, as the voices of an analog poly are. At zero every voice is exact.
- **Spread** fans the voices out between the two speakers.
- **Supersaw** is the fifth stop on each Wave selector: seven saws, with Macro setting
  how far apart they are tuned.
- **DRAW** is the sixth stop on each Wave selector: a wave you draw yourself (see
  "The drawn wave" below). The Wave selectors and LFO Shape show a small picture of each
  wave instead of its name, with a doodle line for DRAW and a stepped line for random.
- **Sync** restarts oscillator 2 with every cycle of oscillator 1. With Sync on, the
  Mod page's Env>Pitch moves oscillator 2 only, which gives the classic sync sweep.
- **X-Mod** lets oscillator 2 bend oscillator 1's phase, even with oscillator 2's
  fader down.
- **Arp** has its own section: Mode, Octaves, Tempo, Gate, Hold and Chord.
- **LFO Sync** is a three-stop selector in the LFO section. FREE: one LFO that takes no
  notice of the keys. KEY: it starts again when a note begins from silence. VOICE:
  every voice has its own LFO, each a little out of step and a little off the others'
  pace, so a held chord shimmers instead of moving as one.
- **Chorus** is a knob in the Voice section, with a Type switch beside it. CHORUS is two
  slowly swept copies, one per side. ENSEMBLE is three, a third of a cycle apart, swept
  by a slow and a fast wave at once: a swirl with no one wobble to be heard in it.
- **Sub pitch** is a selector beside the Sub fader: two octaves below oscillator 1, one
  below, a fifth above, or an octave above. Sub detune (up to 25 cents either way) is
  on the Engine page.
- **Chord** remembers the shape of a chord. Hold a chord and press Chord; after that
  each single key plays that chord, moved to the key. With the arp running, one key
  arpeggiates it. Press Chord again to forget it.
- **Ladder filter** is a third setting of the filter slope on the Engine page
  (12 dB, 24 dB, Ladder): four stages with feedback round all of them, low-pass only,
  losing some bass as resonance rises. The Type selector does nothing while it is chosen.

The layout keeps knob, selector and encoder centres at least 22 mm apart and fader
centres at least 17 mm apart, and the page tests fail if any label overlaps another
label, a control, or its section's outline. The sections sit on a grid: two rows of
equal height on five shared columns, with the same gap between every pair of
neighbours. The panel is 530 x 152 mm.

**The display.** Every control has a picture that follows it: turn a control and its
picture changes, or says in words why there is nothing to show (a delay with its mix
at zero, an oscillator playing a flat drawing). The tests enforce this for every panel control and
every value on the encoder pages, from the default settings, with further tests for
the settings under which a control does nothing. On an encoder page the picture takes
the place of the scope for a moment after an encoder is turned.

The display has two sets of colours, chosen on the Setup page. **Amber** is ambers and
orange on black, for a bare display. **White** is warm whites told apart by brightness,
for a display behind amber acrylic, which turns white into amber by itself. The page's
**Amber acrylic** button lays that sheet over the on-screen display so the white set can
be judged as it will look.

**Oscillator engines.** Each oscillator has an Engine button that steps through four
engines. The Wave selector and the timbre controls mean something different under each:

| Engine | Wave selector | Macro | Color |
|---|---|---|---|
| VA | saw, square, triangle, sine, supersaw, DRAW | shapes the wave (below) | none |
| FM | ratio 1:1, 2:1, 3:1, 3.5:1, 4:1, 0.5:1 | FM amount | feedback |
| Wavetable | table 1 to 5, and table 6: the drawing | position in the table | none |
| Pluck | exciter: noise, soft, snap, string, pick, the drawing | damping | decay |

Oscillator 1's Macro is the panel knob. Its Color, and oscillator 2's Macro and Color,
are on page 2 of the Engine mode.

**Macro shapes the wave** under the VA engine, whichever wave is selected. At zero each
wave is exactly what it was before Macro shaped it (tested bit for bit), and costs no more.

| Wave | Macro (display name) | What it does |
|---|---|---|
| Saw | SOFTEN | Rounds the corners, like the slower reset of an analogue oscillator: a filter that follows the note, from 256 times its pitch down to 1.5 times. |
| Square | PULSE WIDTH | As before. |
| Triangle | SHARK FIN | Bends the ramps into capacitor curves, from a triangle to a full shark fin (a filter that follows the note, from 0 to 1 times its pitch). |
| Sine | SQUARE UP | Adds a square's odd harmonics (3rd to 15th), more of them as Macro rises: a pure sine at zero, close to a square at full. |
| Supersaw | SPREAD | How far apart the seven saws are tuned, as before. |
| DRAW | SOFTEN | Rounds the drawing's corners, as on the saw. |

Each shape is held to the loudness of its Macro-zero wave as Macro turns, and each
keeps high notes clean (nothing louder than -60 dB between the harmonics). On a
patch, Macro now matters wherever it rests on a saw, triangle or sine.

**The drawn wave.** Each patch has a drawing: 16 points joined by straight lines, one
cycle of a wave, shared by both oscillators. A new patch starts with a saw. It is
drawn on the third Engine page:

| Encoder | Turn | Push |
|---|---|---|
| 1 | moves along the 16 points | paint mode on or off: moving along then copies the height onto each point passed |
| 2 | raises or lowers the point | puts the point on the centre line |
| 3 | chooses a starting shape: saw, square, triangle, sine, flat | replaces the drawing with it |
| 4 | | smooths the drawing a little |

The page shows the drawing with the point being edited picked out, and says so when
neither oscillator is playing it. Notes that are sounding follow the drawing as it
changes, a moment behind, with a 20 ms crossfade so nothing clicks. The drawing is
turned into band-limited tables (one per octave) so it stays clean on high notes;
that takes longer than an audio block, so it is done in spare time between other work
(`fw::work()`, called by the Teensy's main loop and by the page between audio blocks).
A drawing is played at about a saw's loudness, whatever its shape; a flat one is
silent. Under the pluck engine the string is struck with the drawing's shape, at the
other exciters' loudness. The drawing is never an FM modulator (its partials would
spread far above the note and fold back), so under FM the sixth stop is a 0.5:1 ratio.

**Soft-control modes.** A mode button puts four values on the encoders. Pressing the
lit button steps to the mode's next page if it has one, then returns home. Pushing an
encoder restores its default.

| Button | Page | Encoders |
|---|---|---|
| FX 1 | Delay | time, tone, feedback, mix |
| FX 2 | 1: Compressor + reverb | compressor, reverb size, reverb tone, reverb mix |
| FX 2 | 2: Vocoder | amount, formant, consonants, smooth |
| Mod | Mod | LFO fade-in, velocity to cutoff, filter envelope to pitch, duck |
| Engine | 1: Voice | unison detune, sub detune, bend range, filter slope (12 dB, 24 dB, ladder) |
| Engine | 2: Osc engines | osc 1 Color, (empty), osc 2 Macro, osc 2 Color |
| Engine | 3: Draw | the patch's drawing (see "The drawn wave") |
| Patch | 1: Patch | slot (push to load), save here (push), guide lights, manual (push) |
| Patch | 2: Name | cursor, (empty), letter, (empty) |
| Patch | 3: Setup | display colours (amber, or white for behind amber acrylic); label lights (steady or dance) |
| Patch | 4: MIDI | clock (internal, MIDI jack, USB), channel (all, 1 to 16), thru, clock out |
| Patch | 5: Input | source (line, mic), gain (0 to 60 dB), send to (off, mix, effects), (empty) |

**Settings.** The MIDI and Input pages hold settings about the instrument, not the
sound: they are kept over a restart and a patch never changes them.

**Clock.** With Clock on INT the arp and the duck run at the Tempo knob's setting. On
MIDI or USB they follow the MIDI clock arriving at that input, and the Tempo knob is not
used (its picture says so, and shows the clock's tempo).

- The arp's sixteenth notes fall on the clock's grid. The tempo is the average spacing
  of the last beat's ticks, and the arp steps on a position that runs on smoothly at that
  tempo and is nudged towards each tick, so ticks that arrive a few milliseconds early
  or late (as they do from a computer) still give even steps.
- **Start** begins the pattern again on the next tick, and nothing plays until that
  tick. **Stop** holds the arp silent until start or continue; notes latched by Hold
  stay latched. **Continue** carries on from where it stopped, in the beat and in the
  pattern. A **song position** message moves the beat to the sender's playhead.
- After a stop, if the clock itself has gone as well (a DAW usually stops sending it), a
  fresh chord is not kept waiting: it plays from the Tempo knob. What was latched before
  the stop stays silent.
- A key pressed up to a quarter of a step late still gets that step; later than that it
  waits for the next one.
- If no clock is arriving (none yet, or it has stopped coming for half a second) the
  Tempo knob is used meanwhile, so the synth is never silent for want of a cable.
- Clock at the input that is not selected is ignored.

**MIDI in and out.** There are two connections, the MIDI jacks and USB. Notes and
controllers are played from either, on the Channel set (or on every channel).
Changing Channel lets go of every note and the pedal, since their note-offs may now
arrive on a channel that is no longer listened to.
With **Thru** on, every note and controller message that arrives is passed on, whatever
its channel: both inputs to the MIDI out jack, and the MIDI jack's input to USB (never
USB back to USB). With **Clock Out** on, the clock the synth is running on is sent too:
its own, 24 ticks to a beat at the Tempo knob's setting, when it keeps time itself; the
incoming clock with its start, stop and continue when it is following. System-exclusive
messages are not passed on. If Thru is switched off while notes are held, or a message
could not be passed on because nothing was reading the output, "all notes off" is sent
on every channel, so nothing further down the chain is left sounding. The synth's own
clock is made in the audio blocks, so on the Teensy each tick can leave up to 2.9 ms
late; the tempo it carries is exact. On the page, a device that is both a MIDI input
and the one picked for MIDI out is not listened to while it is the output: the synth
would be handed its own messages back, round and round. Use one bus each way.

**Audio input.** The firmware takes a stereo input beside its output. **Send To**
chooses where it goes: nowhere, straight to the output beside the synth's own sound
(Mix), or into the effects with it (FX). The Volume knob turns it up and down with the
synth: at the knob's usual setting the input passes at its own level. A line input is
stereo; a microphone is one channel and comes out on both sides. **Gain** is what the firmware asks of the preamp in
front of the converter, so on real hardware it is analog gain. The Input page shows the
level against the converter's limit, and the light by the jack is dark with no signal,
amber with one, and red for nearly half a second whenever the input reaches the limit.
With Send To off the output is exactly what it would be with nothing connected. The
vocoder listens to this input whatever Send To is set to.

**Vocoder** (FX 2, second page). The synth's sound, shaped by a voice at the audio
input: play notes, speak into the microphone, and the notes take on the words. The voice
is split into 16 bands from 140 Hz to 7 kHz and the level of each is followed; the
synth's sound is split into the same bands and each is turned up and down by the voice's.
It sits between the voices and the effects, on one channel (the chorus and reverb after
it can widen it again), and its four values are stored in patches.

- **Vocoder** is the amount: how much of the synth's own sound is replaced by the
  vocoded sound. At 0 it is out of the path altogether. At 100% nothing is heard unless
  a voice is arriving.
- **Formant** moves the voice's shape up or down the synth's sound, up to an octave
  either way: up sounds smaller and brighter, down larger and darker.
- **Consonants** adds hiss to the synth's top three bands, so S, T, F and SH are heard
  even on a dull sound. It is added only for breath sounds (a voice whose strength is
  nearly all at the top), and only while a note is held.
- **Smooth** sets how fast each band's level falls when the voice does, from 8 ms
  (crisp, every syllable distinct) to about a third of a second (slurred).
- With the vocoder on, its page shows the sixteen bands moving with the voice, and says
  why if nothing can be heard: no signal at the input, or no note held.
- It works best on a bright, steady sound: saw or supersaw, filter well open, sustain up.
  Set the input's Gain so the light by the jack is amber, never red. Leave Send To off
  unless the plain voice should be heard as well.

**Patches.** Sixteen slots. Every main control is a real pot, fader or switch, so
loading a patch cannot move it. Instead:

- **Saving** stores every value except master volume and Hold, and the drawing.
  Patches saved by builds before the drawing (patch version 1 or 2) are not loaded:
  their slots read as empty. An empty slot saves on
  one push of encoder 2; a used slot asks for a second push within three seconds.
- **Loading** (push encoder 1, or send a MIDI program change) sets the sound from the
  stored values. Any control that is somewhere else is then *following*: the sound uses
  the stored value, and the control is ignored until it takes over.
- **Take-over.** A pot or fader takes over when it is moved through the stored value, or
  lands within 2.5% of its travel of it (1% for Detune), so the sound never jumps.
  Touching one that has not got there shows a picture of where the panel and the patch
  are and which way to go. A selector or switch takes over on any change.
- **Guide lights.** After a load, the label of each control that is off the patch
  glows, steadily. A control that has to come up is deep red when it is a long way
  below the patch and warms to amber as it closes in; one that has to come down is deep
  cold blue far off and pales as it nears. At the patch's value the label goes back to
  plain warm white, so when no label is coloured the panel matches the patch. A
  selector's light stays on until it is on the patch's stop, even though it is already
  live. Encoder 3 switches the lights between On, Tint (a dim amber on anything still
  following) and Off.
- **Manual** (push encoder 4) drops the patch: every control goes live where it stands.
- The home screen shows the slot and name, with `*` once the sound has changed.
- A MIDI controller message for a panel parameter is treated like a one-control load:
  the control follows the incoming value until it is moved through it (or, for a
  selector, changed). That goes for master volume too.

**Dance** (Setup page, encoder 3) makes the label lights move with the sound instead of
staying plain. Each section's labels follow what that section is doing: the LFO's rise
and fall with it; the envelope faders light in turn as a note passes through Attack,
Decay, Sustain and Release; Cutoff follows the filter envelope's sweep; a ripple runs
from the oscillators through the mixer to the filter on every note; the arp's flash on
each step; Volume follows the level. Everything stays between a dim amber and a bright
cream. While a loaded patch has controls to bring back, the dance stops and the guide
lights show. On hardware this mode changes the LEDs' current many times a second, so
their supply has to be kept well away from the audio's.

On the Teensy the slots sit in the on-board EEPROM for now, which is why there are
sixteen; a memory card or the board's flash would hold far more.

**Arpeggiator and Hold.** The arp plays the held notes as sixteenth notes: up, down,
up-down, random or as played, over 1 to 4 octaves, at 40 to 240 BPM. **Hold** keeps
notes sounding after the keys are let go; the next chord replaces them. It works with
the arp on or off. While the arp runs, the sustain pedal acts as a temporary Hold.
Neither Hold nor Chord comes back on by itself after a restart.

**The LFO's MACRO destination** (it was PW) moves whatever Macro does on that oscillator:
pulse width on a square, the saw's softness, the shark fin, the sine's squaring, the
supersaw's spread, the drawing's softness, FM amount, wavetable position, pluck damping.
With Macro at zero only the upper half of the sweep acts.

**When the processor is short of time.** A target that can time its audio (the Teensy)
reports how much of each block's time was used. If that stays above about 85% the
engine drops voices one at a time, down to two at the fewest, and gives them back
after a calm spell, instead of falling behind. Saving the state (the control
positions and settings, a second after the last change) waits for silence, for up to
half a minute, because on real hardware a save can interrupt the sound. The page's
storage costs the sound nothing, so there it does not wait. Saving a patch is written
at once on either: it is asked for by hand, and must not be lost.

**Compressor** (FX 2 page) turns the loud parts down and everything back up, so one
note and a full chord come out nearer the same level and soft notes and tails are
lifted. Its one control lowers the threshold, raises the ratio (to 4:1) and adds
make-up gain together. At zero it is out of the path altogether.

**Duck** (Mod page) dips the level just after every beat and lets it come back before
the next, as a compressor keyed from a kick drum does. The beat is the arp's Tempo, or
the MIDI clock's when one is being followed (the dips then fall on that clock's beats).
On the synth's own clock, while the arp runs the beat is counted from its steps, four to
a beat, so the two cannot slip apart; otherwise (the arp off, or no key held) it runs on
by itself, starting again with any note played from silence. The first
hundredth of a second of each beat is let through, so a note played on the beat keeps
its attack.

The order is vocoder, chorus, delay, reverb, compressor, duck, then the output
limiter.

**Output** is stereo once Spread, the chorus or the reverb is in use. With Spread at
zero and every effect off, both channels carry exactly the same signal.

Not built yet: sending the input through the voices' filter, loading patches
from files, and the iPad version (Safari on iPad has no Web MIDI, so it needs a small
wrapper app).

## Publishing

`.github/workflows/build.yml` runs the tests and the Teensy compile on every push, and
publishes `build/site` to GitHub Pages on pushes to `main`. Pages must be switched on
once in the repository settings, with **GitHub Actions** as the source.

Third-party material is listed in `THIRD_PARTY.md`.
