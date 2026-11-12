# Controls and MIDI controller numbers

Generated from the firmware's parameter table by `make docs`. Do not edit by hand.

The synth listens on every MIDI channel. Besides the numbers below it responds to
notes with velocity, pitch bend, mod wheel (CC 1, adds LFO depth), sustain pedal (CC 64),
all sound off (CC 120) and all notes off (CC 123).

## Panel controls

| Control id | Section | Name | Type | Stops | CC |
|---|---|---|---|---|---|
| `osc1_wave` | OSC 1 | WAVE | selector | SAW, SQR, TRI, SIN, SUPER | 20 |
| `osc1_octave` | OSC 1 | OCTAVE | selector | -2, -1, 0, +1, +2 | 21 |
| `osc1_macro` | OSC 1 | PULSE WIDTH | pot |  | 22 |
| `osc2_wave` | OSC 2 | WAVE | selector | SAW, SQR, TRI, SIN, SUPER | 23 |
| `osc2_octave` | OSC 2 | OCTAVE | selector | -2, -1, 0, +1, +2 | 24 |
| `osc2_detune` | OSC 2 | DETUNE | pot |  | 25 |
| `mix_osc1` | MIXER | OSC 1 | pot |  | 26 |
| `mix_osc2` | MIXER | OSC 2 | pot |  | 27 |
| `mix_sub` | MIXER | SUB | pot |  | 28 |
| `mix_noise` | MIXER | NOISE | pot |  | 29 |
| `filter_cutoff` | FILTER | CUTOFF | pot |  | 74 |
| `filter_resonance` | FILTER | RESONANCE | pot |  | 71 |
| `filter_env_amount` | FILTER | ENV AMOUNT | pot |  | 85 |
| `filter_keytrack` | FILTER | KEYTRACK | pot |  | 86 |
| `filter_drive` | FILTER | DRIVE | pot |  | 87 |
| `filter_type` | FILTER | TYPE | selector | LP, BP, HP | 88 |
| `voice_glide` | VOICE | GLIDE | pot |  | 5 |
| `voice_mode` | VOICE | MODE | selector | POLY, MONO, UNI | 89 |
| `fenv_attack` | FILTER ENV | ATTACK | pot |  | 102 |
| `fenv_decay` | FILTER ENV | DECAY | pot |  | 103 |
| `fenv_sustain` | FILTER ENV | SUSTAIN | pot |  | 104 |
| `fenv_release` | FILTER ENV | RELEASE | pot |  | 105 |
| `aenv_attack` | AMP ENV | ATTACK | pot |  | 73 |
| `aenv_decay` | AMP ENV | DECAY | pot |  | 75 |
| `aenv_sustain` | AMP ENV | SUSTAIN | pot |  | 79 |
| `aenv_release` | AMP ENV | RELEASE | pot |  | 72 |
| `aenv_velocity` | AMP ENV | VELOCITY | pot |  | 90 |
| `lfo_rate` | LFO | RATE | pot |  | 76 |
| `lfo_depth` | LFO | DEPTH | pot |  | 77 |
| `lfo_shape` | LFO | SHAPE | selector | SINE, TRI, SAW, SQUARE, RANDOM | 108 |
| `lfo_destination` | LFO | DESTINATION | selector | PITCH, CUTOFF, AMP, PW | 109 |
| `volume` | OUTPUT | VOLUME | pot |  | 7 |
| `arp_mode` | ARP | MODE | selector | OFF, UP, DOWN, UP-DOWN, RANDOM, PLAYED | 58 |
| `arp_octaves` | ARP | OCTAVES | selector | 1, 2, 3, 4 | 60 |
| `arp_tempo` | ARP | TEMPO | pot |  | 59 |
| `arp_gate` | ARP | GATE | pot |  | 61 |
| `voice_drift` | VOICE | DRIFT | pot |  | 14 |
| `voice_spread` | VOICE | SPREAD | pot |  | 15 |
| `osc_xmod` | OSC 2 | X-MOD | pot |  | 16 |
| `osc_sync` | OSC 2 | SYNC | selector | OFF, ON | 17 |

## Soft parameters (encoder pages and buttons)

Wave, Macro and Color mean different things under each oscillator engine:
VA: wave, pulse width, none. FM: ratio, FM amount, feedback. Wavetable: table, position, none. Pluck: exciter, damping, decay.

| Parameter id | Page | Name | Values | CC |
|---|---|---|---|---|
| `unison_detune` | ENGINE | UNI DETUNE | 0 to 50 | 110 |
| `sub_octave` | ENGINE | SUB OCTAVE | -1 OCT, -2 OCT | 111 |
| `bend_range` | ENGINE | BEND RANGE | 1 to 12 | 112 |
| `filter_slope` | ENGINE | FILT SLOPE | 12 dB, 24 dB, LADDER | 113 |
| `delay_time` | FX 1 | TIME | 0 to 100 | 46 |
| `delay_tone` | FX 1 | TONE | 0 to 100 | 47 |
| `delay_feedback` | FX 1 | FEEDBACK | 0 to 100 | 48 |
| `delay_mix` | FX 1 | MIX | 0 to 100 | 49 |
| `chorus` | FX 2 | CHORUS | 0 to 100 | 50 |
| `reverb_size` | FX 2 | RVB SIZE | 0 to 100 | 51 |
| `reverb_tone` | FX 2 | RVB TONE | 0 to 100 | 52 |
| `reverb_mix` | FX 2 | RVB MIX | 0 to 100 | 53 |
| `lfo_fade` | MOD | LFO FADE | 0 to 100 | 54 |
| `vel_cutoff` | MOD | VEL>CUTOFF | 0 to 100 | 55 |
| `fenv_pitch` | MOD | ENV>PITCH | 0 to 24 | 56 |
| `lfo_sync` | MOD | LFO SYNC | FREE, KEY | 57 |
| `osc1_color` | OSC 1 | COLOR | 0 to 100 | 62 |
| `osc2_macro` | OSC 2 | MACRO | 0 to 100 | 63 |
| `osc2_color` | OSC 2 | COLOR | 0 to 100 | 114 |
| `osc1_engine` | OSC 1 | ENGINE | VA, FM, WAVETABLE, PLUCK | 115 |
| `osc2_engine` | OSC 2 | ENGINE | VA, FM, WAVETABLE, PLUCK | 116 |
| `hold` | VOICE | HOLD | OFF, ON | 69 |

## Encoders and buttons

| Control id | Type |
|---|---|
| `enc1` | push-encoder |
| `enc2` | push-encoder |
| `enc3` | push-encoder |
| `enc4` | push-encoder |
| `btn_fx1` | button |
| `btn_fx2` | button |
| `btn_mod` | button |
| `btn_engine` | button |
| `btn_patch` | button |
| `btn_osc1_engine` | button |
| `btn_osc2_engine` | button |
| `btn_hold` | button |
| `btn_chord` | button |
