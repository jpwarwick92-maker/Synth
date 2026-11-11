# Controls and MIDI controller numbers

Generated from the firmware's parameter table by `make docs`. Do not edit by hand.

The synth listens on every MIDI channel. Besides the numbers below it responds to
notes with velocity, pitch bend, mod wheel (CC 1, adds LFO depth), sustain pedal (CC 64),
all sound off (CC 120) and all notes off (CC 123).

## Panel controls

| Control id | Section | Name | Type | Stops | CC |
|---|---|---|---|---|---|
| `osc1_wave` | OSC 1 | WAVE | selector | SAW, SQR, TRI, SIN | 20 |
| `osc1_octave` | OSC 1 | OCTAVE | selector | -2, -1, 0, +1, +2 | 21 |
| `osc1_macro` | OSC 1 | PULSE WIDTH | pot |  | 22 |
| `osc2_wave` | OSC 2 | WAVE | selector | SAW, SQR, TRI, SIN | 23 |
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
| `lfo_shape` | LFO | SHAPE | pot |  | 108 |
| `lfo_destination` | LFO | DESTINATION | selector | PITCH, CUTOFF, AMP, PW | 109 |
| `volume` | OUTPUT | VOLUME | pot |  | 7 |

## Engine page (soft controls)

| Parameter id | Encoder | Name | Type | Values | CC |
|---|---|---|---|---|---|
| `unison_detune` | 1 | UNI DETUNE | stepped | 0 to 50 | 110 |
| `sub_octave` | 2 | SUB OCTAVE | stepped | -1 OCT, -2 OCT | 111 |
| `bend_range` | 3 | BEND RANGE | stepped | 1 to 12 | 112 |
| `filter_slope` | 4 | FILT SLOPE | stepped | 12 dB, 24 dB | 113 |

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
| `btn_arp` | button |
| `btn_engine` | button |
| `btn_patch` | button |
| `btn_osc1_engine` | button |
| `btn_osc2_engine` | button |
