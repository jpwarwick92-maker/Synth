// The lights behind the panel's labels.
//
// Normally each is a plain warm white, or, after a patch is loaded, one of
// the guide colours that show which way to move a control (patch.cpp).
// "Dance" is the other thing they can do: move with the sound. Each
// section's labels follow what that section is doing, so the panel reads as
// a picture of the patch at work rather than a random light show:
//
//   LFO                the LFO itself, rising and falling at its rate
//   Filter and amp     the stage that envelope is in: Attack, Decay,
//   envelope faders    Sustain or Release lights as the note passes through it
//   Velocity           flashes with each note, brighter for a harder hit
//   Cutoff             the filter envelope's sweep
//   Oscillators,       a ripple that starts at the oscillators with every
//   mixer, filter      note and runs along the signal path, over a glow
//                      that follows the output level
//   Voice              how many of the eight voices are sounding
//   Arp                a flash on every step, Tempo brightest
//   Volume             the output level
//
// Everything stays in the panel's own range, from a dim amber through amber
// to a bright cream. The guide lights keep their meaning: while a loaded
// patch has controls to bring back, the dance stops and the guide shows.
//
// On hardware, lightColour() for every control is sent to the LED chain
// after each tick. Changing the LEDs' brightness changes the current they
// draw, many times a second in this mode: the LED supply must be kept away
// from the audio's (see the notes in targets/teensy/src/main.cpp).
#include "firmware.h"

#include "fw_math.h"
#include "synth.h"

namespace fw {

const char* const kLightsNames[LIGHTS_COUNT] = {"STEADY", "DANCE"};

namespace {

constexpr uint32_t kDanceLow = 0x3A2408;   // dim amber: a label at rest
constexpr uint32_t kDanceMid = 0xFF9A00;   // amber
constexpr uint32_t kDanceHigh = 0xFFE9C4;  // bright cream: a peak

uint8_t gMode = LIGHTS_STEADY;
// what the animation needs to remember, moved on by lightsTick()
float gLevel;        // output level, quick to rise and slower to fall
float gVoices;       // share of the eight voices sounding
float gSinceNote;    // seconds since the last note started
float gNoteFlash;    // 1 when a note starts, dying away
float gArpFlash;     // 1 at each arp step, dying away
uint32_t gNotesSeen, gStepsSeen;

uint32_t mix(uint32_t a, uint32_t b, float t) {
  uint32_t out = 0;
  for (int shift = 16; shift >= 0; shift -= 8) {
    float ca = (float)((a >> shift) & 255u), cb = (float)((b >> shift) & 255u);
    out |= (uint32_t)(ca + (cb - ca) * t + 0.5f) << shift;
  }
  return out;
}

// 0 = at rest, 1 = a peak.
uint32_t ramp(float level) {
  level = clampf(level, 0.0f, 1.0f);
  return level <= 0.6f ? mix(kDanceLow, kDanceMid, level * (1.0f / 0.6f)) : mix(kDanceMid, kDanceHigh, (level - 0.6f) * 2.5f);
}

// Which of an envelope's four faders (0 Attack .. 3 Release) the note is passing through now.
int stageOf(const Env& e, float sustain) {
  if (e.stage == ENV_ATTACK) return 0;
  if (e.stage == ENV_RELEASE) return 3;
  return absf(e.level - sustain) < 0.04f ? 2 : 1;
}

float danceLevel(int id) {
  const float rest = 0.12f;
  Env filt, amp;
  float vel = 0.0f;
  bool sounding = gSynth.leadVoice(filt, amp, vel);

  switch (kParams[id].section) {
    case SEC_LFO: return 0.15f + 0.85f * (0.5f + 0.5f * clampf(gSynth.lfoNow(), -1.0f, 1.0f));
    case SEC_FENV:
    case SEC_AENV: {
      if (id == P_VELOCITY) return rest + 0.88f * gNoteFlash * (0.3f + 0.7f * vel);
      if (!sounding) return rest;
      bool filter = kParams[id].section == SEC_FENV;
      const Env& e = filter ? filt : amp;
      int first = filter ? P_FENV_A : P_AENV_A;
      int now = stageOf(e, gParams.norm(first + 2));
      if (id - first != now) return rest + 0.2f * e.level;
      return now == 3 ? 0.25f + 0.75f * e.level : 1.0f;  // Release fades with the note
    }
    case SEC_ARP:
      if (!gSynth.arpRunning()) return rest;
      return rest + (id == P_ARP_TEMPO ? 0.88f : 0.45f) * gArpFlash;
    case SEC_OUTPUT: return rest + 0.88f * gLevel;
    case SEC_VOICE: return rest + 0.55f * gVoices + 0.3f * gNoteFlash;
    default: break;
  }
  if (id == P_CUTOFF) return rest + 0.88f * (sounding ? filt.level : 0.0f);
  // Oscillators, mixer, filter: a ripple along the signal path from each new note.
  float place = kParams[id].section == SEC_OSC1 ? 0.0f : kParams[id].section == SEC_OSC2 ? 0.7f : kParams[id].section == SEC_MIXER ? 1.7f : 2.7f;
  float d = gSinceNote * 9.0f - place;  // the front reaches the filter in about a third of a second
  float wave = d > -1.5f && d < 4.0f ? exp2f_(-3.0f * d * d) : 0.0f;
  return rest + 0.4f * gLevel + 0.6f * wave;
}

}  // namespace

void lightsReset() {
  gMode = LIGHTS_STEADY;
  gLevel = gVoices = gNoteFlash = gArpFlash = 0.0f;
  gSinceNote = 10.0f;
  gNotesSeen = gStepsSeen = 0;
}

int lightsMode() { return gMode; }

void lightsSetMode(int mode) {
  mode = clampi(mode, 0, LIGHTS_COUNT - 1);
  if (mode == gMode) return;
  gMode = (uint8_t)mode;
  stateTouched();
}

void lightsTick(uint32_t dtMs) {
  float dt = (float)dtMs * 0.001f;
  float level = minf(gSynth.peakLevel() * 1.6f, 1.0f);
  gLevel = level > gLevel ? level : gLevel + (level - gLevel) * minf(dt * 8.0f, 1.0f);
  gVoices += ((float)gSynth.activeVoices() * (1.0f / (float)kNumVoices) - gVoices) * minf(dt * 10.0f, 1.0f);
  float fade = exp2f_(-dt * 14.0f);
  gNoteFlash *= fade;
  gArpFlash *= fade;
  gSinceNote = minf(gSinceNote + dt, 10.0f);
  uint32_t notes = gSynth.noteCount(), steps = gSynth.arpStepCount();
  if (notes != gNotesSeen) {
    gNotesSeen = notes;
    gNoteFlash = 1.0f;
    gSinceNote = 0.0f;
  }
  if (steps != gStepsSeen) {
    gStepsSeen = steps;
    gArpFlash = 1.0f;
  }
}

uint32_t lightColour(int index) {
  uint32_t guide = gPatch.led(index);  // 0 for a control with no label light
  if (gMode != LIGHTS_DANCE || guide == 0) return guide;
  // The guide keeps the panel while a loaded patch has controls to bring back.
  if (guide != kLedIdle || (gPatch.guide() == GUIDE_ON && gPatch.offCount() > 0)) return guide;
  return ramp(danceLevel(kPanelParams[index]));
}

}  // namespace fw
