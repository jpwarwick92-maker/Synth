// Wavetables for the oscillators.
//
// The saw and triangle are stored as a set of band-limited tables, each
// holding a fixed number of harmonics. An oscillator reads the richest
// table whose top harmonic still sits below half the sample rate, so no
// harmonic ever folds back as a false tone (aliasing), at any sample rate.
// Square and pulse waves are made from two saw readings.
#pragma once

#include "fw_config.h"

namespace fw {

constexpr int kTabLen = 2048;  // power of two
constexpr int kSawLevels = 17;
constexpr int kTriLevels = 11;

// Harmonic count of each saw level, richest first. Half-octave steps.
extern const int16_t kSawHarmonics[kSawLevels];

extern float gSine[kTabLen + 1];
extern float gSaw[kSawLevels][kTabLen + 1];
extern float gTri[kTriLevels][kTabLen + 1];

// Builds every table. Call once at start-up.
void tablesInit();

// Table to use for an oscillator advancing `inc` cycles per sample.
const float* sawTableFor(float inc);
const float* triTableFor(float inc);

// Linear-interpolated read, phase in [0, 1).
inline float tabRead(const float* t, float phase) {
  float p = phase * (float)kTabLen;
  int i = (int)p;
  float f = p - (float)i;
  return t[i] + (t[i + 1] - t[i]) * f;
}

}  // namespace fw
