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

// The saw and triangle sets are stored as 16-bit numbers: half the memory
// of floats, with rounding noise far below anything audible. kTabScale is
// the stored value of 1.0; it leaves room for the overshoot a band-limited
// saw has at its edge (about 1.18).
constexpr float kTabScale = 27000.0f;
extern float gSine[kTabLen + 1];
extern int16_t gSaw[kSawLevels][kTabLen + 1];
extern int16_t gTri[kTriLevels][kTabLen + 1];

// ---- wavetable engine ----
// Five tables of four frames each; the oscillator morphs between frames.
// Every frame is stored at six levels of detail (32, 16, 8, 4, 2, 1
// harmonics) for the same reason the saw is: nothing may fold back.
constexpr int kWtTables = 5;
constexpr int kWtFrames = 4;
constexpr int kWtLevels = 6;
constexpr int kWtLen = 512;  // power of two
extern int16_t gWt[kWtTables][kWtFrames][kWtLevels][kWtLen + 1];
int wtLevelFor(float inc);

// Builds every table. Call once at start-up.
void tablesInit();

// Table to use for an oscillator advancing `inc` cycles per sample.
const int16_t* sawTableFor(float inc);
const int16_t* triTableFor(float inc);

// Linear-interpolated read of a 16-bit table, phase in [0, 1). Result in the table's -1..1 scale.
inline float tabRead16(const int16_t* t, float phase) {
  float p = phase * (float)kTabLen;
  int i = (int)p;
  float f = p - (float)i;
  return ((float)t[i] + (float)(t[i + 1] - t[i]) * f) * (1.0f / kTabScale);
}

// Linear-interpolated read, phase in [0, 1).
inline float tabRead(const float* t, float phase) {
  float p = phase * (float)kTabLen;
  int i = (int)p;
  float f = p - (float)i;
  return t[i] + (t[i + 1] - t[i]) * f;
}

}  // namespace fw
