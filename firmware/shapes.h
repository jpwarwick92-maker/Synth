// What Macro does to the plain (VA) engine's saw, triangle, sine and
// drawing. At Macro zero each wave is exactly as it was before Macro
// shaped it, and costs no more.
//
//  - Saw and drawing: softened. A low-pass filter that follows the note
//    rounds the corners, as the slower reset of an analogue oscillator
//    does. It runs from 256 times the note's frequency (no audible change)
//    down to 1.5 times it.
//  - Triangle: bent into a shark fin. A high-pass filter that follows the
//    note curves each ramp like a capacitor charging and discharging. (This
//    is the same wave as a square through a low-pass filter, the classic
//    shark fin, but cheaper: one table read instead of two.) It runs from
//    0 to 1 times the note's frequency.
//  - Sine: squared up. An odd polynomial of the sine adds a square's odd
//    harmonics (3rd, 5th ... 15th), more of them and stronger as Macro
//    rises. A polynomial of a pure sine makes exactly those harmonics and
//    no others, so it cannot fold back, and harmonics near half the sample
//    rate are faded out before they get there.
//
// Each is held to the loudness of its Macro-zero wave as Macro turns.
// Filters are linear, so neither of them can add aliasing either.
#pragma once

#include "fw_config.h"

namespace fw {

enum ShapeKind : uint8_t { SHAPE_NONE, SHAPE_SOFT, SHAPE_FIN };

// How far above the note the filter sits, as a multiple of its frequency.
float softCutoff(float macro);
float finCutoff(float macro);
// What a reading is multiplied by, so the shaped wave is as loud as the plain one.
float softGain(float macro);  // for the saw
float finGain(float macro);   // for the triangle
// The softening filter's gain for any set of harmonics, at `macro`:
// `power[h - 1]` is the strength (squared) of harmonic h, for h = 1..count.
float softGainFor(const float* power, int count, float macro);

// One-pole filter coefficient for a cutoff `mult` times the frequency of a
// note advancing `inc` cycles per sample. 1 means no filtering at all.
float onePole(float inc, float mult);

// The squared-up sine: the polynomial's coefficients, for a note advancing
// `inc` cycles per sample. At Macro zero it is the sine itself.
constexpr int kSquarePoly = 8;  // harmonics 1, 3, ... 15
void squareUp(float macro, float inc, float* coef);
inline float squareUpAt(const float* c, float s) {
  float s2 = s * s;
  float y = c[kSquarePoly - 1];
  for (int k = kSquarePoly - 2; k >= 0; --k) y = y * s2 + c[k];
  return y * s;
}

// Builds the tables the above read. Call once at start-up (tablesInit does).
void shapesInit();

}  // namespace fw
