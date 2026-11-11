// Small maths library used everywhere in the core.
//
// The core does not link libm: these give identical results in the browser,
// on the Teensy and in the native tests, and they are cheap enough to call
// at control rate on a Cortex-M7.
#pragma once

#include "fw_config.h"

namespace fw {

constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717959f;
constexpr float kHalfPi = 1.57079632679490f;

inline float absf(float x) { return __builtin_fabsf(x); }
inline float sqrtf_(float x) { return __builtin_sqrtf(x); }
inline float floorf_(float x) { return __builtin_floorf(x); }

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline int clampi(int x, int lo, int hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float minf(float a, float b) { return a < b ? a : b; }
inline float maxf(float a, float b) { return a > b ? a : b; }

inline uint32_t floatBits(float f) {
  uint32_t u;
  __builtin_memcpy(&u, &f, sizeof u);
  return u;
}
inline float bitsFloat(uint32_t u) {
  float f;
  __builtin_memcpy(&f, &u, sizeof f);
  return f;
}

// 2^x. Relative error below 1e-7 (far under a hundredth of a cent).
inline float exp2f_(float x) {
  x = clampf(x, -100.0f, 100.0f);
  float fl = floorf_(x);
  int i = (int)fl;
  float y = (x - fl - 0.5f) * 0.69314718056f;  // ln2 * (frac - 0.5), |y| <= 0.347
  float p = 1.0f + y * (1.0f + y * (0.5f + y * (0.166666667f + y * (0.0416666667f +
            y * (0.00833333333f + y * 0.00138888889f)))));
  return 1.41421356237f * p * bitsFloat((uint32_t)(i + 127) << 23);
}

// log2(x) for x > 0. Absolute error around 1e-6.
inline float log2f_(float x) {
  if (x <= 0.0f) return -100.0f;
  uint32_t b = floatBits(x);
  int e = (int)((b >> 23) & 255u) - 127;
  float m = bitsFloat((b & 0x007FFFFFu) | 0x3F800000u);  // 1 <= m < 2
  float t = (m - 1.0f) / (m + 1.0f);
  float t2 = t * t;
  float ln = 2.0f * t * (1.0f + t2 * (0.333333333f + t2 * (0.2f + t2 * (0.142857143f +
             t2 * 0.111111111f))));
  return (float)e + ln * 1.44269504089f;
}

// sin(x), x in radians, any range. Error below 1e-6.
inline float sinf_(float x) {
  float k = floorf_(x * (1.0f / kTwoPi) + 0.5f);
  x -= k * kTwoPi;  // now in [-pi, pi]
  if (x > kHalfPi) x = kPi - x;
  else if (x < -kHalfPi) x = -kPi - x;
  float x2 = x * x;
  return x * (1.0f + x2 * (-0.166666667f + x2 * (0.00833333333f + x2 * (-0.000198412698f +
         x2 * (2.75573192e-6f + x2 * -2.50521084e-8f)))));
}
inline float cosf_(float x) { return sinf_(x + kHalfPi); }
inline float tanf_(float x) { return sinf_(x) / cosf_(x); }

// Smooth saturator, close to tanh. Odd, monotonic, limits at +/-1.
inline float tanhf_(float x) {
  x = clampf(x, -3.0f, 3.0f);
  float x2 = x * x;
  return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

}  // namespace fw
