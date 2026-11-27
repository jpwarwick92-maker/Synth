#include "shapes.h"

#include "fw_math.h"

namespace fw {

namespace {

constexpr int kGainPoints = 65;  // Macro 0, 1/64, ... 1
float gSoftGain[kGainPoints];
float gFinGain[kGainPoints];
// sin((2j + 1) x) written as a polynomial in s = sin x: the coefficient of s^(2k + 1).
double gOddSine[kSquarePoly][kSquarePoly];
float gTaper[kSquarePoly];  // softens the top harmonics so the squared-up sine does not ring

float lookup(const float* table, float macro) {
  float p = clampf(macro, 0.0f, 1.0f) * (float)(kGainPoints - 1);
  int i = (int)p;
  if (i >= kGainPoints - 1) return table[kGainPoints - 1];
  return table[i] + (table[i + 1] - table[i]) * (p - (float)i);
}

}  // namespace

float softCutoff(float macro) { return 1.5f * exp2f_(log2f_(256.0f / 1.5f) * (1.0f - clampf(macro, 0.0f, 1.0f))); }

float finCutoff(float macro) { return clampf(macro, 0.0f, 1.0f); }

float softGain(float macro) { return lookup(gSoftGain, macro); }

float finGain(float macro) { return lookup(gFinGain, macro); }

float softGainFor(const float* power, int count, float macro) {
  float m = softCutoff(macro), inv = 1.0f / (m * m);
  float all = 0.0f, kept = 0.0f;
  for (int h = 1; h <= count; ++h) {
    float p = power[h - 1];
    all += p;
    kept += p / (1.0f + (float)(h * h) * inv);  // a one-pole low-pass's power at harmonic h
  }
  return kept > 1e-12f ? sqrtf_(all / kept) : 1.0f;
}

float onePole(float inc, float mult) {
  float x = inc * mult;
  if (x >= 0.45f) return 1.0f;
  return 1.0f - exp2f_(-1.44269504f * kTwoPi * x);
}

void squareUp(float macro, float inc, float* coef) {
  // Harmonic h (odd) at strength (1/h) r^((h - 1)/2): a sine at r = 0, nearly a
  // square at full. Each harmonic fades out as it nears half the sample rate.
  float r = 0.9f * clampf(macro, 0.0f, 1.0f);
  double a[kSquarePoly];
  double sum = 0.0, rp = 1.0;
  for (int j = 0; j < kSquarePoly; ++j) {
    int h = 2 * j + 1;
    float fade = j == 0 ? 1.0f : clampf((0.46f - (float)h * inc) * (1.0f / 0.06f), 0.0f, 1.0f);
    a[j] = j == 0 ? 1.0 : rp / (double)h * (double)gTaper[j] * (double)fade;
    rp *= (double)r;
    sum += a[j] * a[j];
  }
  // As loud as the sine: the same power, however many harmonics there are.
  double g = 1.0 / (double)sqrtf_((float)sum);
  for (int k = 0; k < kSquarePoly; ++k) {
    double c = 0.0;
    for (int j = k; j < kSquarePoly; ++j) c += a[j] * gOddSine[j][k];
    coef[k] = (float)(c * g);
  }
}

void shapesInit() {
  // Chebyshev polynomials T_n(s); for odd n, sin(n x) = (-1)^((n - 1)/2) T_n(sin x).
  double t[2 * kSquarePoly][2 * kSquarePoly];
  for (int n = 0; n < 2 * kSquarePoly; ++n)
    for (int k = 0; k < 2 * kSquarePoly; ++k) t[n][k] = 0.0;
  t[0][0] = 1.0;
  t[1][1] = 1.0;
  for (int n = 2; n < 2 * kSquarePoly; ++n)
    for (int k = 0; k <= n; ++k) t[n][k] = (k > 0 ? 2.0 * t[n - 1][k - 1] : 0.0) - t[n - 2][k];
  for (int j = 0; j < kSquarePoly; ++j)
    for (int k = 0; k < kSquarePoly; ++k) gOddSine[j][k] = ((j & 1) ? -1.0 : 1.0) * t[2 * j + 1][2 * k + 1];
  for (int j = 0; j < kSquarePoly; ++j) {
    float x = kPi * (float)j / (float)kSquarePoly;
    gTaper[j] = j == 0 ? 1.0f : sinf_(x) / x;
  }

  // Loudness tables, from the harmonics of the waves the filters are fed.
  static float sawPower[512];
  for (int h = 1; h <= 512; ++h) sawPower[h - 1] = 1.0f / (float)(h * h);
  for (int i = 0; i < kGainPoints; ++i) {
    float macro = (float)i / (float)(kGainPoints - 1);
    gSoftGain[i] = i == 0 ? 1.0f : softGainFor(sawPower, 512, macro);
    // the triangle: odd harmonics at 1/h^2; a one-pole high-pass keeps h^2 / (h^2 + m^2) of each
    float m = finCutoff(macro), all = 0.0f, kept = 0.0f;
    for (int h = 1; h < 128; h += 2) {
      float p = 1.0f / (float)(h * h * h * h);
      all += p;
      kept += p * (float)(h * h) / ((float)(h * h) + m * m);
    }
    gFinGain[i] = sqrtf_(all / kept);
  }
}

}  // namespace fw
