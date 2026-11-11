#include "tables.h"

#include "fw_math.h"

namespace fw {

const int16_t kSawHarmonics[kSawLevels] = {512, 362, 256, 181, 128, 90, 64, 45, 32, 22, 16, 11, 8, 5, 4, 2, 1};

// The triangle's harmonics fall away so fast that 64 is plenty: its levels
// are the saw levels from 64 harmonics down.
constexpr int kTriFirstLevel = kSawLevels - kTriLevels;

float gSine[kTabLen + 1];
FW_BIGMEM float gSaw[kSawLevels][kTabLen + 1];
FW_BIGMEM float gTri[kTriLevels][kTabLen + 1];

namespace {
FW_BIGMEM float gAcc[kTabLen];

void addHarmonic(int h, float amp) {
  int idx = 0;
  for (int i = 0; i < kTabLen; ++i) {
    gAcc[i] += amp * gSine[idx];
    idx = (idx + h) & (kTabLen - 1);
  }
}

void store(float* dst) {
  for (int i = 0; i < kTabLen; ++i) dst[i] = gAcc[i];
  dst[kTabLen] = gAcc[0];  // guard sample for interpolation
}
}  // namespace

void tablesInit() {
  for (int i = 0; i <= kTabLen; ++i) gSine[i] = sinf_(kTwoPi * (float)i / (float)kTabLen);
  gSine[kTabLen] = gSine[0];

  // Rising saw, -1..+1:  -(2/pi) * sum sin(2 pi h x) / h
  for (int i = 0; i < kTabLen; ++i) gAcc[i] = 0.0f;
  int h = 1;
  for (int lvl = kSawLevels - 1; lvl >= 0; --lvl) {
    for (; h <= kSawHarmonics[lvl]; ++h) addHarmonic(h, -(2.0f / kPi) / (float)h);
    store(gSaw[lvl]);
  }

  // Triangle, -1..+1:  (8/pi^2) * sum over odd h of (-1)^((h-1)/2) sin(2 pi h x) / h^2
  for (int i = 0; i < kTabLen; ++i) gAcc[i] = 0.0f;
  h = 1;
  for (int lvl = kSawLevels - 1; lvl >= kTriFirstLevel; --lvl) {
    for (; h <= kSawHarmonics[lvl]; ++h) {
      if ((h & 1) == 0) continue;
      float sign = ((h - 1) / 2) & 1 ? -1.0f : 1.0f;
      addHarmonic(h, sign * (8.0f / (kPi * kPi)) / (float)(h * h));
    }
    store(gTri[lvl - kTriFirstLevel]);
  }
}

static int levelFor(float inc) {
  float maxHarm = 0.5f / maxf(inc, 1e-6f);
  for (int l = 0; l < kSawLevels; ++l)
    if ((float)kSawHarmonics[l] <= maxHarm) return l;
  return kSawLevels - 1;
}

const float* sawTableFor(float inc) { return gSaw[levelFor(inc)]; }

const float* triTableFor(float inc) {
  int l = levelFor(inc) - kTriFirstLevel;
  return gTri[l < 0 ? 0 : l];
}

}  // namespace fw
