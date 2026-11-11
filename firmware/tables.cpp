#include "tables.h"

#include "fw_math.h"

namespace fw {

const int16_t kSawHarmonics[kSawLevels] = {512, 362, 256, 181, 128, 90, 64, 45, 32, 22, 16, 11, 8, 5, 4, 2, 1};

// The triangle's harmonics fall away so fast that 64 is plenty: its levels
// are the saw levels from 64 harmonics down.
constexpr int kTriFirstLevel = kSawLevels - kTriLevels;

float gSine[kTabLen + 1];
FW_BIGMEM int16_t gSaw[kSawLevels][kTabLen + 1];
FW_BIGMEM int16_t gTri[kTriLevels][kTabLen + 1];
FW_BIGMEM int16_t gWt[kWtTables][kWtFrames][kWtLevels][kWtLen + 1];

namespace {
FW_BIGMEM float gAcc[kTabLen];

void addHarmonic(int h, float amp) {
  int idx = 0;
  for (int i = 0; i < kTabLen; ++i) {
    gAcc[i] += amp * gSine[idx];
    idx = (idx + h) & (kTabLen - 1);
  }
}

void store(int16_t* dst) {
  for (int i = 0; i < kTabLen; ++i) {
    float v = gAcc[i] * kTabScale;
    dst[i] = (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
  }
  dst[kTabLen] = dst[0];  // guard sample for interpolation
}
}  // namespace

namespace {
void buildWavetables();
}

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

  buildWavetables();
}

// ---- wavetable engine: every frame is a recipe of harmonic strengths ----
namespace {

constexpr int kWtMaxHarm = 32;

// Strength of harmonic h (1..32) in a given frame of a given table.
float wtHarmonic(int table, int frame, int h) {
  switch (table) {
    case 0: {  // sine opening up to a saw
      static const int kTop[kWtFrames] = {1, 3, 8, 32};
      return h <= kTop[frame] ? 1.0f / (float)h : 0.0f;
    }
    case 1: {  // sine opening up to a square (odd harmonics only)
      static const int kTop[kWtFrames] = {1, 3, 9, 31};
      return (h & 1) && h <= kTop[frame] ? 1.0f / (float)h : 0.0f;
    }
    case 2: {  // a resonant peak sweeping upward, vowel-like
      static const float kPeak[kWtFrames] = {2.0f, 4.0f, 8.0f, 16.0f};
      float d = ((float)h - kPeak[frame]) / (0.35f * kPeak[frame] + 0.6f);
      return (1.0f + 5.0f * exp2f_(-1.4427f * d * d)) / (float)h;
    }
    default: {  // organ stops: a few widely spaced harmonics, more with each frame
      static const uint8_t kStops[kWtFrames][6] = {{1, 0, 0, 0, 0, 0}, {1, 2, 4, 0, 0, 0}, {1, 2, 4, 8, 16, 0}, {1, 3, 5, 9, 17, 31}};
      for (int i = 0; i < 6; ++i)
        if (kStops[frame][i] == h) return 1.0f / sqrtf_((float)(i + 1));
      return 0.0f;
    }
  }
}

// Sums one frame at one level of detail into gAcc and returns its peak.
float sumFrame(int table, int frame, int level) {
  const int stride = kTabLen / kWtLen;
  int top = kWtMaxHarm >> level;
  for (int i = 0; i < kWtLen; ++i) gAcc[i] = 0.0f;
  for (int h = 1; h <= top; ++h) {
    float a = wtHarmonic(table, frame, h);
    if (a == 0.0f) continue;
    int idx = 0;
    for (int i = 0; i < kWtLen; ++i) {
      gAcc[i] += a * gSine[idx];
      idx = (idx + h * stride) & (kTabLen - 1);
    }
  }
  float peak = 0.0f;
  for (int i = 0; i < kWtLen; ++i) peak = maxf(peak, absf(gAcc[i]));
  return peak;
}

void buildWavetables() {
  for (int t = 0; t < kWtTables; ++t)
    for (int f = 0; f < kWtFrames; ++f) {
      // One scale for all six levels, so they are equally loud, set by the
      // tallest of them: taking harmonics away can make a wave's peaks
      // higher, and a level that overflowed would be clipped.
      float peak = 0.0f;
      for (int lvl = 0; lvl < kWtLevels; ++lvl) peak = maxf(peak, sumFrame(t, f, lvl));
      float scale = 32000.0f / maxf(peak, 1e-6f);
      for (int lvl = 0; lvl < kWtLevels; ++lvl) {
        sumFrame(t, f, lvl);
        int16_t* dst = gWt[t][f][lvl];
        for (int i = 0; i < kWtLen; ++i) dst[i] = (int16_t)(gAcc[i] * scale);
        dst[kWtLen] = dst[0];
      }
    }
}

}  // namespace

int wtLevelFor(float inc) {
  float maxHarm = 0.5f / maxf(inc, 1e-6f);
  for (int l = 0; l < kWtLevels; ++l)
    if ((float)(kWtMaxHarm >> l) <= maxHarm) return l;
  return kWtLevels - 1;
}

static int levelFor(float inc) {
  float maxHarm = 0.5f / maxf(inc, 1e-6f);
  for (int l = 0; l < kSawLevels; ++l)
    if ((float)kSawHarmonics[l] <= maxHarm) return l;
  return kSawLevels - 1;
}

const int16_t* sawTableFor(float inc) { return gSaw[levelFor(inc)]; }

const int16_t* triTableFor(float inc) {
  int l = levelFor(inc) - kTriFirstLevel;
  return gTri[l < 0 ? 0 : l];
}

}  // namespace fw
