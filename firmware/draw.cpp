#include "draw.h"

#include "fw_math.h"
#include "shapes.h"

namespace fw {

Drawing gDraw;

const char* const kDrawShapeNames[DRAWSHAPE_COUNT] = {"SAW", "SQUARE", "TRIANGLE", "SINE", "FLAT"};

namespace {

// Two prepared copies, about 57 KB each: one played, one being built.
FW_BIGMEM DrawCopy gCopies[2];
// Build space: the octave tables are summed here a harmonic at a time, and
// a wavetable frame's six levels here before they are scaled and stored.
FW_BIGMEM float gSum[kTabLen];
FW_BIGMEM float gFrameSum[6][512];

// Tables are stored with 1.0 at this value, which leaves room for a
// drawing whose points sit far from its mean (up to 2, once the mean is taken off).
constexpr float kDrawScale = 15000.0f;
constexpr float kSawRms = 0.57735f;   // a saw's loudness: what a drawing is brought to
constexpr float kMaxGain = 8.0f;      // a nearly flat drawing is not blown up into noise
constexpr float kMaxPeak = 1.18f;     // no louder at its peak than a band-limited saw
constexpr int kHarmonicsPerSlice = 4;
constexpr int kWtFrameTop[4] = {32, 8, 3, 1};  // table 6's frames: the drawing, softer, softer still, a sine

enum Stage : uint8_t { STAGE_SPECTRUM, STAGE_HARMONICS, STAGE_FRAMES, STAGE_PUBLISH };

inline float sineAt(int idx) { return gSine[idx & (kTabLen - 1)]; }
inline float cosineAt(int idx) { return gSine[(idx + kTabLen / 4) & (kTabLen - 1)]; }

inline int16_t toSample(float v, float scale) {
  float s = v * scale;
  s = s > 32767.0f ? 32767.0f : s < -32767.0f ? -32767.0f : s;
  return (int16_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
}

// The 16 points as -1..1.
inline float unit(int8_t p) { return (float)p * (1.0f / 127.0f); }

}  // namespace

int drawLevelFor(float inc) {
  float room = 0.5f / maxf(inc, 1e-6f);  // the harmonics that fit below half the sample rate
  for (int l = 0; l < kDrawLevels; ++l)
    if ((float)(kDrawHarmonics >> l) < room) return l;
  return kDrawLevels - 1;
}

void Drawing::reset() {
  for (int i = 0; i < kDrawPoints; ++i) pts_[i] = 0;
  startShape(DRAWSHAPE_SAW);
  edits_ = 1;
  built_ = 0;
  building_ = false;
  target_ = 0;
  stage_ = item_ = 0;
  peak_ = 0.0f;
  live_ = 0;
  oldCopy_ = 0;
  ready_ = false;
  readyCopy_ = 0;
  fade_ = 0.0f;
  fadeNow_ = fadeStep_ = 0.0f;
}

void Drawing::buildNow() {
  // No audio yet: build straight into the live copy's place, then take it.
  ready_ = false;
  fade_ = 0.0f;
  building_ = false;
  beginBuild();
  while (building_) step();
  live_ = readyCopy_;
  oldCopy_ = readyCopy_;
  ready_ = false;
}

bool Drawing::setPoint(int i, int v) {
  int8_t p = (int8_t)clampi(v, -127, 127);
  i &= kDrawPoints - 1;
  if (pts_[i] == p) return false;
  pts_[i] = p;
  ++edits_;
  return true;
}

bool Drawing::setPoints(const int8_t* p) {
  bool changed = false;
  for (int i = 0; i < kDrawPoints; ++i) {
    int8_t v = (int8_t)clampi(p[i], -127, 127);
    if (pts_[i] != v) changed = true;
    pts_[i] = v;
  }
  if (changed) ++edits_;
  return changed;
}

bool Drawing::startShape(int shape) {
  int8_t p[kDrawPoints];
  for (int i = 0; i < kDrawPoints; ++i) {
    float v;
    switch (shape) {
      case DRAWSHAPE_SAW: v = -1.0f + 2.0f * (float)i / (float)(kDrawPoints - 1); break;
      case DRAWSHAPE_SQUARE: v = i < kDrawPoints / 2 ? 0.8f : -0.8f; break;
      case DRAWSHAPE_TRI: v = 1.0f - 2.0f * absf((float)i - (float)(kDrawPoints / 2)) / (float)(kDrawPoints / 2); break;
      case DRAWSHAPE_SINE: v = sinf_(kTwoPi * (float)i / (float)kDrawPoints); break;
      default: v = 0.0f; break;
    }
    float s = v * 127.0f;
    p[i] = (int8_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
  }
  return setPoints(p);
}

bool Drawing::smooth() {
  int8_t p[kDrawPoints];
  for (int i = 0; i < kDrawPoints; ++i) {
    int sum = pts_[(i + kDrawPoints - 1) % kDrawPoints] + 2 * pts_[i] + pts_[(i + 1) % kDrawPoints];
    p[i] = (int8_t)(sum >= 0 ? (sum + 2) / 4 : -((-sum + 2) / 4));
  }
  return setPoints(p);
}

bool Drawing::settled() const { return !building_ && !ready_ && fade_ <= 0.0f && edits_ == built_; }

bool Drawing::silent() const {
  float lo = 1.0f, hi = -1.0f;
  for (int i = 0; i < kDrawPoints; ++i) {
    lo = minf(lo, unit(pts_[i]));
    hi = maxf(hi, unit(pts_[i]));
  }
  return hi - lo < 1e-3f;
}

bool Drawing::work() {
  if (!building_) {
    if (edits_ == built_) return false;
    if (ready_ || fade_ > 0.0f) return true;  // the audio has not finished taking the last copy yet
    beginBuild();
    return true;
  }
  step();
  return building_ || edits_ != built_;
}

void Drawing::beginBuild() {
  target_ = live_ == 0 ? 1 : 0;
  DrawCopy& c = gCopies[target_];
  for (int i = 0; i < kDrawPoints; ++i) c.pts[i] = pts_[i];
  built_ = edits_;  // a later edit means building again, once this one is done
  stage_ = STAGE_SPECTRUM;
  item_ = 0;
  building_ = true;
}

// One slice of the build: no more than a few harmonics' worth of work.
void Drawing::step() {
  DrawCopy& c = gCopies[target_];
  switch (stage_) {
    case STAGE_SPECTRUM: {
      // A polyline's slope changes only at its points, so its spectrum is
      // that of the second differences, divided by (2 pi h)^2. Those 16
      // numbers give every harmonic: the pattern repeats every 16.
      float d2[kDrawPoints];
      for (int n = 0; n < kDrawPoints; ++n)
        d2[n] = unit(c.pts[(n + 1) % kDrawPoints]) - 2.0f * unit(c.pts[n]) + unit(c.pts[(n + kDrawPoints - 1) % kDrawPoints]);
      for (int k = 0; k < kDrawPoints; ++k) {
        float re = 0.0f, im = 0.0f;
        for (int n = 0; n < kDrawPoints; ++n) {
          int idx = (k * n * (kTabLen / kDrawPoints)) & (kTabLen - 1);
          re += d2[n] * cosineAt(idx);
          im -= d2[n] * sineAt(idx);
        }
        re_[k] = re;
        im_[k] = im;
      }
      // Loudness, from the harmonics: each contributes 2 |c_h|^2.
      static float power[kDrawHarmonics];
      float ms = 0.0f;
      for (int h = 1; h <= kDrawHarmonics; ++h) {
        int k = h % kDrawPoints;
        float scale = (float)kDrawPoints / (4.0f * kPi * kPi * (float)(h * h));
        float p = 2.0f * (re_[k] * re_[k] + im_[k] * im_[k]) * scale * scale;
        power[h - 1] = p;
        ms += p;
      }
      float rms = sqrtf_(ms);
      c.silent = rms < 1e-4f;
      c.gain = c.silent ? 0.0f : minf(kSawRms / rms, kMaxGain);
      for (int i = 0; i < kDrawGainPoints; ++i)
        c.softGain[i] = c.silent ? 1.0f : softGainFor(power, kDrawHarmonics, (float)i / (float)(kDrawGainPoints - 1));
      // The exciter: the points straight, less their mean (a constant offset
      // would ring in the string for seconds), at the other exciters' loudness.
      float mean = 0.0f, sq = 0.0f, dev = 0.0f;
      for (int n = 0; n < kDrawPoints; ++n) mean += unit(c.pts[n]);
      mean *= 1.0f / (float)kDrawPoints;
      for (int n = 0; n < kDrawPoints; ++n) {
        float a = unit(c.pts[n]) - mean, b = unit(c.pts[(n + 1) % kDrawPoints]) - mean;
        sq += (a * a + a * b + b * b) * (1.0f / 3.0f);  // the mean square along a straight segment
        dev = maxf(dev, absf(a));
      }
      float var = sq * (1.0f / (float)kDrawPoints);
      c.pluckMean = mean;
      c.pluckGain = var > 1e-6f ? minf(minf(kSawRms / sqrtf_(var), kMaxGain), 1.0f / dev) : 0.0f;
      for (int i = 0; i < kTabLen; ++i) gSum[i] = 0.0f;
      peak_ = 0.0f;
      stage_ = STAGE_HARMONICS;
      item_ = 1;
      break;
    }
    case STAGE_HARMONICS: {
      for (int n = 0; n < kHarmonicsPerSlice && item_ <= kDrawHarmonics; ++n, ++item_) {
        int h = item_, k = h % kDrawPoints;
        float scale = -(float)kDrawPoints / (4.0f * kPi * kPi * (float)(h * h));
        float a = 2.0f * re_[k] * scale, b = -2.0f * im_[k] * scale;  // cosine and sine parts
        if (a != 0.0f || b != 0.0f) {
          int idx = 0;
          for (int i = 0; i < kTabLen; ++i) {
            gSum[i] += a * cosineAt(idx) + b * sineAt(idx);
            idx = (idx + h) & (kTabLen - 1);
          }
        }
        if ((h & (h - 1)) == 0) {  // 1, 2, 4 ... 128 harmonics: one octave's table is complete
          int lvl = 0;
          while ((kDrawHarmonics >> lvl) > h) ++lvl;
          int16_t* dst = c.tab[lvl];
          for (int i = 0; i < kTabLen; ++i) {
            peak_ = maxf(peak_, absf(gSum[i]));
            dst[i] = toSample(gSum[i], kDrawScale);
          }
          dst[kTabLen] = dst[0];
        }
      }
      if (item_ > kDrawHarmonics) {
        stage_ = STAGE_FRAMES;
        item_ = 0;
      }
      break;
    }
    case STAGE_FRAMES: {
      // Table 6 for the wavetable engine: four frames of six levels each.
      // Item 7f + l sums level l of frame f; item 7f + 6 scales and stores the frame.
      int frame = item_ / 7, sub = item_ % 7;
      const int stride = kTabLen / 512;
      if (sub < 6) {
        int top = kWtFrameTop[frame] < (32 >> sub) ? kWtFrameTop[frame] : (32 >> sub);
        float* acc = gFrameSum[sub];
        for (int i = 0; i < 512; ++i) acc[i] = 0.0f;
        for (int h = 1; h <= top; ++h) {
          int k = h % kDrawPoints;
          float scale = -(float)kDrawPoints / (4.0f * kPi * kPi * (float)(h * h));
          float a = 2.0f * re_[k] * scale, b = -2.0f * im_[k] * scale;
          if (frame == 3) {  // a pure sine, at the drawing's own phase (or plain, if it has no fundamental)
            float m = sqrtf_(a * a + b * b);
            if (m < 1e-6f) {
              a = 0.0f;
              b = 1.0f;
            } else {
              a /= m;
              b /= m;
            }
          } else if (frame > 0) {  // fewer harmonics, the top ones tapered off so they do not ring
            float x = kPi * (float)h / (float)(kWtFrameTop[frame] + 1);
            float taper = sinf_(x) / x;
            a *= taper;
            b *= taper;
          }
          int idx = 0;
          for (int i = 0; i < 512; ++i) {
            acc[i] += a * cosineAt(idx) + b * sineAt(idx);
            idx = (idx + h * stride) & (kTabLen - 1);
          }
        }
      } else {
        // One scale for all six levels (as gWt has), set by the tallest of them.
        float peak = 0.0f;
        for (int l = 0; l < 6; ++l)
          for (int i = 0; i < 512; ++i) peak = maxf(peak, absf(gFrameSum[l][i]));
        float scale = peak > 1e-6f ? 32000.0f / peak : 0.0f;
        for (int l = 0; l < 6; ++l) {
          int16_t* dst = c.wt[frame][l];
          for (int i = 0; i < 512; ++i) dst[i] = toSample(gFrameSum[l][i], scale);
          dst[512] = dst[0];
        }
      }
      if (++item_ >= 4 * 7) stage_ = STAGE_PUBLISH;
      break;
    }
    default: {
      // A reading (in kTabScale units) times this is the drawing at a saw's loudness, its peak capped.
      float g = c.gain;
      if (g * peak_ > kMaxPeak) g = kMaxPeak / peak_;
      c.gain = g * (kTabScale / kDrawScale);
      readyCopy_ = (int8_t)target_;
      building_ = false;
      ready_ = true;
      break;
    }
  }
}

void Drawing::audioBlock(int len, float sampleRate) {
  if (ready_ && fade_ <= 0.0f) {
    oldCopy_ = live_;
    live_ = readyCopy_;
    ready_ = false;
    fade_ = 1.0f;
  }
  fadeNow_ = fade_;
  float step = 1.0f / (0.02f * sampleRate);  // 20 ms
  fadeStep_ = fade_ > 0.0f ? step : 0.0f;
  if (fade_ > 0.0f) {
    float f = fade_ - step * (float)len;
    fade_ = f > 0.0f ? f : 0.0f;
  }
}

const DrawCopy& Drawing::live() const { return gCopies[live_]; }
const DrawCopy& Drawing::old() const { return gCopies[oldCopy_]; }

}  // namespace fw
