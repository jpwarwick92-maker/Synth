#include "fx.h"

#include "fw_math.h"
#include "tables.h"

namespace fw {

Effects gFx;

namespace {

// The lines are sized for 48 kHz, the rate the page runs at; the Teensy
// runs at 44.1 kHz. At a higher rate everything still works and stays in
// bounds, but there is less time in each line: the longest delay is shorter
// than its setting says and the reverb's lines are scaled down together.
constexpr int kChorusLen = 2048;    // 42 ms at 48 kHz
constexpr int kDelayLen = 36000;    // 0.75 s at 48 kHz
constexpr int kReverbLen = 2304;    // 48 ms at 48 kHz
constexpr int kAllpassLen = 512;

// Feedback loops left to die away end up holding numbers too small to
// matter but slow to compute with on some processors. Anything this small
// is called zero.
inline float tiny0(float x) { return absf(x) < 1e-15f ? 0.0f : x; }

float gChorus[kChorusLen];
FW_BIGMEM int16_t gDelay[kDelayLen];  // 16-bit: half the memory, noise far below the signal
float gReverb[4][kReverbLen];
float gAllpass[2][kAllpassLen];

// reverb line lengths in seconds: mutually prime at any common sample rate
const float kReverbSec[4] = {0.02971f, 0.03711f, 0.04113f, 0.04377f};
const float kAllpassSec[2] = {0.00531f, 0.00189f};

// Reads a delay line at a fractional position, 0 <= pos <= len. The top end
// is included on purpose: a position a hair below zero, wrapped by adding
// len, rounds up to exactly len in a float, and must read slot 0, not the
// memory after the buffer.
inline float readFrac(const float* buf, int len, float pos) {
  int i0 = (int)pos;
  float f = pos - (float)i0;
  if (i0 >= len) i0 -= len;
  if (i0 < 0) i0 = 0;
  int i1 = i0 + 1;
  if (i1 >= len) i1 -= len;
  return buf[i0] + (buf[i1] - buf[i0]) * f;
}

// Nothing a voice does should reach the effects as a wild value, but the
// reverb and chorus lines are plain floats with long memories: one bad
// sample would ring in them for seconds, or for ever if it is not a number.
// So what goes in is held to a sane range first.
inline float sane(float x) {
  if (!(x == x)) return 0.0f;  // not a number
  return clampf(x, -4.0f, 4.0f);
}

}  // namespace

void Effects::init(float sampleRate) {
  sr_ = sampleRate;
  chorusOn_ = delayOn_ = reverbOn_ = false;
  chPos_ = 0;
  chPhase_ = 0.0f;
  dlPos_ = 0;
  dlLen_ = minf(0.3f * sr_, (float)(kDelayLen - 2));
  dlGlide_ = 1.0f - exp2f_(-1.4427f / (0.06f * sr_));
  dlLp_ = 0.0f;
  rvPos_ = 0;
  // If the longest line does not fit its buffer, all four shrink by the
  // same factor, so they stay different lengths and the reverb keeps its character.
  rvScale_ = minf(1.0f, (float)(kReverbLen - 2) / (kReverbSec[3] * sr_));
  for (int i = 0; i < 4; ++i) {
    rvLen_[i] = clampi((int)(kReverbSec[i] * sr_ * rvScale_), 8, kReverbLen - 1);
    rvLp_[i] = 0.0f;
  }
  for (int i = 0; i < 2; ++i) {
    apLen_[i] = clampi((int)(kAllpassSec[i] * sr_), 4, kAllpassLen - 1);
    apPos_[i] = 0;
  }
}

void Effects::process(const float* inL, const float* inR, float* outL, float* outR, int n, const FxSettings& s) {
  for (int i = 0; i < n; ++i) {
    outL[i] = sane(inL[i]);
    outR[i] = sane(inR[i]);
  }
  inL = outL;  // from here on, only the checked copy is read
  inR = outR;

  // ---- chorus: two slowly swept taps, one per side ----
  bool chorus = s.chorus > 0.001f;
  if (chorus && !chorusOn_)
    for (int i = 0; i < kChorusLen; ++i) gChorus[i] = 0.0f;
  chorusOn_ = chorus;
  if (chorus) {
    float room = minf(1.0f, (float)(kChorusLen - 4) / (0.016f * sr_));  // keeps the taps inside the line at any rate
    float base = 0.012f * sr_ * room, depth = 0.004f * sr_ * s.chorus * room;
    float wet = 0.5f * s.chorus;
    float inc = 0.6f / sr_;
    for (int i = 0; i < n; ++i) {
      float mono = 0.5f * (inL[i] + inR[i]);
      gChorus[chPos_] = mono;
      float ph2 = chPhase_ + 0.25f;
      if (ph2 >= 1.0f) ph2 -= 1.0f;
      float dL = base + depth * tabRead(gSine, chPhase_);
      float dR = base + depth * tabRead(gSine, ph2);
      float pL = (float)chPos_ - dL, pR = (float)chPos_ - dR;
      if (pL < 0.0f) pL += (float)kChorusLen;
      if (pR < 0.0f) pR += (float)kChorusLen;
      float dryL = inL[i], dryR = inR[i];
      outL[i] = dryL + wet * (readFrac(gChorus, kChorusLen, pL) - mono);
      outR[i] = dryR + wet * (readFrac(gChorus, kChorusLen, pR) - mono);
      if (++chPos_ >= kChorusLen) chPos_ = 0;
      chPhase_ += inc;
      if (chPhase_ >= 1.0f) chPhase_ -= 1.0f;
    }
  }

  // ---- delay: one line, repeats in the centre ----
  bool delay = s.dlyMix > 0.001f;
  if (delay && !delayOn_) {
    for (int i = 0; i < kDelayLen; ++i) gDelay[i] = 0;
    dlLp_ = 0.0f;
  }
  delayOn_ = delay;
  if (delay) {
    float target = clampf(s.dlyTime * sr_, 16.0f, (float)(kDelayLen - 2));
    float tone = 1.0f - exp2f_(-1.4427f * kTwoPi * (500.0f * exp2f_(s.dlyTone * 4.585f)) / sr_);
    if (tone > 1.0f) tone = 1.0f;
    float fdbk = 0.9f * s.dlyFdbk, wet = 0.8f * s.dlyMix;
    for (int i = 0; i < n; ++i) {
      dlLen_ += (target - dlLen_) * dlGlide_;   // changing the time bends the pitch, like tape
      float pos = (float)dlPos_ - dlLen_;
      if (pos < 0.0f) pos += (float)kDelayLen;
      int i0 = (int)pos;
      float f = pos - (float)i0;
      if (i0 >= kDelayLen) i0 -= kDelayLen;  // pos can round up to exactly the length (see readFrac)
      int i1 = i0 + 1 < kDelayLen ? i0 + 1 : 0;
      float echo = ((float)gDelay[i0] + (float)(gDelay[i1] - gDelay[i0]) * f) * (1.0f / 32767.0f);
      dlLp_ = tiny0(dlLp_ + tone * (echo - dlLp_));
      float write = tanhf_(0.5f * (outL[i] + outR[i]) + dlLp_ * fdbk);
      gDelay[dlPos_] = (int16_t)(write * 32767.0f);
      if (++dlPos_ >= kDelayLen) dlPos_ = 0;
      outL[i] += echo * wet;
      outR[i] += echo * wet;
    }
  }

  // ---- reverb: two diffusers into four cross-fed delay lines ----
  bool reverb = s.rvbMix > 0.001f;
  if (reverb && !reverbOn_) {
    for (int l = 0; l < 4; ++l) {
      for (int i = 0; i < kReverbLen; ++i) gReverb[l][i] = 0.0f;
      rvLp_[l] = 0.0f;
    }
    for (int a = 0; a < 2; ++a)
      for (int i = 0; i < kAllpassLen; ++i) gAllpass[a][i] = 0.0f;
  }
  reverbOn_ = reverb;
  if (reverb) {
    // gain per trip round the loop for the level to fall 60 dB in rvbTime
    float g = exp2f_(-9.966f * 0.038f * rvScale_ / maxf(s.rvbTime, 0.05f));
    float damp = 0.15f + 0.85f * s.rvbTone;
    // a long tail piles up on itself; trim it so a bigger room is not simply louder
    float wet = 0.95f * s.rvbMix * sqrtf_(1.0f - g * g);
    for (int i = 0; i < n; ++i) {
      float x = 0.5f * (outL[i] + outR[i]);
      for (int a = 0; a < 2; ++a) {  // diffusion: smears each note into a wash
        float d = gAllpass[a][apPos_[a]];
        float v = tiny0(x + 0.6f * d);
        gAllpass[a][apPos_[a]] = v;
        if (++apPos_[a] >= apLen_[a]) apPos_[a] = 0;
        x = d - 0.6f * v;
      }
      float d[4];
      for (int l = 0; l < 4; ++l) {
        int rp = rvPos_ - rvLen_[l];
        if (rp < 0) rp += kReverbLen;
        rvLp_[l] = tiny0(rvLp_[l] + damp * (gReverb[l][rp] - rvLp_[l]));
        d[l] = rvLp_[l];
      }
      // every line feeds every other, in a pattern that neither gains nor loses energy
      float a = d[0] + d[1], b = d[0] - d[1], c = d[2] + d[3], e = d[2] - d[3];
      gReverb[0][rvPos_] = x + g * 0.5f * (a + c);
      gReverb[1][rvPos_] = x + g * 0.5f * (b + e);
      gReverb[2][rvPos_] = x + g * 0.5f * (a - c);
      gReverb[3][rvPos_] = x + g * 0.5f * (b - e);
      if (++rvPos_ >= kReverbLen) rvPos_ = 0;
      outL[i] += wet * (d[0] + 0.5f * d[2]);
      outR[i] += wet * (d[1] + 0.5f * d[3]);
    }
  }
}

}  // namespace fw
