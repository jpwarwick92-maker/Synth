#include "vocoder.h"

#include "fw_math.h"

namespace fw {

Vocoder gVocoder;

namespace {

constexpr float kLowHz = 140.0f, kHighHz = 7000.0f;
// Each band is two identical band-pass sections. With this sharpness the
// pair is 3 dB down where it meets its neighbour.
constexpr float kSectionQ = 2.46f;
// A voice at a healthy level opens its strongest bands fully.
constexpr float kGain = 12.0f;
constexpr float kMaxBandGain = 3.0f;
constexpr float kHiss = 0.25f;  // the noise's level at full Consonants

inline float tiny0(float x) { return absf(x) < 1e-15f ? 0.0f : x; }

}  // namespace

float vocBandHz(int k) {
  // equal steps on a musical scale: each band is 1.298 times the one below
  return kLowHz * exp2f_(log2f_(kHighHz / kLowHz) * (float)k / (float)(kVocBands - 1));
}

float mapVocRelease(float smooth) { return 0.008f * exp2f_(5.3f * clampf(smooth, 0.0f, 1.0f)); }  // 8 ms .. 315 ms

// Coefficients for every band, with the centres moved by `shiftSemis`.
void Vocoder::setBands(float* b0, float* a1, float* a2, float shiftSemis, float rate) const {
  float shift = exp2f_(shiftSemis * (1.0f / 12.0f));
  for (int k = 0; k < kVocBands; ++k) {
    float hz = minf(vocBandHz(k) * shift, 0.45f * rate);  // never past what the sample rate can carry
    float w = kTwoPi * hz / rate;
    float alpha = sinf_(w) / (2.0f * kSectionQ);
    float norm = 1.0f / (1.0f + alpha);
    b0[k] = alpha * norm;
    a1[k] = -2.0f * cosf_(w) * norm;
    a2[k] = (1.0f - alpha) * norm;
  }
}

void Vocoder::init(float sampleRate) {
  sr_ = sampleRate;
  on_ = false;
  amount_ = 0.0f;
  formant_ = 0.0f;
  smooth_ = -1.0f;
  attack_ = 1.0f - exp2f_(-1.4427f / (0.003f * 0.5f * sr_));  // a band's level rises in about 3 ms
  half_ = false;
  held_ = 0.0f;
  gate_ = 0.0f;
  gateCoef_ = 1.0f - exp2f_(-1.4427f * (float)kCtrlBlock / (0.02f * sr_));
  rng_ = 0x51ED270Bu;
  setBands(vb0_, va1_, va2_, 0.0f, 0.5f * sr_);
  setBands(cb0_, ca1_, ca2_, 0.0f, sr_);
  for (int k = 0; k < kVocBands; ++k) {
    for (int i = 0; i < 4; ++i) vz_[k][i] = cz_[k][i] = 0.0f;
    env_[k] = 0.0f;
    gain_[k] = 0.0f;
    fall_[k] = 0.0f;
    shown_[k] = 0.0f;
    // A voice is weaker the higher it goes; the upper bands are turned up to
    // match, or the result is dull and hard to follow.
    weight_[k] = kGain * clampf(sqrtf_(vocBandHz(k) * (1.0f / 500.0f)), 0.8f, 3.0f);
  }
}

void Vocoder::process(float* busL, float* busR, const float* voice, int n, const VocSettings& s) {
  bool want = s.amount > 0.001f;
  if (!want && amount_ <= 0.001f) {  // off: the synth's sound is left exactly as it is
    if (on_) {
      on_ = false;
      amount_ = 0.0f;
      for (int k = 0; k < kVocBands; ++k) shown_[k] = 0.0f;
    }
    return;
  }
  if (!on_) {  // switched on: start from silence, not from whatever was left last time
    on_ = true;
    for (int k = 0; k < kVocBands; ++k) {
      for (int i = 0; i < 4; ++i) vz_[k][i] = cz_[k][i] = 0.0f;
      env_[k] = gain_[k] = 0.0f;
    }
    gate_ = 0.0f;
    half_ = false;
  }
  if (s.formant != formant_) {
    formant_ = s.formant;
    setBands(cb0_, ca1_, ca2_, formant_, sr_);
  }
  if (s.smooth != smooth_) {
    smooth_ = s.smooth;
    float sec = mapVocRelease(smooth_);
    for (int k = 0; k < kVocBands; ++k) {
      // a low band must not fall faster than two of its own cycles, or its level would shake
      float bandSec = maxf(sec, 2.0f / vocBandHz(k));
      fall_[k] = 1.0f - exp2f_(-1.4427f / (bandSec * 0.5f * sr_));
    }
  }
  gate_ += ((s.playing ? 1.0f : 0.0f) - gate_) * gateCoef_;
  // The noise is for sounds made with breath alone (S, T, F, SH), which
  // are nearly all in the top bands. A vowel has most of its strength
  // lower down, and gets none: it would only turn breathy.
  float low = 0.0f, high = 0.0f;
  for (int k = 0; k < kVocBands - kVocHissBands - 3; ++k) low += env_[k];
  for (int k = kVocBands - kVocHissBands; k < kVocBands; ++k) high += env_[k];
  float breath = high > 1e-6f ? clampf((high - 0.5f * low) / high, 0.0f, 1.0f) : 0.0f;
  float hiss = kHiss * s.consonants * gate_ * breath;
  float target = want ? s.amount : 0.0f;
  float step = (target - amount_) / (float)n;

  for (int i = 0; i < n; ++i) {
    float v = voice ? voice[i] : 0.0f;
    float carrier = 0.5f * (busL[i] + busR[i]);
    rng_ = rng_ * 1664525u + 1013904223u;
    float bright = carrier + hiss * ((float)(int32_t)rng_ * (1.0f / 2147483648.0f));
    // The voice's bands and their levels, on every second sample: the
    // two samples of a pair are averaged, which also keeps what lies above
    // the half rate's reach from folding back into the bands.
    if (!half_) {
      held_ = v;
      half_ = true;
    } else {
      half_ = false;
      float m = 0.5f * (held_ + v);
      for (int k = 0; k < kVocBands; ++k) {
        float* z = vz_[k];
        float b0 = vb0_[k], a1 = va1_[k], a2 = va2_[k];
        float y = b0 * m + z[0];
        z[0] = z[1] - a1 * y;
        z[1] = -b0 * m - a2 * y;
        float y2 = b0 * y + z[2];
        z[2] = z[3] - a1 * y2;
        z[3] = -b0 * y - a2 * y2;
        float level = absf(y2), e = env_[k];
        e += (level - e) * (level > e ? attack_ : fall_[k]);
        env_[k] = e;
        // neighbouring bands are added with opposite signs: where two meet
        // they would otherwise partly cancel and leave a dip between every pair
        float g = minf(e * weight_[k], kMaxBandGain);
        gain_[k] = (k & 1) ? -g : g;
      }
    }
    // the synth's bands, each turned up and down by the voice's
    float wet = 0.0f;
    for (int k = 0; k < kVocBands; ++k) {
      float x = k >= kVocBands - kVocHissBands ? bright : carrier;
      float* z = cz_[k];
      float b0 = cb0_[k], a1 = ca1_[k], a2 = ca2_[k];
      float y = b0 * x + z[0];
      z[0] = z[1] - a1 * y;
      z[1] = -b0 * x - a2 * y;
      float y2 = b0 * y + z[2];
      z[2] = z[3] - a1 * y2;
      z[3] = -b0 * y - a2 * y2;
      wet += y2 * gain_[k];
    }
    amount_ += step;
    float keep = 1.0f - amount_;
    busL[i] = busL[i] * keep + wet * amount_;
    busR[i] = busR[i] * keep + wet * amount_;
  }
  amount_ = target;

  for (int k = 0; k < kVocBands; ++k) {
    for (int i = 0; i < 4; ++i) {
      vz_[k][i] = tiny0(vz_[k][i]);
      cz_[k][i] = tiny0(cz_[k][i]);
    }
    env_[k] = tiny0(env_[k]);
    float g = minf(env_[k] * weight_[k], kMaxBandGain) * (1.0f / kMaxBandGain);
    shown_[k] = sqrtf_(g);
  }
}

}  // namespace fw
