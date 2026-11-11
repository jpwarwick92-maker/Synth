// Effects after the voices: chorus, delay, reverb, in that order.
//
// Mono in, stereo out. Each effect is skipped completely when its amount is
// zero, so with everything off the output is exactly the dry signal on both
// channels. No allocation: every delay line is a fixed static buffer.
#pragma once

#include "fw_config.h"

namespace fw {

struct FxSettings {
  float chorus;    // 0..1 amount
  float dlyTime;   // seconds
  float dlyTone;   // 0..1, dark to bright repeats
  float dlyFdbk;   // 0..1
  float dlyMix;    // 0..1
  float rvbTime;   // seconds to die away
  float rvbTone;   // 0..1, dark to bright tail
  float rvbMix;    // 0..1
};

class Effects {
 public:
  void init(float sampleRate);
  // in: n mono samples. outL/outR: n samples each. n <= kMaxBlock.
  void process(const float* in, float* outL, float* outR, int n, const FxSettings& s);

 private:
  float sr_;
  // chorus
  bool chorusOn_;
  int chPos_;
  float chPhase_;
  // delay
  bool delayOn_;
  int dlPos_;
  float dlLen_;      // current delay in samples, glides to the set time
  float dlGlide_;
  float dlLp_;
  // reverb
  bool reverbOn_;
  int rvPos_;
  int rvLen_[4];
  float rvLp_[4];
  int apPos_[2];
  int apLen_[2];
};

extern Effects gFx;

}  // namespace fw
