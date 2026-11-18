// Effects after the voices: chorus, delay, reverb, in that order.
//
// Stereo in, stereo out. Each effect is skipped completely when its amount
// is zero, so with everything off the output is exactly the input. No allocation: every delay line is a fixed static buffer.
#pragma once

#include "fw_config.h"

namespace fw {

struct FxSettings {
  float chorus;    // 0..1 amount
  bool ensemble;   // three swirling copies in place of the chorus's two
  float dlyTime;   // seconds
  float dlyTone;   // 0..1, dark to bright repeats
  float dlyFdbk;   // 0..1
  float dlyMix;    // 0..1
  float rvbTime;   // seconds to die away
  float rvbTone;   // 0..1, dark to bright tail
  float rvbMix;    // 0..1
  float compress;  // 0..1 amount of the compressor
  float duck;      // 0..1 depth of the dip on every beat
  float beatPhase; // where in the beat this block starts, 0..1
  float beatStep;  // how far the beat moves on per sample
  float beatSec;   // one beat in seconds
};

class Effects {
 public:
  void init(float sampleRate);
  // n samples in each buffer, n <= kMaxBlock.
  void process(const float* inL, const float* inR, float* outL, float* outR, int n, const FxSettings& s);

 private:
  float sr_;
  // chorus
  bool chorusOn_;
  int chPos_;
  float chPhase_;
  float chFast_;     // the ensemble's second, faster sweep
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
  float rvScale_;    // 1, or less when the sample rate is too high for the lines' full lengths
  float rvLp_[4];
  int apPos_[2];
  int apLen_[2];
  // compressor and duck
  bool compOn_;
  float cEnv_;       // the level it is following
  float cGain_;      // the gain it is applying now
  float cUp_, cDown_;  // how fast the level follower rises and falls, per sample
  float duckGain_;
};

extern Effects gFx;

}  // namespace fw
