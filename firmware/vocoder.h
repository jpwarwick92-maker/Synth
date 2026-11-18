// The vocoder: the synth's sound, shaped by a voice at the audio input.
//
// The voice (the modulator) is split into 16 bands from 140 Hz to 7 kHz
// and the level of each band is followed. The synth's own sound (the
// carrier) is split into the same bands, and each of its bands is turned
// up and down by the level of the voice's band. What comes out is the
// synth's notes with the voice's vowels and consonants on them.
//
// It sits between the voices and the effects. With the amount at zero it is
// out of the path altogether. It works on one channel: the two sides of the
// synth's sound are added before it, and the effects after it can widen
// the result again.
//
// The voice's side runs at half the sample rate: its bands stop at 7 kHz
// and only their levels are wanted, so nothing is lost, and it takes about
// a quarter off the processor time. The synth's side runs at the full rate.
#pragma once

#include "fw_config.h"

namespace fw {

constexpr int kVocBands = 16;
constexpr int kVocHissBands = 3;  // the top bands, which Consonants feeds with noise

struct VocSettings {
  float amount;      // 0..1: how much of the synth's sound is replaced by the vocoded sound
  float formant;     // semitones the synth's bands sit above (+) or below (-) the voice's
  float consonants;  // 0..1: noise added to the carrier's top bands, so S and T are heard
  float smooth;      // 0..1: how slowly a band's level falls, from crisp to slurred
  bool playing;      // a voice is sounding: the noise is only added while one is
};

// The centre of band k of the voice, in Hz.
float vocBandHz(int k);
// How long a band's level takes to fall, in seconds, for a Smooth setting.
float mapVocRelease(float smooth);

class Vocoder {
 public:
  void init(float sampleRate);
  // busL and busR hold the synth's sound and are changed in place. `voice`
  // is the audio input as one channel, or nullptr when there is none.
  void process(float* busL, float* busR, const float* voice, int n, const VocSettings& s);

  // ---- read by the display ----
  bool on() const { return on_; }
  float band(int k) const { return shown_[k]; }  // the level of the voice's band k, 0..1

 private:
  void setBands(float* b0, float* a1, float* a2, float shiftSemis, float rate) const;

  float sr_;
  bool on_;
  float amount_;         // as it is now: it glides to the setting
  float formant_;        // the shift the synth's bands were last set for
  float smooth_;         // the Smooth setting the fall times were last set for
  float attack_;
  float gate_;           // 0..1, follows `playing`
  float gateCoef_;
  bool half_;            // the voice's side has the first of a pair of samples waiting
  float held_;
  uint32_t rng_;
  // one two-pole band-pass section's coefficients for each band: the voice's, then the synth's
  float vb0_[kVocBands], va1_[kVocBands], va2_[kVocBands];
  float cb0_[kVocBands], ca1_[kVocBands], ca2_[kVocBands];
  // each band is two such sections in a row; these are their memories
  float vz_[kVocBands][4];
  float cz_[kVocBands][4];
  float env_[kVocBands];
  float gain_[kVocBands];  // what each of the synth's bands is multiplied by now
  float fall_[kVocBands];
  float weight_[kVocBands];
  volatile float shown_[kVocBands];
};

extern Vocoder gVocoder;

}  // namespace fw
