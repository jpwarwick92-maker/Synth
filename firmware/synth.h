// The sound engine: 8 voices, each with two oscillators, a sub and noise,
// a mixer, a multimode filter, two envelopes and a shared LFO; then an
// arpeggiator and Hold in front of the voices, and effects behind them.
//
// Two contexts use it. The audio context calls process(). The control
// context (MIDI and panel handling) posts note events through a small
// lock-free queue and writes parameters into gParams; process() picks both
// up at the start of each block. Nothing here allocates memory, and no
// member has an initialiser: init() sets everything, so the global needs
// no start-up code.
#pragma once

#include "fw_config.h"
#include "params.h"

namespace fw {

struct Svf {
  float ic1, ic2;
};

enum EnvStage : uint8_t { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_RELEASE };

struct Env {
  float level;
  uint8_t stage;
};

struct EnvCoef {
  float a, d, s, r;
};

constexpr int kKsLen = 2048;  // pluck engine: string length in samples (power of two)

struct Osc {
  float phase;
  const int16_t* tab;     // VA: current band-limited table
  const int16_t* tabOld;  // VA: previous table, faded out over one control block
  float xf;             // VA: 1 -> 0 crossfade from tabOld to tab
  float phase2;         // FM: modulator phase
  float fb;             // FM: modulator feedback memory. Pluck: exciter filter memory
  float lp;             // Pluck: loop filter memory
  int16_t* ks;          // Pluck: the string (a delay line)
  int ksPos;
  int excite;           // Pluck: samples of excitation still to feed in
  int exciteLen;
  bool pluck;           // Pluck: pluck the string at the next block
  float ss[6];          // Supersaw: phases of the six saws either side of the centre one
  float blep;           // Sync: second half of the correction that smooths a reset, due next sample
};

struct Voice {
  bool active;      // producing sound
  bool gate;        // envelopes held open
  bool keyDown;     // key physically held
  bool sustained;   // key released but held by the pedal
  uint8_t note;
  float vel;        // 0..1
  uint32_t age;     // note-on order, for stealing the oldest
  float pitch;      // current pitch in MIDI note units (glides)
  float target;     // pitch being glided towards
  Osc o1, o2, sub;
  uint8_t wave1, wave2;
  uint8_t eng1, eng2;
  float lfoAmt;     // 0..1, fades the LFO in after the note starts
  float drift1, drift2, driftCut;  // slow random wander of each oscillator's pitch and of the cutoff
  uint32_t driftRng;
  Svf f1, f2;
  Env aenv, fenv;
  uint32_t rng;
};

enum EventType : uint8_t {
  EV_NOTE_ON,
  EV_NOTE_OFF,
  EV_SUSTAIN,
  EV_BEND,          // a = LSB, b = MSB (14-bit)
  EV_MOD_WHEEL,
  EV_ALL_NOTES_OFF,
  EV_ALL_SOUND_OFF,
  EV_CHORD,         // the Chord button was pressed
};

struct OscRun;  // one oscillator's settings for one control block (synth.cpp)

class Synth {
 public:
  void init(float sampleRate);

  // Control context. Returns false if the queue is full.
  bool post(uint8_t type, uint8_t a = 0, uint8_t b = 0);

  // Audio context. Stereo output, n <= kMaxBlock.
  void process(float* outL, float* outR, int n);

  // ---- read by the display (control context) ----
  int activeVoices() const { return activeCount_; }
  bool voiceActive(int i) const { return voices_[i].active; }
  float peakLevel() const { return peak_; }
  float sampleRate() const { return sr_; }
  bool chordOn() const { return chordOn_; }
  const float* scope() const { return scope_; }
  uint32_t scopeWritePos() const { return scopePos_; }

 private:
  struct Patch;

  void handleEvent(uint8_t type, uint8_t a, uint8_t b);
  // keys as played, before Hold and the arpeggiator
  void keyDown(uint8_t note, uint8_t velocity);
  void keyUp(uint8_t note);
  void dropUnheld();
  void toggleChord();
  int keyNotes(int i, uint8_t* out) const;
  bool neededElsewhere(uint8_t note, int except) const;
  void soundKey(int i);
  void silenceKey(int i);
  void removeKey(int i);
  bool latching() const { return hold_ || (arpMode_ != ARP_OFF && pedal_); }
  void arpAdvance(int len);
  int arpPick(uint8_t& velocity);
  // notes as the voices get them
  void noteOn(uint8_t note, uint8_t velocity);
  void noteOff(uint8_t note);
  void setSustain(bool down);
  void releaseAll();
  void killAll();
  int pickVoice(uint8_t note);
  void trigger(Voice& v, uint8_t note, float vel);
  void releaseVoice(Voice& v);
  void derive(Patch& p, int len);
  void renderVoice(Voice& v, int index, const Patch& p, float* outL, float* outR, int n);
  void prepOsc(Osc& o, OscRun& r, uint8_t& lastEngine, uint8_t& lastWave, uint8_t engine, uint8_t wave, float inc,
               float macro, float color, float pw);

  float sr_;
  Voice voices_[kNumVoices];
  uint32_t clock_;
  uint8_t mode_;
  float lastPitch_;

  // held-note stack for mono and unison (last note wins)
  uint8_t stackNote_[16];
  uint8_t stackVel_[16];
  int stackN_;

  // keys physically down, and the notes being played: keys down plus any
  // that Hold has latched, in the order they were played
  uint8_t physNote_[16];
  int physN_;
  uint8_t setNote_[16];
  uint8_t setVel_[16];
  bool setChord_[16];         // this key plays the remembered chord, not just itself
  int setN_;

  // chord memory: the shape of a chord, as semitones above its lowest note
  bool chordOn_;
  int8_t chordStep_[6];
  int chordN_;
  bool hold_;
  bool pedal_;

  uint8_t arpMode_;
  float arpClock_;            // samples since the last step
  float arpGateLeft_;         // samples until the sounding note is let go
  int arpIndex_;
  int arpNote_;               // note the arpeggiator has sounding, or -1
  bool arpRestart_;
  uint32_t arpRng_;

  bool sustain_;
  float bend_;                // -1..1 target
  float bendSmooth_;
  float modWheel_;            // 0..1

  float lfoPhase_;
  float lfoValue_;
  float lfoRandom_;
  uint32_t lfoRng_;

  float smooth_[P_COUNT];     // smoothed pot positions, 0..1
  float smoothCoef_;          // per control block
  float peakDecay_;           // per control block

  // event queue, single producer / single consumer
  struct Event {
    uint8_t type, a, b;
  };
  static constexpr int kQueueLen = 64;
  Event queue_[kQueueLen];
  volatile uint32_t qWrite_;
  volatile uint32_t qRead_;

  // display feeds
  int activeCount_;
  float peak_;
  FW_BIGMEM static float scope_[kScopeLen];
  volatile uint32_t scopePos_;
};

extern Synth gSynth;

}  // namespace fw
