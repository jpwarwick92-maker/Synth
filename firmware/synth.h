// The sound engine: 8 voices, each with two oscillators, a sub and noise,
// a mixer, a multimode filter, two envelopes and a shared LFO.
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

struct Osc {
  float phase;
  const float* tab;     // current band-limited table
  const float* tabOld;  // previous table, faded out over one control block
  float xf;             // 1 -> 0 crossfade from tabOld to tab
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
};

class Synth {
 public:
  void init(float sampleRate);

  // Control context. Returns false if the queue is full.
  bool post(uint8_t type, uint8_t a = 0, uint8_t b = 0);

  // Audio context. Mono output, n <= kMaxBlock.
  void process(float* out, int n);

  // ---- read by the display (control context) ----
  int activeVoices() const { return activeCount_; }
  bool voiceActive(int i) const { return voices_[i].active; }
  float peakLevel() const { return peak_; }
  float sampleRate() const { return sr_; }
  const float* scope() const { return scope_; }
  uint32_t scopeWritePos() const { return scopePos_; }

 private:
  struct Patch;

  void handleEvent(uint8_t type, uint8_t a, uint8_t b);
  void noteOn(uint8_t note, uint8_t velocity);
  void noteOff(uint8_t note);
  void setSustain(bool down);
  void releaseAll();
  void killAll();
  int pickVoice(uint8_t note);
  void trigger(Voice& v, uint8_t note, float vel);
  void releaseVoice(Voice& v);
  void derive(Patch& p, int len);
  void renderVoice(Voice& v, int index, const Patch& p, float* out, int n);

  float sr_;
  Voice voices_[kNumVoices];
  uint32_t clock_;
  uint8_t mode_;
  float lastPitch_;

  // held-note stack for mono and unison (last note wins)
  uint8_t stackNote_[16];
  uint8_t stackVel_[16];
  int stackN_;

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
