#include "synth.h"

#include "fw_math.h"
#include "fx.h"
#include "tables.h"

namespace fw {

Synth gSynth;
FW_BIGMEM float Synth::scope_[kScopeLen];

namespace {
// One string per oscillator per voice for the pluck engine. 16-bit keeps
// this to 64 KB; the string's own decay hides the rounding.
int16_t gStrings[kNumVoices][2][kKsLen];

const float kFmRatios[WAVE_COUNT] = {1.0f, 2.0f, 3.0f, 3.5f, 4.0f};

// Supersaw: how far each of the six side saws sits from the centre one at
// full detune (as a fraction of its frequency), and how loud the sides are.
const float kSuperDetune[6] = {-0.11002313f, -0.06288439f, -0.01952356f, 0.01991221f, 0.06216538f, 0.10745242f};
constexpr float kSuperSide = 0.6f, kSuperGain = 0.42f;

// Each voice's place in the stereo field at full Spread, and its fixed
// share of mistuning at full Drift (as analog voice cards differ).
const float kVoicePan[kNumVoices] = {-1.0f, 1.0f, -0.6f, 0.6f, -0.3f, 0.3f, -0.8f, 0.8f};
const float kVoiceTune[kNumVoices] = {0.3f, -0.7f, 1.0f, -0.2f, -1.0f, 0.6f, -0.4f, 0.8f};
const float kVoiceCut[kNumVoices] = {-0.5f, 0.9f, 0.2f, -1.0f, 0.6f, -0.3f, 1.0f, -0.8f};
}  // namespace

// Everything the voices need for one control block, worked out once from
// the (smoothed) parameters.
struct Synth::Patch {
  uint8_t wave1, wave2, ftype;
  uint8_t eng1, eng2;
  bool slope24, ladder, driveOn;
  float oct1, oct2;      // semitones
  float detune2;         // semitones
  float macro1, macro2;  // 0..1: each oscillator's main timbre control
  float color1, color2;  // 0..1: its second one, where the engine has one
  float mix[4];          // osc1, osc2, sub, noise
  float cutoffOct;       // octaves above 20 Hz
  float lfoCut;          // octaves
  float lfoPw;           // added to pulse width, or to Macro on the other engines
  float lfoFadeStep;     // how far the LFO fades in per block (1 = at once)
  float velCut;          // octaves at full velocity
  float fenvPitch;       // semitones at full envelope
  float k, comp;         // filter damping, resonance loudness trim
  float ladderK, ladderComp;  // ladder: feedback amount, and the level it costs put back
  float envAmt;          // octaves
  float keytrack;        // 0..1
  DriveSettings drive;
  float glideCoef;       // 1 = no glide
  EnvCoef aenv, fenv;
  float velAmt;
  float lfoPitch;        // semitones
  float tremGain;
  float bendSemis;
  float uniDetune;       // semitones, half the total spread
  float subDiv;          // sub frequency as a fraction of osc 1
  float volume;
  float drift, driftCoef;  // 0..1, and how fast the wander moves per block
  float spread;            // 0..1
  float xmod;              // 0..1: oscillator 2 bending oscillator 1
  bool sync;               // oscillator 2 restarts with each cycle of oscillator 1
  FxSettings fx;
};

// One oscillator's settings for one control block.
struct OscRun {
  uint8_t engine, wave;
  float inc, pw;
  float fmIndex, fmRatio, fmFeedback;  // index and feedback in cycles
  const int16_t* wtA;
  const int16_t* wtB;
  float wtMix;
  float ksDelay, ksDamp, ksGain;
  float ssInc[6];  // supersaw: the six side saws' speeds
};

namespace {

inline float softClip(float x) {
  float a = absf(x);
  if (a <= 0.7f) return x;  // exactly linear at normal levels
  float y = 0.7f + 0.3f * tanhf_((a - 0.7f) * (1.0f / 0.3f));
  return x < 0.0f ? -y : y;
}

inline float wrap01(float x) {
  x -= floorf_(x);
  return x >= 1.0f ? 0.0f : x;
}

inline float nextNoise(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return (float)(int32_t)s * (1.0f / 2147483648.0f);
}

inline float envTick(Env& e, const EnvCoef& c) {
  switch (e.stage) {
    case ENV_ATTACK:
      e.level = 1.3f + (e.level - 1.3f) * c.a;
      if (e.level >= 1.0f) {
        e.level = 1.0f;
        e.stage = ENV_DECAY;
      }
      break;
    case ENV_DECAY: {
      float d = e.level - c.s;
      e.level = absf(d) < 1e-5f ? c.s : c.s + d * c.d;
      break;
    }
    case ENV_RELEASE:
      e.level *= c.r;
      if (e.level < 1e-4f) {
        e.level = 0.0f;
        e.stage = ENV_IDLE;
      }
      break;
    default:
      break;
  }
  return e.level;
}

inline float readTable(const Osc& o, float ph) {
  float y = tabRead16(o.tab, ph);
  if (o.xf > 0.0f) y += (tabRead16(o.tabOld, ph) - y) * o.xf;
  return y;
}

inline void advance(Osc& o, float inc) {
  float ph = o.phase + inc;
  if (ph >= 1.0f) ph -= 1.0f;
  o.phase = ph;
  if (o.xf > 0.0f) {
    o.xf -= 1.0f / (float)kCtrlBlock;
    if (o.xf < 0.0f) o.xf = 0.0f;
  }
}

// A pulse is the difference of two saws, so it is band-limited too.
inline float pulseAt(const Osc& o, float ph, float pw) {
  float ph2 = ph + 1.0f - pw;
  if (ph2 >= 1.0f) ph2 -= 1.0f;
  return readTable(o, ph) - readTable(o, ph2) - (2.0f * pw - 1.0f);
}

inline void setTable(Osc& o, const int16_t* t, bool fade) {
  if (t == o.tab) return;
  o.tabOld = fade ? o.tab : t;
  o.xf = fade ? 1.0f : 0.0f;
  o.tab = t;
}

// What starts a plucked string moving, chosen by the Wave selector.
inline float exciteSample(Osc& o, uint8_t type, uint32_t& rng) {
  int n = o.exciteLen - o.excite;
  switch (type) {
    case 0: return nextNoise(rng);                                     // bright noise
    case 1:                                                            // soft noise
      o.fb += 0.25f * (nextNoise(rng) - o.fb);
      return o.fb * 2.0f;
    case 2: return n * 4 < o.exciteLen ? nextNoise(rng) * 1.5f : 0.0f;   // a short snap
    case 3: return 2.0f * (float)n / (float)o.exciteLen - 1.0f;        // a ramp: pitched, string-like
    default: {                                                         // a thin, bright pick
      float nz = nextNoise(rng);
      float y = nz - o.fb;
      o.fb = nz;
      return y * 0.7f;
    }
  }
}

// An oscillator's output now, whichever engine it is running. `pm` shifts
// the phase it is read at (cross-modulation), in cycles. Does not move the
// oscillator on, except for the pluck engine, whose string steps as it is read.
inline float oscValue(Osc& o, const OscRun& r, float pm, uint32_t& rng) {
  switch (r.engine) {
    case ENG_VA: {
      float ph = pm == 0.0f ? o.phase : wrap01(o.phase + pm);
      switch (r.wave) {
        case WAVE_SAW:
        case WAVE_TRI: return readTable(o, ph);
        case WAVE_SQR: return pulseAt(o, ph, r.pw);
        case WAVE_SUPER: {  // seven saws: one in tune, three either side
          float sides = 0.0f;
          for (int k = 0; k < 6; ++k) sides += tabRead16(o.tab, pm == 0.0f ? o.ss[k] : wrap01(o.ss[k] + pm));
          return (readTable(o, ph) + kSuperSide * sides) * kSuperGain;
        }
        default: return tabRead(gSine, ph);
      }
    }
    case ENG_FM: {  // one sine bending the phase of another
      float m = tabRead(gSine, wrap01(o.phase2 + r.fmFeedback * o.fb));
      o.fb = m;
      return tabRead(gSine, wrap01(o.phase + pm + r.fmIndex * m));
    }
    case ENG_WAVETABLE: {  // morph between two neighbouring frames
      float p = (pm == 0.0f ? o.phase : wrap01(o.phase + pm)) * (float)kWtLen;
      int i = (int)p;
      float f = p - (float)i;
      const int m = kWtLen - 1;
      int i0 = (i - 1) & m, i1 = i & m, i2 = (i + 1) & m, i3 = (i + 2) & m;
      // blend the frames first, then read through four points: the short
      // tables need the smoother curve to stay clean on rich waves
      float y0 = (float)r.wtA[i0] + (float)(r.wtB[i0] - r.wtA[i0]) * r.wtMix;
      float y1 = (float)r.wtA[i1] + (float)(r.wtB[i1] - r.wtA[i1]) * r.wtMix;
      float y2 = (float)r.wtA[i2] + (float)(r.wtB[i2] - r.wtA[i2]) * r.wtMix;
      float y3 = (float)r.wtA[i3] + (float)(r.wtB[i3] - r.wtA[i3]) * r.wtMix;
      float c1 = 0.5f * (y2 - y0);
      float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
      float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
      return (((c3 * f + c2) * f + c1) * f + y1) * (1.0f / 32000.0f);
    }
    default: {  // pluck: a delay line fed back through a gentle filter
      float rp = (float)o.ksPos - r.ksDelay;
      if (rp < 0.0f) rp += (float)kKsLen;
      int i0 = (int)rp;
      float f = rp - (float)i0;
      int i1 = (i0 + 1) & (kKsLen - 1);
      float y = ((float)o.ks[i0] + (float)(o.ks[i1] - o.ks[i0]) * f) * (1.0f / 32767.0f);
      o.lp += r.ksDamp * (y - o.lp);
      float in = o.lp * r.ksGain;
      if (o.excite > 0) {
        float e = exciteSample(o, r.wave, rng);
        --o.excite;
        in += e;
        y += e;  // heard at once, not one trip round the string later
      }
      o.ks[o.ksPos] = (int16_t)(clampf(in, -1.0f, 1.0f) * 32767.0f);
      o.ksPos = (o.ksPos + 1) & (kKsLen - 1);
      return y;
    }
  }
}

// Moves an oscillator on by one sample.
inline void oscStep(Osc& o, const OscRun& r) {
  if (r.engine == ENG_FM) {
    o.phase2 = wrap01(o.phase2 + r.inc * r.fmRatio);
  } else if (r.engine == ENG_VA && r.wave == WAVE_SUPER) {
    for (int k = 0; k < 6; ++k) {
      float ph = o.ss[k] + r.ssInc[k];
      o.ss[k] = ph >= 1.0f ? ph - 1.0f : ph;
    }
  }
  advance(o, r.inc);
}

struct SvfCoef {
  float a1, a2, a3, k, bpGain;
};

inline SvfCoef svfCoef(float g, float k) {
  SvfCoef c;
  c.a1 = 1.0f / (1.0f + g * (g + k));
  c.a2 = g * c.a1;
  c.a3 = g * c.a2;
  c.k = k;
  c.bpGain = k > 1.0f ? k : 1.0f;
  return c;
}

// State-variable filter, trapezoidal integration (stable at any cutoff).
inline float svfTick(Svf& s, float v0, const SvfCoef& c, uint8_t type) {
  float v3 = v0 - s.ic2;
  float v1 = c.a1 * s.ic1 + c.a2 * v3;
  float v2 = s.ic2 + c.a2 * s.ic1 + c.a3 * v3;
  s.ic1 = 2.0f * v1 - s.ic1;
  s.ic2 = 2.0f * v2 - s.ic2;
  switch (type) {
    case FILT_LP: return v2;
    case FILT_BP: return v1 * c.bpGain;
    default: return v0 - c.k * v1 - v2;
  }
}

// Removes `note` from a list of notes with matching velocities.
inline void dropNote(uint8_t* notes, uint8_t* vels, int& n, uint8_t note) {
  int w = 0;
  for (int i = 0; i < n; ++i)
    if (notes[i] != note) {
      notes[w] = notes[i];
      if (vels) vels[w] = vels[i];
      ++w;
    }
  n = w;
}

}  // namespace

void Synth::init(float sampleRate) {
  sr_ = sampleRate;
  clock_ = 0;
  mode_ = (uint8_t)gParams.step(P_VOICE_MODE);
  lastPitch_ = 60.0f;
  stackN_ = 0;
  physN_ = setN_ = 0;
  chordOn_ = false;
  chordN_ = 0;
  hold_ = gParams.step(P_HOLD) != 0;
  pedal_ = false;
  arpMode_ = (uint8_t)gParams.step(P_ARP_MODE);
  arpClock_ = arpGateLeft_ = 0.0f;
  arpIndex_ = 0;
  arpNote_ = -1;
  arpRestart_ = true;
  arpRng_ = 0x2545F491u;
  sustain_ = false;
  bend_ = bendSmooth_ = 0.0f;
  modWheel_ = 0.0f;
  lfoPhase_ = lfoValue_ = lfoRandom_ = 0.0f;
  lfoRng_ = 0x1234567u;
  qWrite_ = qRead_ = 0;
  activeCount_ = 0;
  peak_ = 0.0f;
  scopePos_ = 0;
  for (int i = 0; i < kScopeLen; ++i) scope_[i] = 0.0f;

  // one-pole coefficients for a 16-sample control block
  smoothCoef_ = 1.0f - exp2f_(-1.4427f * (float)kCtrlBlock / (0.006f * sr_));
  peakDecay_ = exp2f_(-1.4427f * (float)kCtrlBlock / (0.25f * sr_));

  for (int i = 0; i < P_COUNT; ++i) smooth_[i] = gParams.norm(i);

  const int16_t* rest = gSaw[kSawLevels - 1];
  for (int i = 0; i < kNumVoices; ++i) {
    Voice& v = voices_[i];
    v.active = v.gate = v.keyDown = v.sustained = false;
    v.note = 60;
    v.vel = 0.0f;
    v.age = 0;
    v.pitch = v.target = 60.0f;
    v.o1 = Osc{(float)i * 0.123f, rest, rest, 0.0f, 0.0f, 0.0f, 0.0f, gStrings[i][0], 0, 0, 1, false, {}, 0.0f};
    v.o2 = Osc{(float)i * 0.071f + 0.3f, rest, rest, 0.0f, 0.0f, 0.0f, 0.0f, gStrings[i][1], 0, 0, 1, false, {}, 0.0f};
    v.sub = Osc{0.0f, rest, rest, 0.0f, 0.0f, 0.0f, 0.0f, nullptr, 0, 0, 1, false, {}, 0.0f};
    for (int k = 0; k < 6; ++k) {  // the supersaw's saws start scattered, so they never line up into one
      v.o1.ss[k] = wrap01(0.137f * (float)(k + 1) + 0.071f * (float)i);
      v.o2.ss[k] = wrap01(0.211f * (float)(k + 1) + 0.053f * (float)i + 0.4f);
    }
    v.drift1 = v.drift2 = v.driftCut = 0.0f;
    v.driftRng = 0x6C8E9CF5u * (uint32_t)(i + 3) | 1u;
    v.wave1 = v.wave2 = WAVE_SAW;
    v.eng1 = v.eng2 = ENG_VA;
    v.lfoAmt = 1.0f;
    v.f1 = Svf{0.0f, 0.0f};
    v.f2 = Svf{0.0f, 0.0f};
    v.aenv = Env{0.0f, ENV_IDLE};
    v.fenv = Env{0.0f, ENV_IDLE};
    v.rng = 0x9E3779B9u * (uint32_t)(i + 1) | 1u;
    for (int o = 0; o < 2; ++o)
      for (int k = 0; k < kKsLen; ++k) gStrings[i][o][k] = 0;
  }
  gFx.init(sampleRate);
}

bool Synth::post(uint8_t type, uint8_t a, uint8_t b) {
  uint32_t w = qWrite_;
  uint32_t next = (w + 1) % kQueueLen;
  if (next == qRead_) return false;
  queue_[w] = Event{type, a, b};
  qWrite_ = next;
  return true;
}

void Synth::handleEvent(uint8_t type, uint8_t a, uint8_t b) {
  switch (type) {
    case EV_NOTE_ON: keyDown(a, b); break;
    case EV_NOTE_OFF: keyUp(a); break;
    case EV_SUSTAIN:
      pedal_ = a >= 64;
      if (arpMode_ != ARP_OFF) {  // with the arpeggiator running the pedal is a temporary Hold
        if (!pedal_ && !hold_) dropUnheld();
      } else {
        setSustain(pedal_);
      }
      break;
    case EV_BEND: bend_ = (float)(((int)a | ((int)b << 7)) - 8192) * (1.0f / 8192.0f); break;
    case EV_MOD_WHEEL: modWheel_ = (float)a * (1.0f / 127.0f); break;
    case EV_ALL_NOTES_OFF:
      releaseAll();
      physN_ = setN_ = 0;
      arpNote_ = -1;
      break;
    case EV_ALL_SOUND_OFF: killAll(); break;
    case EV_CHORD: toggleChord(); break;
    default: break;
  }
}

// ---------------------------------------------------------------- keys, Hold, arpeggiator

// The notes one played key sounds: itself, or, with chord memory on, the
// remembered chord built up from it. Returns how many.
int Synth::keyNotes(int i, uint8_t* out) const {
  if (!chordOn_ || !setChord_[i]) {
    out[0] = setNote_[i];
    return 1;
  }
  int n = 0;
  for (int k = 0; k < chordN_; ++k) {
    int note = setNote_[i] + chordStep_[k];
    if (note <= 127) out[n++] = (uint8_t)note;
  }
  return n;
}

// Is `note` also sounded by a key other than number `except`?
bool Synth::neededElsewhere(uint8_t note, int except) const {
  uint8_t notes[6];
  for (int i = 0; i < setN_; ++i) {
    if (i == except) continue;
    int n = keyNotes(i, notes);
    for (int k = 0; k < n; ++k)
      if (notes[k] == note) return true;
  }
  return false;
}

void Synth::soundKey(int i) {
  uint8_t notes[6];
  int n = keyNotes(i, notes);
  for (int k = 0; k < n; ++k) noteOn(notes[k], setVel_[i]);
}

// Stops what a key sounds, except notes another held key is also sounding.
void Synth::silenceKey(int i) {
  uint8_t notes[6];
  int n = keyNotes(i, notes);
  for (int k = 0; k < n; ++k)
    if (!neededElsewhere(notes[k], i)) noteOff(notes[k]);
}

void Synth::removeKey(int i) {
  for (int k = i + 1; k < setN_; ++k) {
    setNote_[k - 1] = setNote_[k];
    setVel_[k - 1] = setVel_[k];
    setChord_[k - 1] = setChord_[k];
  }
  --setN_;
}

void Synth::keyDown(uint8_t note, uint8_t velocity) {
  bool arp = arpMode_ != ARP_OFF;
  // With Hold on, the first key after every key has been let go starts a
  // fresh chord: what was latched gives way to it.
  if (latching() && physN_ == 0 && setN_ > 0) {
    if (!arp)
      for (int i = setN_ - 1; i >= 0; --i) {
        silenceKey(i);
        removeKey(i);
      }
    setN_ = 0;
    arpRestart_ = true;
  }

  dropNote(physNote_, nullptr, physN_, note);
  if (physN_ < 16) physNote_[physN_++] = note;

  for (int i = 0; i < setN_; ++i)
    if (setNote_[i] == note) {  // the same key again: it is replaced, and sounds afresh below
      removeKey(i);
      break;
    }
  if (setN_ == 16) {
    if (!arp) silenceKey(0);
    removeKey(0);
  }
  setNote_[setN_] = note;
  setVel_[setN_] = velocity;
  setChord_[setN_] = chordOn_;
  ++setN_;

  if (!arp) soundKey(setN_ - 1);
  else if (setN_ == 1) arpRestart_ = true;  // first note: start on it at once
}

void Synth::keyUp(uint8_t note) {
  dropNote(physNote_, nullptr, physN_, note);
  if (latching()) return;  // Hold keeps it sounding
  for (int i = 0; i < setN_; ++i)
    if (setNote_[i] == note) {
      if (arpMode_ == ARP_OFF) silenceKey(i);
      removeKey(i);
      return;
    }
}

// Hold (or the pedal) was let go: every note without a finger on it stops.
void Synth::dropUnheld() {
  for (int i = setN_ - 1; i >= 0; --i) {
    bool down = false;
    for (int k = 0; k < physN_; ++k) down = down || physNote_[k] == setNote_[i];
    if (down) continue;
    if (arpMode_ == ARP_OFF) silenceKey(i);
    removeKey(i);
  }
}

// The Chord button. Off -> on: remember the shape of the chord being held
// (two notes or more); from then on each key plays that shape. On -> off:
// forget it, and keys that were playing it go back to single notes.
void Synth::toggleChord() {
  if (chordOn_) {
    for (int i = 0; i < setN_; ++i) {
      if (!setChord_[i]) continue;
      uint8_t notes[6];
      int n = keyNotes(i, notes);
      setChord_[i] = false;
      if (arpMode_ != ARP_OFF) continue;
      for (int k = 0; k < n; ++k)
        if (notes[k] != setNote_[i] && !neededElsewhere(notes[k], -1)) noteOff(notes[k]);
    }
    chordOn_ = false;
    return;
  }
  if (setN_ < 2) return;  // nothing held to learn from
  uint8_t sorted[16];
  for (int i = 0; i < setN_; ++i) sorted[i] = setNote_[i];
  for (int i = 1; i < setN_; ++i) {
    uint8_t v = sorted[i];
    int j = i - 1;
    for (; j >= 0 && sorted[j] > v; --j) sorted[j + 1] = sorted[j];
    sorted[j + 1] = v;
  }
  chordN_ = setN_ < 6 ? setN_ : 6;
  for (int i = 0; i < chordN_; ++i) chordStep_[i] = (int8_t)(sorted[i] - sorted[0]);
  chordOn_ = true;  // the keys held now carry on as they are; the next key plays the chord
}

// Which note the arpeggiator plays next, or -1 with nothing held.
int Synth::arpPick(uint8_t& velocity) {
  if (setN_ == 0) return -1;
  // every note the held keys sound, in the order played, each once
  uint8_t notes[32], vels[32];
  int count = 0;
  for (int i = 0; i < setN_; ++i) {
    uint8_t key[6];
    int n = keyNotes(i, key);
    for (int k = 0; k < n && count < 32; ++k) {
      bool have = false;
      for (int c = 0; c < count; ++c) have = have || notes[c] == key[k];
      if (have) continue;
      notes[count] = key[k];
      vels[count] = setVel_[i];
      ++count;
    }
  }
  if (arpMode_ != ARP_PLAYED) {  // every pattern but "as played" works from low to high
    for (int i = 1; i < count; ++i) {
      uint8_t n = notes[i], v = vels[i];
      int j = i - 1;
      for (; j >= 0 && notes[j] > n; --j) {
        notes[j + 1] = notes[j];
        vels[j + 1] = vels[j];
      }
      notes[j + 1] = n;
      vels[j + 1] = v;
    }
  }
  int octaves = gParams.step(P_ARP_OCTAVES) + 1;
  int m = count * octaves;
  int k;
  switch (arpMode_) {
    case ARP_DOWN: k = m - 1 - arpIndex_ % m; break;
    case ARP_UPDOWN: {
      int period = m > 1 ? 2 * m - 2 : 1;
      int j = arpIndex_ % period;
      k = j < m ? j : 2 * m - 2 - j;
      break;
    }
    case ARP_RANDOM:
      arpRng_ ^= arpRng_ << 13;
      arpRng_ ^= arpRng_ >> 17;
      arpRng_ ^= arpRng_ << 5;
      k = (int)(arpRng_ % (uint32_t)m);
      break;
    default: k = arpIndex_ % m; break;  // up, and as played
  }
  int note = notes[k % count] + 12 * (k / count);
  while (note > 127) note -= 12;
  velocity = vels[k % count];
  return note;
}

void Synth::arpAdvance(int len) {
  if (arpMode_ == ARP_OFF) return;
  if (setN_ == 0) {
    if (arpNote_ >= 0) {
      noteOff((uint8_t)arpNote_);
      arpNote_ = -1;
    }
    arpRestart_ = true;
    return;
  }
  // sixteenth notes at the set tempo
  float bpm = mapTempoBpm(gParams.norm(P_ARP_TEMPO));
  float stepLen = sr_ * 60.0f / (bpm * 4.0f);
  if (arpRestart_) {
    arpClock_ = stepLen;
    arpIndex_ = 0;
    arpRestart_ = false;
  }
  arpClock_ += (float)len;
  if (arpNote_ >= 0) {
    arpGateLeft_ -= (float)len;
    if (arpGateLeft_ <= 0.0f) {
      noteOff((uint8_t)arpNote_);
      arpNote_ = -1;
    }
  }
  if (arpClock_ < stepLen) return;
  arpClock_ -= stepLen;
  if (arpNote_ >= 0) noteOff((uint8_t)arpNote_);
  uint8_t vel = 100;
  arpNote_ = arpPick(vel);
  ++arpIndex_;
  if (arpNote_ < 0) return;
  noteOn((uint8_t)arpNote_, vel);
  float gate = mapArpGate(gParams.norm(P_ARP_GATE));
  arpGateLeft_ = gate >= 0.999f ? stepLen * 2.0f : stepLen * gate;  // full gate: held until the next step
}

// ---------------------------------------------------------------- voices

void Synth::trigger(Voice& v, uint8_t note, float vel) {
  if (!v.active) {
    bool glideOn = gParams.norm(P_GLIDE) >= 0.01f;
    v.pitch = glideOn ? lastPitch_ : (float)note;
    v.f1 = Svf{0.0f, 0.0f};
    v.f2 = Svf{0.0f, 0.0f};
    v.aenv.level = 0.0f;
    v.fenv.level = 0.0f;
  }
  v.active = true;
  v.gate = true;
  v.keyDown = true;
  v.sustained = false;
  v.note = note;
  v.vel = vel;
  v.age = ++clock_;
  v.target = (float)note;
  v.aenv.stage = ENV_ATTACK;
  v.fenv.stage = ENV_ATTACK;
  v.lfoAmt = 0.0f;
  v.o1.pluck = v.o2.pluck = true;
}

void Synth::releaseVoice(Voice& v) {
  v.gate = false;
  v.keyDown = false;
  v.sustained = false;
  if (v.aenv.stage != ENV_IDLE) v.aenv.stage = ENV_RELEASE;
  if (v.fenv.stage != ENV_IDLE) v.fenv.stage = ENV_RELEASE;
}

int Synth::pickVoice(uint8_t note) {
  // the same key played again reuses its voice
  for (int i = 0; i < kNumVoices; ++i) {
    const Voice& v = voices_[i];
    if (v.active && v.note == note && (v.keyDown || v.sustained)) return i;
  }
  for (int i = 0; i < kNumVoices; ++i)
    if (!voices_[i].active) return i;
  // then the quietest voice that is already dying away
  int best = -1;
  float bestLevel = 2.0f;
  for (int i = 0; i < kNumVoices; ++i) {
    const Voice& v = voices_[i];
    if (!v.gate && v.aenv.level < bestLevel) {
      bestLevel = v.aenv.level;
      best = i;
    }
  }
  if (best >= 0) return best;
  // all eight are held: the oldest note gives way
  best = 0;
  for (int i = 1; i < kNumVoices; ++i)
    if (voices_[i].age < voices_[best].age) best = i;
  return best;
}

void Synth::noteOn(uint8_t note, uint8_t velocity) {
  float x = (float)velocity * (1.0f / 127.0f);
  float vel = x * x;

  if (gParams.step(P_LFO_SYNC) != 0) {  // key sync: the LFO starts over when a note starts from silence
    bool any = false;
    for (int i = 0; i < kNumVoices; ++i) any = any || voices_[i].active;
    if (!any) lfoPhase_ = 0.0f;
  }

  if (mode_ == MODE_POLY) {
    trigger(voices_[pickVoice(note)], note, vel);
    lastPitch_ = (float)note;
    return;
  }

  // mono and unison: last note wins, held notes are remembered
  bool wasHeld = stackN_ > 0;
  dropNote(stackNote_, stackVel_, stackN_, note);
  if (stackN_ == 16) dropNote(stackNote_, stackVel_, stackN_, stackNote_[0]);
  stackNote_[stackN_] = note;
  stackVel_[stackN_] = velocity;
  ++stackN_;

  int nv = mode_ == MODE_UNI ? kNumVoices : 1;
  for (int i = 0; i < nv; ++i) {
    Voice& v = voices_[i];
    if (wasHeld && v.active && v.gate) {  // legato: slide without restarting the envelopes
      v.note = note;
      v.target = (float)note;
      v.keyDown = true;
      v.sustained = false;
    } else {
      trigger(v, note, vel);
    }
  }
  lastPitch_ = (float)note;
}

void Synth::noteOff(uint8_t note) {
  if (mode_ == MODE_POLY) {
    for (int i = 0; i < kNumVoices; ++i) {
      Voice& v = voices_[i];
      if (v.active && v.note == note && v.keyDown) {
        v.keyDown = false;
        if (sustain_) v.sustained = true;
        else releaseVoice(v);
      }
    }
    return;
  }

  dropNote(stackNote_, stackVel_, stackN_, note);
  int nv = mode_ == MODE_UNI ? kNumVoices : 1;
  if (stackN_ > 0) {  // fall back to the previous held note
    uint8_t top = stackNote_[stackN_ - 1];
    for (int i = 0; i < nv; ++i) {
      Voice& v = voices_[i];
      if (v.active) {
        v.note = top;
        v.target = (float)top;
      }
    }
    lastPitch_ = (float)top;
  } else {
    for (int i = 0; i < nv; ++i) {
      Voice& v = voices_[i];
      if (v.active && v.keyDown) {
        v.keyDown = false;
        if (sustain_) v.sustained = true;
        else releaseVoice(v);
      }
    }
  }
}

void Synth::setSustain(bool down) {
  sustain_ = down;
  if (down) return;
  for (int i = 0; i < kNumVoices; ++i)
    if (voices_[i].sustained) releaseVoice(voices_[i]);
}

void Synth::releaseAll() {
  for (int i = 0; i < kNumVoices; ++i)
    if (voices_[i].active) releaseVoice(voices_[i]);
  stackN_ = 0;
}

void Synth::killAll() {
  for (int i = 0; i < kNumVoices; ++i) {
    Voice& v = voices_[i];
    v.active = v.gate = v.keyDown = v.sustained = false;
    v.aenv = Env{0.0f, ENV_IDLE};
    v.fenv = Env{0.0f, ENV_IDLE};
    v.f1 = Svf{0.0f, 0.0f};
    v.f2 = Svf{0.0f, 0.0f};
  }
  stackN_ = 0;
  physN_ = setN_ = 0;
  arpNote_ = -1;
  arpRestart_ = true;
  sustain_ = false;
  pedal_ = false;
}

// ---------------------------------------------------------------- per-block settings

void Synth::derive(Patch& p, int len) {
  for (int i = 0; i < P_COUNT; ++i)
    if (kParams[i].kind == PK_POT) smooth_[i] += (gParams.norm(i) - smooth_[i]) * smoothCoef_;
  const float* sm = smooth_;
  float blockSec = (float)len / sr_;

  p.wave1 = (uint8_t)gParams.step(P_OSC1_WAVE);
  p.wave2 = (uint8_t)gParams.step(P_OSC2_WAVE);
  p.eng1 = (uint8_t)gParams.step(P_OSC1_ENGINE);
  p.eng2 = (uint8_t)gParams.step(P_OSC2_ENGINE);
  p.oct1 = (float)(gParams.step(P_OSC1_OCTAVE) - 2) * 12.0f;
  p.oct2 = (float)(gParams.step(P_OSC2_OCTAVE) - 2) * 12.0f;
  p.detune2 = mapDetuneCents(sm[P_OSC2_DETUNE]) * 0.01f;
  p.macro1 = sm[P_OSC1_MACRO];
  p.macro2 = gParams.frac(P_OSC2_MACRO);
  p.color1 = gParams.frac(P_OSC1_COLOR);
  p.color2 = gParams.frac(P_OSC2_COLOR);

  // ---- LFO: one for all voices, free running ----
  lfoPhase_ += mapLfoHz(sm[P_LFO_RATE]) * blockSec;
  if (lfoPhase_ >= 1.0f) {
    lfoPhase_ -= floorf_(lfoPhase_);
    lfoRandom_ = nextNoise(lfoRng_);
  }
  float lfo;
  switch (gParams.step(P_LFO_SHAPE)) {
    case LFOS_SINE: lfo = tabRead(gSine, lfoPhase_); break;
    case LFOS_TRI: lfo = 1.0f - 4.0f * absf(lfoPhase_ - 0.5f); break;
    case LFOS_SAW: lfo = 1.0f - 2.0f * lfoPhase_; break;
    case LFOS_SQUARE: lfo = lfoPhase_ < 0.5f ? 1.0f : -1.0f; break;
    default: lfo = lfoRandom_; break;
  }
  lfoValue_ = lfo;
  float depth = clampf(sm[P_LFO_DEPTH] + modWheel_, 0.0f, 1.0f);  // mod wheel adds depth
  p.lfoPitch = 0.0f;
  p.lfoCut = 0.0f;
  p.lfoPw = 0.0f;
  p.tremGain = 1.0f;
  switch (gParams.step(P_LFO_DEST)) {
    case LFO_PITCH: p.lfoPitch = lfo * depth * depth * 12.0f; break;
    case LFO_CUTOFF: p.lfoCut = lfo * depth * 4.0f; break;
    case LFO_AMP: p.tremGain = 1.0f - depth * (0.5f - 0.5f * lfo); break;
    default: p.lfoPw = lfo * depth * 0.4f; break;
  }
  float fade = mapFadeTime(gParams.frac(P_LFO_FADE));
  p.lfoFadeStep = fade <= 0.0f ? 1.0f : blockSec / fade;

  p.mix[0] = sm[P_MIX_OSC1];
  p.mix[1] = sm[P_MIX_OSC2];
  p.mix[2] = sm[P_MIX_SUB];
  p.mix[3] = sm[P_MIX_NOISE];

  // ---- filter ----
  p.ftype = (uint8_t)gParams.step(P_FILT_TYPE);
  p.slope24 = gParams.step(P_FILT_SLOPE) == SLOPE_24;
  p.ladder = gParams.step(P_FILT_SLOPE) == SLOPE_LADDER;
  p.ladderK = mapLadderFeedback(sm[P_RESONANCE]);
  p.ladderComp = 1.0f + 0.5f * p.ladderK;  // a ladder loses bass as resonance rises; give half of it back
  p.cutoffOct = sm[P_CUTOFF] * 10.0f;
  float q = mapResonanceQ(sm[P_RESONANCE]);
  p.k = 1.0f / q;
  p.comp = q > 1.0f ? 1.0f / sqrtf_(q) : 1.0f;  // keeps high resonance from getting far louder
  p.envAmt = mapEnvAmtOct(sm[P_FILT_ENV_AMT]);
  p.keytrack = sm[P_KEYTRACK];
  p.velCut = gParams.frac(P_VEL_CUTOFF) * 4.0f;
  float dn = sm[P_DRIVE];
  p.driveOn = dn > 0.002f;
  p.drive = mapDrive(dn);

  // ---- voice ----
  float glide = mapGlideTime(sm[P_GLIDE]);
  p.glideCoef = glide <= 0.0f ? 1.0f : 1.0f - exp2f_(-1.4427f * 3.0f * blockSec / glide);
  bendSmooth_ += (bend_ - bendSmooth_) * 0.25f;
  p.bendSemis = bendSmooth_ * (float)(gParams.step(P_BEND_RANGE) + kParams[P_BEND_RANGE].intMin);
  p.uniDetune = (float)gParams.step(P_UNI_DETUNE) * 0.01f;
  p.subDiv = gParams.step(P_SUB_OCTAVE) == 0 ? 0.5f : 0.25f;
  p.fenvPitch = (float)gParams.step(P_FENV_PITCH);

  // ---- envelopes: per-sample one-pole coefficients ----
  float invSr = 1.0f / sr_;
  p.aenv.a = exp2f_(-2.1155f * invSr / mapEnvTime(sm[P_AENV_A]));
  p.aenv.d = exp2f_(-9.966f * invSr / mapEnvTime(sm[P_AENV_D]));
  p.aenv.s = sm[P_AENV_S];
  p.aenv.r = exp2f_(-9.966f * invSr / mapEnvTime(sm[P_AENV_R]));
  p.fenv.a = exp2f_(-2.1155f * invSr / mapEnvTime(sm[P_FENV_A]));
  p.fenv.d = exp2f_(-9.966f * invSr / mapEnvTime(sm[P_FENV_D]));
  p.fenv.s = sm[P_FENV_S];
  p.fenv.r = exp2f_(-9.966f * invSr / mapEnvTime(sm[P_FENV_R]));
  p.velAmt = sm[P_VELOCITY];

  p.volume = mapVolume(sm[P_VOLUME]) * 0.35f;  // 0.35 leaves headroom for 8 voices

  p.drift = sm[P_DRIFT];
  p.driftCoef = 1.0f - exp2f_(-1.4427f * kTwoPi * 0.8f * blockSec);  // wanders at under 1 Hz
  p.spread = sm[P_SPREAD];
  p.xmod = sm[P_XMOD];
  p.sync = gParams.step(P_SYNC) != 0;

  // ---- effects ----
  p.fx.chorus = gParams.frac(P_CHORUS);
  p.fx.dlyTime = mapDelayTime(gParams.frac(P_DLY_TIME));
  p.fx.dlyTone = gParams.frac(P_DLY_TONE);
  p.fx.dlyFdbk = gParams.frac(P_DLY_FDBK);
  p.fx.dlyMix = gParams.frac(P_DLY_MIX);
  p.fx.rvbTime = mapReverbTime(gParams.frac(P_RVB_SIZE));
  p.fx.rvbTone = gParams.frac(P_RVB_TONE);
  p.fx.rvbMix = gParams.frac(P_RVB_MIX);
}

// Sets one oscillator up for a block: picks tables, works out the engine's numbers.
void Synth::prepOsc(Osc& o, OscRun& r, uint8_t& lastEngine, uint8_t& lastWave, uint8_t engine, uint8_t wave,
                    float inc, float macro, float color, float pw) {
  bool sameSound = lastEngine == engine && lastWave == wave;
  r.engine = engine;
  r.wave = wave;
  r.inc = inc;
  r.pw = pw;
  switch (engine) {
    case ENG_VA:
      if (wave == WAVE_SUPER) {
        // Macro is the detune. Never quite zero, so it always sounds like more than one saw.
        float amount = 0.05f + 0.95f * macro * macro;
        for (int k = 0; k < 6; ++k) r.ssInc[k] = minf(inc * (1.0f + kSuperDetune[k] * amount), 0.45f);
        setTable(o, sawTableFor(minf(inc * 1.11f, 0.45f)), sameSound);  // clean for the sharpest of the seven
      } else {
        setTable(o, wave == WAVE_TRI ? triTableFor(inc) : sawTableFor(inc), sameSound);
      }
      break;
    case ENG_FM: {
      r.fmRatio = kFmRatios[wave < WAVE_COUNT ? wave : 0];
      // More modulation means higher partials. Hold it back as the note
      // rises so that none of any strength reach half the sample rate.
      // `orders` is how many steps of the modulator's frequency fit between
      // the note and that limit. Feedback roughens the modulator, which
      // spreads the partials further, so it is only let in where there is
      // plenty of room, and it shrinks the room left for the amount.
      float orders = (0.5f / inc - 1.0f) / r.fmRatio;
      float feedback = color * 1.25f * clampf((orders - 12.0f) * (1.0f / 24.0f), 0.0f, 1.0f);  // radians
      float room = (orders / (1.0f + 2.0f * feedback) - 3.5f) * (1.0f / 1.6f);
      float index = minf(macro * macro * 8.0f, maxf(room, 0.0f));
      r.fmIndex = index * (1.0f / kTwoPi);
      r.fmFeedback = feedback * (1.0f / kTwoPi);
      break;
    }
    case ENG_WAVETABLE: {
      int lvl = wtLevelFor(inc);
      float pos = macro * (float)(kWtFrames - 1);
      int f0 = pos >= (float)(kWtFrames - 1) ? kWtFrames - 2 : (int)pos;
      int table = wave < kWtTables ? wave : 0;
      r.wtA = gWt[table][f0][lvl];
      r.wtB = gWt[table][f0 + 1][lvl];
      r.wtMix = pos - (float)f0;
      break;
    }
    default: {  // pluck
      r.ksDamp = 1.0f - 0.8f * macro;  // Macro is damping: higher is duller
      // the loop filter delays the wave a little; shorten the string to stay in tune
      r.ksDelay = clampf(1.0f / inc - (1.0f - r.ksDamp) / r.ksDamp, 2.0f, (float)(kKsLen - 2));
      r.ksGain = 0.985f + 0.0148f * color;  // Color is how long the string rings
      if (o.pluck || !sameSound) {
        for (int i = 0; i < kKsLen; ++i) o.ks[i] = 0;
        o.lp = 0.0f;
        o.fb = 0.0f;
        o.exciteLen = clampi((int)(1.0f / inc), 2, kKsLen - 2);
        o.excite = o.exciteLen;
      }
      break;
    }
  }
  o.pluck = false;
  lastEngine = engine;
  lastWave = wave;
}

void Synth::renderVoice(Voice& v, int index, const Patch& p, float* outL, float* outR, int n) {
  // ---- modulation that belongs to this voice ----
  v.lfoAmt = minf(v.lfoAmt + p.lfoFadeStep, 1.0f);
  float lfoPw = p.lfoPw * v.lfoAmt;

  // Drift: each voice is a little out of tune with the others, by a fixed
  // amount plus a slow wander, as the voices of an analog poly are.
  float drift1 = 0.0f, drift2 = 0.0f, driftCut = 0.0f;
  if (p.drift > 0.0f) {
    v.drift1 += (nextNoise(v.driftRng) - v.drift1) * p.driftCoef;
    v.drift2 += (nextNoise(v.driftRng) - v.drift2) * p.driftCoef;
    v.driftCut += (nextNoise(v.driftRng) - v.driftCut) * p.driftCoef;
    drift1 = p.drift * (0.04f * kVoiceTune[index] + 3.0f * v.drift1);   // semitones
    drift2 = p.drift * (-0.04f * kVoiceTune[index] + 3.0f * v.drift2);
    driftCut = p.drift * (0.15f * kVoiceCut[index] + 8.0f * v.driftCut);  // octaves
  }

  // ---- pitch ----
  v.pitch += (v.target - v.pitch) * p.glideCoef;
  float detune = mode_ == MODE_UNI ? p.uniDetune * ((float)index * (2.0f / (float)(kNumVoices - 1)) - 1.0f) : 0.0f;
  float base = v.pitch + p.bendSemis + p.lfoPitch * v.lfoAmt + detune - 69.0f;
  // Env>Pitch moves both oscillators. With Sync on it moves only oscillator
  // 2, the one being reset, which is what makes the classic sync sweep.
  float sweep = p.fenvPitch * v.fenv.level;
  float base1 = base + drift1 + (p.sync ? 0.0f : sweep);
  float base2 = base + drift2 + sweep;
  float invSr = 1.0f / sr_;
  float inc1 = minf(440.0f * exp2f_((base1 + p.oct1) * (1.0f / 12.0f)) * invSr, 0.45f);
  float inc2 = minf(440.0f * exp2f_((base2 + p.oct2 + p.detune2) * (1.0f / 12.0f)) * invSr, 0.45f);
  float incSub = inc1 * p.subDiv;

  OscRun r1, r2;
  // Pulse width for the VA engine; on the others the LFO's "PW" destination sweeps Macro instead.
  float pw1 = clampf(mapPulseWidth(p.macro1) + lfoPw, 0.05f, 0.95f);
  float pw2 = clampf(mapPulseWidth(p.macro2) + lfoPw, 0.05f, 0.95f);
  prepOsc(v.o1, r1, v.eng1, v.wave1, p.eng1, p.wave1, inc1, clampf(p.macro1 + lfoPw * 1.2f, 0.0f, 1.0f), p.color1, pw1);
  prepOsc(v.o2, r2, v.eng2, v.wave2, p.eng2, p.wave2, inc2, clampf(p.macro2 + lfoPw * 1.2f, 0.0f, 1.0f), p.color2, pw2);
  setTable(v.sub, sawTableFor(incSub), true);

  // Cross-mod: oscillator 2's wave bends oscillator 1's phase. Like FM, it
  // adds partials above the note, so it is held back as the notes get high.
  float room = clampf((0.5f / maxf(inc1, inc2) - 6.0f) * (1.0f / 30.0f), 0.0f, 1.0f);
  float xmod = p.xmod * p.xmod * 0.6f * room;  // cycles
  bool sync = p.sync && p.eng2 != ENG_PLUCK;   // a string has no cycle to restart

  // ---- filter coefficients for this block ----
  float oct = p.cutoffOct + p.lfoCut * v.lfoAmt + p.envAmt * v.fenv.level + p.velCut * v.vel + driftCut +
              p.keytrack * (v.pitch - 60.0f) * (1.0f / 12.0f);
  float fc = clampf(20.0f * exp2f_(oct), 20.0f, 0.45f * sr_);
  float g = tanf_(kPi * fc * invSr);
  SvfCoef c2 = svfCoef(g, p.k);
  SvfCoef c1 = svfCoef(g, 1.41421356f);  // first stage of the 24 dB slope: no peak of its own
  // Ladder: four simple low-pass stages in a row with feedback round all
  // four, solved together so it stays in tune and stable at any cutoff.
  float lG = g / (1.0f + g), lA = 1.0f - lG;
  float lG2 = lG * lG, lG3 = lG2 * lG;
  float lDen = 1.0f / (1.0f + p.ladderK * lG2 * lG2);

  float velGain = 1.0f - p.velAmt * (1.0f - v.vel);
  float trem = 1.0f - (1.0f - p.tremGain) * v.lfoAmt;
  float amp = velGain * trem * (mode_ == MODE_UNI ? 0.45f : 1.0f);

  // Spread: where this voice sits between the speakers. A lone mono voice stays in the middle.
  float pan = mode_ == MODE_MONO ? 0.0f : p.spread * kVoicePan[index];
  float gainL = amp * (pan > 0.0f ? 1.0f - pan : 1.0f);
  float gainR = amp * (pan < 0.0f ? 1.0f + pan : 1.0f);

  bool on1 = p.mix[0] > 1e-4f, on2 = p.mix[1] > 1e-4f, onSub = p.mix[2] > 1e-4f, onNoise = p.mix[3] > 1e-4f;
  bool need2 = on2 || xmod > 0.0f;  // oscillator 2 is also needed, unheard, to bend oscillator 1
  float m1 = p.mix[0] * 0.5f, m2 = p.mix[1] * 0.5f, m3 = p.mix[2] * 0.5f, m4 = p.mix[3] * 0.5f;

  for (int i = 0; i < n; ++i) {
    envTick(v.fenv, p.fenv);
    float ae = envTick(v.aenv, p.aenv);

    // oscillator 2 first: oscillator 1 may be bent by it, and may restart it
    float s2 = 0.0f;
    if (need2) {
      s2 = oscValue(v.o2, r2, 0.0f, v.rng) + v.o2.blep;
      v.o2.blep = 0.0f;
    }
    oscStep(v.o2, r2);

    float before1 = v.o1.phase;
    float s1 = on1 ? oscValue(v.o1, r1, xmod * s2, v.rng) : 0.0f;
    oscStep(v.o1, r1);

    if (sync && v.o1.phase < before1) {
      // Oscillator 1 began a new cycle during this sample, `t` of a sample
      // ago. Oscillator 2 restarts from that same instant. The jump in its
      // wave is smoothed over this sample and the next, or it would ring
      // with false tones.
      float t = clampf(v.o1.phase / r1.inc, 0.0f, 1.0f);
      float was = need2 ? oscValue(v.o2, r2, 0.0f, v.rng) : 0.0f;
      v.o2.phase = wrap01(t * r2.inc);
      if (r2.engine == ENG_FM) v.o2.phase2 = wrap01(t * r2.inc * r2.fmRatio);
      if (need2) {
        float jump = oscValue(v.o2, r2, 0.0f, v.rng) - was;
        s2 += jump * t * t * 0.5f;
        v.o2.blep = -jump * (1.0f - t) * (1.0f - t) * 0.5f;
      }
    }

    float x = s1 * m1;
    if (on2) x += s2 * m2;
    if (onSub) x += pulseAt(v.sub, v.sub.phase, 0.5f) * m3;
    advance(v.sub, incSub);
    if (onNoise) x += nextNoise(v.rng) * m4;

    if (p.driveOn) x += p.drive.mix * (tanhf_(x * p.drive.pre) * p.drive.preTrim - x);
    if (p.ladder) {
      float& z1 = v.f1.ic1;
      float& z2 = v.f1.ic2;
      float& z3 = v.f2.ic1;
      float& z4 = v.f2.ic2;
      float fed = lA * (lG3 * z1 + lG2 * z2 + lG * z3 + z4);  // what the four stages are about to hand back
      // Saturating here keeps a whistle from running away. The limit is set
      // high (at 2, not 1) so ordinary levels pass almost untouched: pushed
      // hard, the saturator also pulls the resonance flat, as analog ladders do.
      float u = 2.0f * tanhf_(0.5f * (x * p.ladderComp - p.ladderK * fed) * lDen);
      float y = u;
      float t = (y - z1) * lG; y = t + z1; z1 = y + t;
      t = (y - z2) * lG; y = t + z2; z2 = y + t;
      t = (y - z3) * lG; y = t + z3; z3 = y + t;
      t = (y - z4) * lG; y = t + z4; z4 = y + t;
      x = y;
    } else {
      x *= p.comp;
      if (p.slope24) x = svfTick(v.f1, x, c1, p.ftype);
      x = svfTick(v.f2, x, c2, p.ftype);
    }
    if (p.driveOn) x += p.drive.mix * (tanhf_(x * p.drive.post) * p.drive.postTrim - x);  // after the filter: these harmonics stay

    float out = x * ae;
    outL[i] += out * gainL;
    outR[i] += out * gainR;
  }

  if (v.aenv.stage == ENV_IDLE) {
    v.active = false;
    v.gate = false;
    v.fenv = Env{0.0f, ENV_IDLE};
  }
}

void Synth::process(float* outL, float* outR, int n) {
  while (qRead_ != qWrite_) {
    Event e = queue_[qRead_];
    qRead_ = (qRead_ + 1) % kQueueLen;
    handleEvent(e.type, e.a, e.b);
  }

  // ---- switches that change how notes are handled ----
  bool hold = gParams.step(P_HOLD) != 0;
  if (hold != hold_) {
    hold_ = hold;
    if (!latching()) dropUnheld();
  }
  uint8_t arp = (uint8_t)gParams.step(P_ARP_MODE);
  if ((arp != ARP_OFF) != (arpMode_ != ARP_OFF)) {
    releaseAll();  // hand the held notes from the keys to the arpeggiator, or back
    arpNote_ = -1;
    arpRestart_ = true;
    arpMode_ = arp;
    if (arp == ARP_OFF)
      for (int i = 0; i < setN_; ++i) soundKey(i);
  }
  arpMode_ = arp;
  uint8_t mode = (uint8_t)gParams.step(P_VOICE_MODE);
  if (mode != mode_) {  // switching voice mode lets go of everything, then picks the held notes up again
    releaseAll();
    mode_ = mode;
    arpNote_ = -1;
    if (arpMode_ == ARP_OFF)
      for (int i = 0; i < setN_; ++i) soundKey(i);
  }

  int done = 0;
  int active = 0;
  float busL[kCtrlBlock], busR[kCtrlBlock];
  while (done < n) {
    int len = n - done < kCtrlBlock ? n - done : kCtrlBlock;
    for (int i = 0; i < len; ++i) busL[i] = busR[i] = 0.0f;

    Patch p;
    derive(p, len);
    arpAdvance(len);

    active = 0;
    for (int i = 0; i < kNumVoices; ++i) {
      if (!voices_[i].active) continue;
      renderVoice(voices_[i], i, p, busL, busR, len);
      ++active;
    }

    for (int i = 0; i < len; ++i) {
      busL[i] *= p.volume;
      busR[i] *= p.volume;
    }
    float* l = outL + done;
    float* r = outR + done;
    gFx.process(busL, busR, l, r, len, p.fx);

    float peak = peak_ * peakDecay_;
    uint32_t pos = scopePos_;
    for (int i = 0; i < len; ++i) {
      l[i] = softClip(l[i]);
      r[i] = softClip(r[i]);
      float x = 0.5f * (l[i] + r[i]);
      scope_[pos & (kScopeLen - 1)] = x;
      ++pos;
      float a = maxf(absf(l[i]), absf(r[i]));
      if (a > peak) peak = a;
    }
    scopePos_ = pos;
    peak_ = peak < 1e-6f ? 0.0f : peak;
    done += len;
  }
  activeCount_ = active;
}

}  // namespace fw
