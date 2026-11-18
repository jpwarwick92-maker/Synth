#include "synth.h"

#include "fw_math.h"
#include "fx.h"
#include "midi.h"
#include "tables.h"
#include "vocoder.h"

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
// With one LFO per voice, each runs a few percent faster or slower than the
// knob says, so they never fall into step with each other.
const float kVoiceLfoRate[kNumVoices] = {1.0f, 0.94f, 1.07f, 0.97f, 1.04f, 0.91f, 1.1f, 0.985f};
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
  // The LFO. `lfo` is the shared one's value now (-1..1); with one LFO per
  // voice each voice works out its own from lfoStep and lfoShape. The four
  // amounts say how far a full swing moves each destination.
  float lfo;
  bool lfoPerVoice;
  uint8_t lfoShape;
  float lfoStep;         // cycles per control block
  float lfoPitchAmt;     // semitones
  float lfoCutAmt;       // octaves
  float lfoPwAmt;        // added to pulse width, or to Macro on the other engines
  float lfoTremAmt;      // 0..1: how far the level is pulled down at the bottom of the swing
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
  float bendSemis;
  float uniDetune;       // semitones, half the total spread
  float subRatio;        // sub frequency as a multiple of oscillator 1's
  float volume;
  float drift, driftCoef;  // 0..1, and how fast the wander moves per block
  float spread;            // 0..1
  float xmod;              // 0..1: oscillator 2 bending oscillator 1
  bool sync;               // oscillator 2 restarts with each cycle of oscillator 1
  FxSettings fx;
  VocSettings voc;
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
  uint32_t ssInc[6];  // supersaw: the six side saws' speeds, as 32-bit fractions of a cycle per sample
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

// The LFO's five shapes, -1..1, at `phase` (0..1) through its cycle.
// `random` is the value the random shape is holding for this cycle.
inline float lfoWave(uint8_t shape, float phase, float random) {
  switch (shape) {
    case LFOS_SINE: return tabRead(gSine, phase);
    case LFOS_TRI: return 1.0f - 4.0f * absf(phase - 0.5f);
    case LFOS_SAW: return 1.0f - 2.0f * phase;
    case LFOS_SQUARE: return phase < 0.5f ? 1.0f : -1.0f;
    default: return random;
  }
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
          // The six side saws, read with integer arithmetic throughout:
          // the phase is a 32-bit fraction of a cycle, its top 11 bits pick
          // the table entry and the next 15 the place between two entries.
          uint32_t shift = pm == 0.0f ? 0u : (uint32_t)(wrap01(pm) * 4294967296.0f);  // wrap01 < 1, so this fits
          const int16_t* t = o.tab;
          int32_t acc = 0;
          for (int k = 0; k < 6; ++k) {
            uint32_t at = o.ss[k] + shift;
            uint32_t idx = at >> 21;  // kTabLen is 2^11
            int32_t f = (int32_t)((at >> 6) & 0x7FFFu);
            int32_t a = t[idx];
            acc += a + (((t[idx + 1] - a) * f) >> 15);
          }
          float sides = (float)acc * (1.0f / kTabScale);
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
      i0 &= kKsLen - 1;  // rp can round up to exactly the length: that is slot 0, not the memory after the string
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
    for (int k = 0; k < 6; ++k) o.ss[k] += r.ssInc[k];
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

// The audio input as the converter gives it: never outside -1..1, never not a number.
inline float cleanIn(float x) {
  if (!(x == x)) return 0.0f;
  return x > 1.0f ? 1.0f : x < -1.0f ? -1.0f : x;
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
  arpSteps_ = 0;
  arpBeatStep_ = 0;
  sustain_ = false;
  bend_ = bendSmooth_ = 0.0f;
  modWheel_ = 0.0f;
  lfoPhase_ = lfoValue_ = lfoRandom_ = 0.0f;
  beatPhase_ = 0.0f;
  beatReset_ = false;
  lfoRng_ = 0x1234567u;
  clkSource_ = (uint8_t)setting(SET_CLOCK);
  sampleNow_ = 0;
  clkOutPhase_ = 0.0f;
  clkState_ = clkSource_ == CLOCK_INTERNAL ? CLK_INTERNAL : CLK_WAITING;
  tempoBpm_ = mapTempoBpm(gParams.norm(P_ARP_TEMPO));
  beatShown_ = 0.0f;
  clockReset();
  inGainMix_ = inGainFx_ = 0.0f;
  inCoef_ = 1.0f - exp2f_(-1.4427f / (0.005f * sr_));  // the input fades in and out over a few milliseconds
  inPeak_ = 0.0f;
  inClip_ = 0.0f;
  qWrite_ = qRead_ = 0;
  overflow_ = false;
  load_ = 0.0f;
  voiceLimit_ = kNumVoices;
  overBlocks_ = 0;
  calmSamples_ = 0.0f;
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
    v.fresh = v.letGo = v.choke = false;
    v.note = 60;
    v.vel = 0.0f;
    v.age = 0;
    v.pitch = v.target = 60.0f;
    v.o1 = Osc{(float)i * 0.123f, rest, rest, 0.0f, 0.0f, 0.0f, 0.0f, gStrings[i][0], 0, 0, 1, false, {}, 0.0f};
    v.o2 = Osc{(float)i * 0.071f + 0.3f, rest, rest, 0.0f, 0.0f, 0.0f, 0.0f, gStrings[i][1], 0, 0, 1, false, {}, 0.0f};
    v.sub = Osc{0.0f, rest, rest, 0.0f, 0.0f, 0.0f, 0.0f, nullptr, 0, 0, 1, false, {}, 0.0f};
    for (int k = 0; k < 6; ++k) {  // the supersaw's saws start scattered, so they never line up into one
      v.o1.ss[k] = (uint32_t)(wrap01(0.137f * (float)(k + 1) + 0.071f * (float)i) * 4294967296.0f);
      v.o2.ss[k] = (uint32_t)(wrap01(0.211f * (float)(k + 1) + 0.053f * (float)i + 0.4f) * 4294967296.0f);
    }
    v.drift1 = v.drift2 = v.driftCut = 0.0f;
    v.driftRng = 0x6C8E9CF5u * (uint32_t)(i + 3) | 1u;
    v.wave1 = v.wave2 = WAVE_SAW;
    v.eng1 = v.eng2 = ENG_VA;
    v.lfoAmt = 1.0f;
    v.lfoPhase = (float)i * (1.0f / (float)kNumVoices);  // spread round the cycle from the start
    v.lfoRandom = 0.0f;
    v.f1 = Svf{0.0f, 0.0f};
    v.f2 = Svf{0.0f, 0.0f};
    v.aenv = Env{0.0f, ENV_IDLE};
    v.fenv = Env{0.0f, ENV_IDLE};
    v.rng = 0x9E3779B9u * (uint32_t)(i + 1) | 1u;
    for (int o = 0; o < 2; ++o)
      for (int k = 0; k < kKsLen; ++k) gStrings[i][o][k] = 0;
  }
  gFx.init(sampleRate);
  gVocoder.init(sampleRate);
}

bool Synth::post(uint8_t type, uint8_t a, uint8_t b) {
  uint32_t w = qWrite_;
  uint32_t next = (w + 1) % kQueueLen;
  if (next == qRead_) {
    overflow_ = true;
    return false;
  }
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
    case EV_BEND: setBend((float)(((int)a | ((int)b << 7)) - 8192) * (1.0f / 8192.0f)); break;
    case EV_MOD_WHEEL: setModWheel((float)a * (1.0f / 127.0f)); break;
    case EV_ALL_NOTES_OFF:
      releaseAll();
      physN_ = setN_ = 0;
      arpNote_ = -1;
      break;
    case EV_ALL_SOUND_OFF: killAll(); break;
    case EV_CHORD: toggleChord(); break;
    case EV_CLOCK: case EV_START: case EV_CONTINUE: case EV_STOP: case EV_SONG_POS: clockEvent(type, a, b); break;
    default: break;
  }
}

// ---------------------------------------------------------------- following a clock

// Nothing known about the clock yet: the first tick to arrive will be position 0.
void Synth::clockReset() {
  extLocked_ = false;
  extRun_ = true;
  extArmed_ = true;
  extTick_ = 0;
  extNext_ = 0;
  extLead_ = 0.0f;
  extStep_ = -1;
  extStepFire_ = false;
  extPeriod_ = sr_ * 60.0f / (mapTempoBpm(gParams.norm(P_ARP_TEMPO)) * 24.0f);
  extSince_ = 0.0f;
  extStampN_ = extStampPos_ = 0;
}

void Synth::clockEvent(uint8_t type, uint8_t a, uint8_t b) {
  if (clkSource_ == CLOCK_INTERNAL) return;
  switch (type) {
    case EV_START:  // the next tick is the first downbeat
      extRun_ = true;
      extArmed_ = true;
      extNext_ = 0;
      arpRestart_ = true;
      return;
    case EV_CONTINUE:  // the next tick carries on from where stop left things
      if (!extRun_) {
        extRun_ = true;
        extArmed_ = true;
      }
      return;
    case EV_STOP:
      if (extRun_) {
        extRun_ = false;
        if (!extArmed_) extNext_ = extTick_ + 1;
      }
      return;
    case EV_SONG_POS:  // the sender moved its playhead: the next tick is at this sixteenth note
      extNext_ = 6u * ((uint32_t)a | ((uint32_t)b << 7));
      extArmed_ = true;
      return;
    default: break;
  }

  // ---- a tick ----
  // Tempo: the average gap between the ticks of the last beat. Ticks are
  // only seen at the start of an audio block, so one gap alone is rough;
  // a beat's worth of them is not.
  if (!extLocked_) {
    extLocked_ = true;
    extStampN_ = 0;
  }
  bool running = extStampN_ >= 2;  // the smooth position has been moving since the last tick
  if (extStampN_ > 0) {
    int gaps = extStampN_ < kClockWindow ? extStampN_ : kClockWindow - 1;
    uint32_t oldest = extStamp_[(extStampPos_ + kClockWindow - gaps) % kClockWindow];
    float average = (float)(sampleNow_ - oldest) / (float)gaps;
    float fastest = sr_ * 60.0f / (400.0f * 24.0f);  // nothing sends 400 beats a minute: a burst of late ticks
    if (average < fastest) average = fastest;
    extPeriod_ += (average - extPeriod_) * (gaps < 6 ? 1.0f : 0.3f);
  }
  extStamp_[extStampPos_] = sampleNow_;
  extStampPos_ = (extStampPos_ + 1) % kClockWindow;
  if (extStampN_ < kClockWindow) ++extStampN_;
  extSince_ = 0.0f;

  if (!extRun_) return;  // stopped: the tempo is still measured, the position stays where it is
  if (extArmed_) {
    extArmed_ = false;
    extTick_ = extNext_;
    extLead_ = 0.0f;
    // a tick that lands exactly on a sixteenth note plays that step
    extStep_ = (int32_t)(extTick_ / 6u) - (extTick_ % 6u == 0 ? 1 : 0);
    return;
  }
  ++extTick_;
  extLead_ -= 1.0f;  // the smooth position should be exactly on this tick now
  if (!running || extLead_ > 1.5f || extLead_ < -1.5f) extLead_ = 0.0f;  // not moving yet, or far out: jump
  else extLead_ *= 0.8f;                                                 // a little out: close a fifth of the gap
}

// Once per control block: works out what is keeping time, moves the smooth
// position on, and sends the synth's own clock when it is the one in charge.
void Synth::clockAdvance(int len) {
  uint8_t source = clkSource_;
  extStepFire_ = false;
  float knobBpm = mapTempoBpm(gParams.norm(P_ARP_TEMPO));
  if (source == CLOCK_INTERNAL) {
    clkState_ = CLK_INTERNAL;
    tempoBpm_ = knobBpm;
    if (setting(SET_CLOCK_OUT)) {
      clkOutPhase_ += (float)len * knobBpm * 24.0f / (60.0f * sr_);
      while (clkOutPhase_ >= 1.0f) {
        clkOutPhase_ -= 1.0f;
        gMidi.clockTick();
      }
    } else {
      clkOutPhase_ = 0.0f;
    }
    return;
  }
  if (extLocked_) {
    extSince_ += (float)len;
    if (extSince_ > maxf(0.5f * sr_, 4.0f * extPeriod_)) {  // the clock has gone away
      extLocked_ = false;
      if (!extArmed_) {
        extArmed_ = true;  // if it comes back, it carries on counting from here
        extNext_ = extTick_ + 1;
      }
    }
  }
  if (!extRun_) {
    clkState_ = CLK_STOPPED;
    tempoBpm_ = extLocked_ ? sr_ * 2.5f / extPeriod_ : knobBpm;
    return;
  }
  if (!extLocked_) {
    clkState_ = CLK_WAITING;
    tempoBpm_ = knobBpm;
    return;
  }
  clkState_ = CLK_LOCKED;
  tempoBpm_ = sr_ * 2.5f / extPeriod_;  // 60 seconds / 24 ticks
  if (extArmed_) return;                // waiting for the tick that says where things are
  if (extStampN_ >= 2) {                // (with one tick so far the tempo is not known yet: stay on that tick)
    extLead_ += (float)len / extPeriod_;
    if (extLead_ > 12.0f) extLead_ = 12.0f;
  }
  int32_t step = (int32_t)(extTick_ / 6u) + (int32_t)floorf_(((float)(extTick_ % 6u) + extLead_) * (1.0f / 6.0f));
  if (step > extStep_) {
    extStep_ = step;
    extStepFire_ = true;
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
  if (clkSource_ != CLOCK_INTERNAL && !extRun_) {  // the clock's sender said stop: wait for it
    if (arpNote_ >= 0) {
      noteOff((uint8_t)arpNote_);
      arpNote_ = -1;
    }
    arpRestart_ = true;
    return;
  }
  if (clkSource_ != CLOCK_INTERNAL && extLocked_) {
    // Following a clock: sixteenth notes fall where its ticks put them (six ticks each).
    float stepLen = 6.0f * extPeriod_;
    bool fire = extStepFire_;
    float into = (float)(extTick_ % 6u) + extLead_;  // ticks since the step began
    into -= 6.0f * floorf_(into * (1.0f / 6.0f));
    if (arpRestart_) {
      arpIndex_ = 0;
      arpBeatStep_ = 0;
      arpRestart_ = false;
      // The first key usually comes down a little after the step it was
      // aimed at. Within a quarter of a step it still gets that step,
      // rather than waiting for the next one.
      if (!extArmed_ && into < 1.5f) fire = true;
    }
    arpClock_ = into * extPeriod_;  // kept in step, in case the clock goes away and the Tempo knob takes over
    if (arpNote_ >= 0) {
      arpGateLeft_ -= (float)len;
      if (arpGateLeft_ <= 0.0f) {
        noteOff((uint8_t)arpNote_);
        arpNote_ = -1;
      }
    }
    if (fire && !extArmed_) arpStep(stepLen);
    return;
  }
  // sixteenth notes at the set tempo
  float bpm = mapTempoBpm(gParams.norm(P_ARP_TEMPO));
  float stepLen = sr_ * 60.0f / (bpm * 4.0f);
  if (arpRestart_) {
    arpClock_ = stepLen;
    arpIndex_ = 0;
    arpBeatStep_ = 0;
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
  arpStep(stepLen);
}

// Plays the arpeggiator's next note.
void Synth::arpStep(float stepLen) {
  if (arpNote_ >= 0) noteOff((uint8_t)arpNote_);
  uint8_t vel = 100;
  arpNote_ = arpPick(vel);
  ++arpIndex_;
  ++arpSteps_;
  ++arpBeatStep_;
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
  v.fresh = true;
  v.letGo = false;
  v.choke = false;
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
  if (v.fresh) {  // nothing of this note has sounded yet: let the block play before letting go,
    v.letGo = true;  // or a note-on and note-off arriving together would be silent
    v.keyDown = false;
    v.sustained = false;
    return;
  }
  v.gate = false;
  v.keyDown = false;
  v.sustained = false;
  if (v.aenv.stage != ENV_IDLE) v.aenv.stage = ENV_RELEASE;
  if (v.fenv.stage != ENV_IDLE) v.fenv.stage = ENV_RELEASE;
}

int Synth::pickVoice(uint8_t note) {
  // the same key played again reuses its voice
  int sounding = 0;
  for (int i = 0; i < kNumVoices; ++i) {
    const Voice& v = voices_[i];
    if (v.active && v.note == note && (v.keyDown || v.sustained)) return i;
    sounding += v.active;
  }
  if (sounding < voiceLimit_)  // (fewer than eight may be allowed while the processor is overloaded)
    for (int i = 0; i < kNumVoices; ++i)
      if (!voices_[i].active) return i;
  // then the quietest voice that is already dying away
  int best = -1;
  float bestLevel = 2.0f;
  for (int i = 0; i < kNumVoices; ++i) {
    const Voice& v = voices_[i];
    if (v.active && !v.gate && v.aenv.level < bestLevel) {
      bestLevel = v.aenv.level;
      best = i;
    }
  }
  if (best >= 0) return best;
  // every voice allowed is held: the oldest note gives way
  best = -1;
  for (int i = 0; i < kNumVoices; ++i)
    if (voices_[i].active && (best < 0 || voices_[i].age < voices_[best].age)) best = i;
  return best < 0 ? 0 : best;
}

bool Synth::leadVoice(Env& filter, Env& amp, float& velocity) const {
  int lead = -1;
  for (int i = 0; i < kNumVoices; ++i)
    if (voices_[i].active && (lead < 0 || voices_[i].age > voices_[lead].age)) lead = i;
  if (lead < 0) return false;
  filter = voices_[lead].fenv;
  amp = voices_[lead].aenv;
  velocity = voices_[lead].vel;
  return true;
}

// The processor cannot keep up: drop the voice that will be missed least,
// and allow one fewer until things have been calm for a while.
void Synth::shedVoice() {
  int sounding = 0, pick = -1;
  for (int i = 0; i < kNumVoices; ++i) {
    const Voice& v = voices_[i];
    if (!v.active || v.choke) continue;
    ++sounding;
    if (pick < 0) {
      pick = i;
      continue;
    }
    const Voice& b = voices_[pick];
    bool better = v.gate != b.gate ? !v.gate : (v.gate ? v.age < b.age : v.aenv.level < b.aenv.level);
    if (better) pick = i;
  }
  if (sounding <= 2) return;  // never below two voices
  voiceLimit_ = sounding - 1;
  voices_[pick].choke = true;
}

void Synth::noteOn(uint8_t note, uint8_t velocity) {
  float x = (float)velocity * (1.0f / 127.0f);
  float vel = x * x;

  bool any = false;
  for (int i = 0; i < kNumVoices; ++i) any = any || voices_[i].active;
  if (!any) {  // a note starting from silence
    if (gParams.step(P_LFO_SYNC) == LFOSYNC_KEY) lfoPhase_ = 0.0f;  // key sync: the LFO starts over
    beatReset_ = true;                                              // and the duck's beat starts with it
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

  int nv = mode_ == MODE_UNI ? voiceLimit_ : 1;  // (all eight, unless the processor is overloaded)
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
    v.fresh = v.letGo = v.choke = false;
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
  gFx.init(sr_);  // all sound off means the echoes and the reverb tail as well
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
  p.lfoShape = (uint8_t)gParams.step(P_LFO_SHAPE);
  p.lfoStep = mapLfoHz(sm[P_LFO_RATE]) * blockSec;
  p.lfoPerVoice = gParams.step(P_LFO_SYNC) == LFOSYNC_VOICE;
  p.lfo = lfoWave(p.lfoShape, lfoPhase_, lfoRandom_);
  lfoValue_ = p.lfo;
  float depth = clampf(sm[P_LFO_DEPTH] + modWheel_, 0.0f, 1.0f);  // mod wheel adds depth
  p.lfoPitchAmt = p.lfoCutAmt = p.lfoPwAmt = p.lfoTremAmt = 0.0f;
  switch (gParams.step(P_LFO_DEST)) {
    case LFO_PITCH: p.lfoPitchAmt = depth * depth * 12.0f; break;
    case LFO_CUTOFF: p.lfoCutAmt = depth * 4.0f; break;
    case LFO_AMP: p.lfoTremAmt = depth; break;
    default: p.lfoPwAmt = depth * 0.4f; break;
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
  p.subRatio = mapSubRatio(gParams.step(P_SUB_OCTAVE), (float)(gParams.step(P_SUB_DETUNE) + kParams[P_SUB_DETUNE].intMin));
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

  // ---- vocoder ----
  p.voc.amount = gParams.frac(P_VOCODER);
  p.voc.formant = (float)(gParams.step(P_VOC_FORMANT) + kParams[P_VOC_FORMANT].intMin);
  p.voc.consonants = gParams.frac(P_VOC_CONSONANTS);
  p.voc.smooth = gParams.frac(P_VOC_SMOOTH);
  p.voc.playing = false;  // filled in once the voices have been counted

  // ---- effects ----
  p.fx.chorus = sm[P_CHORUS];
  p.fx.ensemble = gParams.step(P_CHORUS_TYPE) == CHORUS_ENSEMBLE;
  p.fx.compress = gParams.frac(P_COMPRESS);
  p.fx.duck = gParams.frac(P_DUCK);
  // The duck keeps time with the arp's Tempo, one dip to a beat. While the
  // arp runs, the beat is counted from its steps (four to a beat), so the
  // two can never slip apart; otherwise it runs free from the last note
  // that started out of silence.
  float bpm = mapTempoBpm(sm[P_ARP_TEMPO]);
  p.fx.beatSec = 60.0f / bpm;
  p.fx.beatStep = 1.0f / (p.fx.beatSec * sr_);
  bool following = clkState_ == CLK_LOCKED && !extArmed_;
  if (following) {  // a clock from outside sets the beat, for the duck as for the arp
    p.fx.beatSec = 24.0f * extPeriod_ / sr_;
    p.fx.beatStep = 1.0f / (24.0f * extPeriod_);
    beatReset_ = false;
    float beats = ((float)(extTick_ % 24u) + extLead_) * (1.0f / 24.0f);
    beatPhase_ = beats - floorf_(beats);
    if (beatPhase_ >= 1.0f) beatPhase_ = 0.0f;
  }
  if (beatReset_) {
    beatReset_ = false;
    beatPhase_ = 0.0f;
  }
  if (arpMode_ != ARP_OFF && !following) {
    float stepLen = sr_ * 15.0f / bpm;
    beatPhase_ = ((float)((arpBeatStep_ + 3u) & 3u) + clampf(arpClock_ / stepLen, 0.0f, 1.0f)) * 0.25f;
  }
  p.fx.beatPhase = beatPhase_;
  beatShown_ = beatPhase_;
  if (arpMode_ == ARP_OFF && !following) {
    beatPhase_ += p.fx.beatStep * (float)len;
    if (beatPhase_ >= 1.0f) beatPhase_ -= floorf_(beatPhase_);
  }
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
        for (int k = 0; k < 6; ++k)
          r.ssInc[k] = (uint32_t)(minf(inc * (1.0f + kSuperDetune[k] * amount), 0.45f) * 4294967296.0f);
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
      // A string too long for its buffer plays an octave up (or two) rather than a wrong note.
      float period = 1.0f / inc;
      while (period > (float)(kKsLen - 4)) period *= 0.5f;
      r.ksDelay = clampf(period - (1.0f - r.ksDamp) / r.ksDamp, 2.0f, (float)(kKsLen - 2));
      r.ksGain = 0.985f + 0.0148f * color;  // Color is how long the string rings
      if (o.pluck || !sameSound) {
        for (int i = 0; i < kKsLen; ++i) o.ks[i] = 0;
        o.lp = 0.0f;
        o.fb = 0.0f;
        o.exciteLen = clampi((int)period, 2, kKsLen - 2);
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
  float lfo = p.lfo;
  if (p.lfoPerVoice) {  // this voice's own LFO, a little off the others' pace
    v.lfoPhase += p.lfoStep * kVoiceLfoRate[index] * ((float)n * (1.0f / (float)kCtrlBlock));
    if (v.lfoPhase >= 1.0f) {
      v.lfoPhase -= floorf_(v.lfoPhase);
      v.lfoRandom = nextNoise(v.rng);
    }
    lfo = lfoWave(p.lfoShape, v.lfoPhase, v.lfoRandom);
  }
  float lfoNow = lfo * v.lfoAmt;
  float lfoPw = p.lfoPwAmt * lfoNow;

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
  float base = v.pitch + p.bendSemis + p.lfoPitchAmt * lfoNow + detune - 69.0f;
  // Env>Pitch moves both oscillators. With Sync on it moves only oscillator
  // 2, the one being reset, which is what makes the classic sync sweep.
  float sweep = p.fenvPitch * v.fenv.level;
  float base1 = base + drift1 + (p.sync ? 0.0f : sweep);
  float base2 = base + drift2 + sweep;
  float invSr = 1.0f / sr_;
  float inc1 = minf(440.0f * exp2f_((base1 + p.oct1) * (1.0f / 12.0f)) * invSr, 0.45f);
  float inc2 = minf(440.0f * exp2f_((base2 + p.oct2 + p.detune2) * (1.0f / 12.0f)) * invSr, 0.45f);
  float incSub = minf(inc1 * p.subRatio, 0.45f);

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
  float oct = p.cutoffOct + p.lfoCutAmt * lfoNow + p.envAmt * v.fenv.level + p.velCut * v.vel + driftCut +
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
  float trem = 1.0f - p.lfoTremAmt * (0.5f - 0.5f * lfo) * v.lfoAmt;
  float amp = velGain * trem * (mode_ == MODE_UNI ? 0.45f : 1.0f);

  // Spread: where this voice sits between the speakers. A lone mono voice stays in the middle.
  float pan = mode_ == MODE_MONO ? 0.0f : p.spread * kVoicePan[index];
  float gainL = amp * (pan > 0.0f ? 1.0f - pan : 1.0f);
  float gainR = amp * (pan < 0.0f ? 1.0f + pan : 1.0f);

  bool on1 = p.mix[0] > 1e-4f, on2 = p.mix[1] > 1e-4f, onSub = p.mix[2] > 1e-4f, onNoise = p.mix[3] > 1e-4f;
  // Oscillator 2 is also needed, unheard, to bend oscillator 1. A plucked
  // string is always run: left alone with its fader down it would freeze
  // mid-pluck and sound when the fader came up.
  bool need2 = on2 || xmod > 0.0f || r2.engine == ENG_PLUCK;
  bool run1 = on1 || r1.engine == ENG_PLUCK;
  float chokeGain = 1.0f, chokeStep = v.choke ? 1.0f / (float)n : 0.0f;
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
    float s1 = run1 ? oscValue(v.o1, r1, xmod * s2, v.rng) : 0.0f;
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

    float out = x * ae * chokeGain;
    chokeGain -= chokeStep;
    outL[i] += out * gainL;
    outR[i] += out * gainR;
  }

  if (v.choke) v.aenv = Env{0.0f, ENV_IDLE};  // dropped to save processor time: it has just faded out
  if (v.aenv.stage == ENV_IDLE) {
    v.active = false;
    v.gate = false;
    v.keyDown = v.sustained = v.fresh = v.letGo = v.choke = false;
    v.fenv = Env{0.0f, ENV_IDLE};
  }
}

void Synth::process(const float* inL, const float* inR, float* outL, float* outR, int n) {
  uint8_t source = (uint8_t)setting(SET_CLOCK);
  if (source != clkSource_) {  // before the events: a start sent just after the setting changed must count
    clkSource_ = source;
    clockReset();
  }
  while (qRead_ != qWrite_) {
    Event e = queue_[qRead_];
    qRead_ = (qRead_ + 1) % kQueueLen;
    handleEvent(e.type, e.a, e.b);
  }
  if (overflow_) {  // an event was lost, perhaps a note-off: let everything go rather than risk a stuck note
    overflow_ = false;
    handleEvent(EV_ALL_NOTES_OFF, 0, 0);
    setSustain(false);
    pedal_ = false;
  }

  // ---- overload: shed a voice when blocks keep taking nearly all the time there is ----
  float load = load_;
  if (load > 0.85f) {
    calmSamples_ = 0.0f;
    if (++overBlocks_ >= 3) {
      overBlocks_ = 0;
      shedVoice();
    }
  } else {
    overBlocks_ = 0;
    if (voiceLimit_ < kNumVoices && load < 0.6f) {
      calmSamples_ += (float)n;
      if (calmSamples_ > 2.0f * sr_) {  // two calm seconds earn a voice back
        calmSamples_ = 0.0f;
        ++voiceLimit_;
      }
    }
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
    if (arp == ARP_OFF) {
      if (!latching()) dropUnheld();  // keys only the pedal was latching have no finger on them
      sustain_ = pedal_;              // the pedal goes back to being a sustain pedal
      for (int i = 0; i < setN_; ++i) soundKey(i);
    } else {
      setSustain(false);              // the pedal is now a temporary Hold, not sustain
    }
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
    clockAdvance(len);
    derive(p, len);
    arpAdvance(len);

    active = 0;
    for (int i = 0; i < kNumVoices; ++i) {
      if (!voices_[i].active) continue;
      renderVoice(voices_[i], i, p, busL, busR, len);
      ++active;
    }

    // ---- the audio input ----
    // Metered whenever the target supplies one, so its level can be set
    // before it is sent anywhere. A microphone is one channel: the left.
    const float* il = inL ? inL + done : nullptr;
    const float* ir = il && inR && setting(SET_INPUT_SOURCE) != INSRC_MIC ? inR + done : il;

    // ---- the vocoder: the voices' sound shaped by the voice at the input ----
    if (p.voc.amount > 0.001f || gVocoder.on()) {
      float voice[kCtrlBlock];
      if (il)
        for (int i = 0; i < len; ++i) voice[i] = 0.5f * (cleanIn(il[i]) + cleanIn(ir[i]));
      p.voc.playing = active > 0;
      gVocoder.process(busL, busR, il ? voice : nullptr, len, p.voc);
    }

    for (int i = 0; i < len; ++i) {
      busL[i] *= p.volume;
      busR[i] *= p.volume;
    }
    float inPeak = inPeak_ * peakDecay_;
    float inClip = inClip_ > (float)len ? inClip_ - (float)len : 0.0f;
    if (il) {
      for (int i = 0; i < len; ++i) {
        float a = maxf(absf(cleanIn(il[i])), absf(cleanIn(ir[i])));
        if (a > inPeak) inPeak = a;
      }
      if (inPeak >= 0.98f) inClip = 0.4f * sr_;  // at the converter's limit: the light stays on long enough to see
    }
    inPeak_ = inPeak < 1e-6f ? 0.0f : inPeak;
    inClip_ = inClip;
    int inTo = il ? setting(SET_INPUT_TO) : INPUT_OFF;
    if (!il) inGainMix_ = inGainFx_ = 0.0f;
    float fxTarget = inTo == INPUT_FX ? 1.0f : 0.0f;
    if (fxTarget > 0.0f || inGainFx_ > 0.0f) {  // into the effects, beside the synth's own sound
      float g = inGainFx_;
      for (int i = 0; i < len; ++i) {
        g += (fxTarget - g) * inCoef_;
        busL[i] += g * cleanIn(il[i]);
        busR[i] += g * cleanIn(ir[i]);
      }
      inGainFx_ = fxTarget == 0.0f && g < 1e-4f ? 0.0f : g;
    }

    float* l = outL + done;
    float* r = outR + done;
    gFx.process(busL, busR, l, r, len, p.fx);

    float mixTarget = inTo == INPUT_MIX ? 1.0f : 0.0f;
    if (mixTarget > 0.0f || inGainMix_ > 0.0f) {  // straight to the output, after the effects
      float g = inGainMix_;
      for (int i = 0; i < len; ++i) {
        g += (mixTarget - g) * inCoef_;
        l[i] += g * cleanIn(il[i]);
        r[i] += g * cleanIn(ir[i]);
      }
      inGainMix_ = mixTarget == 0.0f && g < 1e-4f ? 0.0f : g;
    }

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
  sampleNow_ += (uint32_t)n;

  // Notes that began in this block have now sounded: any that were let go
  // before they had can be released.
  for (int i = 0; i < kNumVoices; ++i) {
    Voice& v = voices_[i];
    if (!v.fresh) continue;
    v.fresh = false;
    if (v.letGo) {
      v.letGo = false;
      releaseVoice(v);
    }
  }
}

}  // namespace fw
