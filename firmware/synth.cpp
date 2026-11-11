#include "synth.h"

#include "fw_math.h"
#include "tables.h"

namespace fw {

Synth gSynth;
FW_BIGMEM float Synth::scope_[kScopeLen];

// Everything the voices need for one control block, worked out once from
// the (smoothed) parameters.
struct Synth::Patch {
  uint8_t wave1, wave2, ftype;
  bool slope24, driveOn;
  float oct1, oct2;      // semitones
  float detune2;         // semitones
  float pw1, pw2;
  float mix[4];          // osc1, osc2, sub, noise
  float cutoffOct;       // octaves above 20 Hz, LFO included
  float k, comp;         // filter damping, resonance loudness trim
  float envAmt;          // octaves
  float keytrack;        // 0..1
  float driveGain, driveMakeup;
  float glideCoef;       // 1 = no glide
  EnvCoef aenv, fenv;
  float velAmt;
  float lfoPitch;        // semitones
  float tremGain;
  float bendSemis;
  float uniDetune;       // semitones, half the total spread
  float subDiv;          // sub frequency as a fraction of osc 1
  float volume;
};

namespace {

inline float softClip(float x) {
  float a = absf(x);
  if (a <= 0.7f) return x;  // exactly linear at normal levels
  float y = 0.7f + 0.3f * tanhf_((a - 0.7f) * (1.0f / 0.3f));
  return x < 0.0f ? -y : y;
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
  float y = tabRead(o.tab, ph);
  if (o.xf > 0.0f) y += (tabRead(o.tabOld, ph) - y) * o.xf;
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
inline float pulseRead(const Osc& o, float pw) {
  float ph = o.phase;
  float ph2 = ph + 1.0f - pw;
  if (ph2 >= 1.0f) ph2 -= 1.0f;
  return readTable(o, ph) - readTable(o, ph2) - (2.0f * pw - 1.0f);
}

inline float oscTick(Osc& o, uint8_t wave, float inc, float pw) {
  float y;
  switch (wave) {
    case WAVE_SAW: y = readTable(o, o.phase); break;
    case WAVE_SQR: y = pulseRead(o, pw); break;
    case WAVE_TRI: y = readTable(o, o.phase); break;
    default: y = tabRead(gSine, o.phase); break;
  }
  advance(o, inc);
  return y;
}

inline void setTable(Osc& o, const float* t, bool fade) {
  if (t == o.tab) return;
  o.tabOld = fade ? o.tab : t;
  o.xf = fade ? 1.0f : 0.0f;
  o.tab = t;
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

inline float nextNoise(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return (float)(int32_t)s * (1.0f / 2147483648.0f);
}

}  // namespace

void Synth::init(float sampleRate) {
  sr_ = sampleRate;
  clock_ = 0;
  mode_ = (uint8_t)gParams.step(P_VOICE_MODE);
  lastPitch_ = 60.0f;
  stackN_ = 0;
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

  const float* rest = gSaw[kSawLevels - 1];
  for (int i = 0; i < kNumVoices; ++i) {
    Voice& v = voices_[i];
    v.active = v.gate = v.keyDown = v.sustained = false;
    v.note = 60;
    v.vel = 0.0f;
    v.age = 0;
    v.pitch = v.target = 60.0f;
    v.o1 = Osc{(float)i * 0.123f, rest, rest, 0.0f};
    v.o2 = Osc{(float)i * 0.071f + 0.3f, rest, rest, 0.0f};
    v.sub = Osc{0.0f, rest, rest, 0.0f};
    v.wave1 = v.wave2 = WAVE_SAW;
    v.f1 = Svf{0.0f, 0.0f};
    v.f2 = Svf{0.0f, 0.0f};
    v.aenv = Env{0.0f, ENV_IDLE};
    v.fenv = Env{0.0f, ENV_IDLE};
    v.rng = 0x9E3779B9u * (uint32_t)(i + 1) | 1u;
  }
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
    case EV_NOTE_ON: noteOn(a, b); break;
    case EV_NOTE_OFF: noteOff(a); break;
    case EV_SUSTAIN: setSustain(a >= 64); break;
    case EV_BEND: bend_ = (float)(((int)a | ((int)b << 7)) - 8192) * (1.0f / 8192.0f); break;
    case EV_MOD_WHEEL: modWheel_ = (float)a * (1.0f / 127.0f); break;
    case EV_ALL_NOTES_OFF: releaseAll(); break;
    case EV_ALL_SOUND_OFF: killAll(); break;
    default: break;
  }
}

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

  if (mode_ == MODE_POLY) {
    trigger(voices_[pickVoice(note)], note, vel);
    lastPitch_ = (float)note;
    return;
  }

  // mono and unison: last note wins, held notes are remembered
  bool wasHeld = stackN_ > 0;
  int w = 0;
  for (int i = 0; i < stackN_; ++i)
    if (stackNote_[i] != note) {
      stackNote_[w] = stackNote_[i];
      stackVel_[w] = stackVel_[i];
      ++w;
    }
  stackN_ = w;
  if (stackN_ == 16) {
    for (int i = 1; i < 16; ++i) {
      stackNote_[i - 1] = stackNote_[i];
      stackVel_[i - 1] = stackVel_[i];
    }
    stackN_ = 15;
  }
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

  int w = 0;
  for (int i = 0; i < stackN_; ++i)
    if (stackNote_[i] != note) {
      stackNote_[w] = stackNote_[i];
      stackVel_[w] = stackVel_[i];
      ++w;
    }
  stackN_ = w;

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
  sustain_ = false;
}

void Synth::derive(Patch& p, int len) {
  for (int i = 0; i < P_COUNT; ++i)
    if (kParams[i].kind == PK_POT) smooth_[i] += (gParams.norm(i) - smooth_[i]) * smoothCoef_;
  const float* sm = smooth_;
  float blockSec = (float)len / sr_;

  p.wave1 = (uint8_t)gParams.step(P_OSC1_WAVE);
  p.wave2 = (uint8_t)gParams.step(P_OSC2_WAVE);
  p.oct1 = (float)(gParams.step(P_OSC1_OCTAVE) - 2) * 12.0f;
  p.oct2 = (float)(gParams.step(P_OSC2_OCTAVE) - 2) * 12.0f;
  p.detune2 = mapDetuneCents(sm[P_OSC2_DETUNE]) * 0.01f;

  // ---- LFO: one for all voices, free running ----
  lfoPhase_ += mapLfoHz(sm[P_LFO_RATE]) * blockSec;
  if (lfoPhase_ >= 1.0f) {
    lfoPhase_ -= floorf_(lfoPhase_);
    lfoRandom_ = nextNoise(lfoRng_);
  }
  float lfo;
  switch (mapLfoShape(sm[P_LFO_SHAPE])) {
    case LFOS_SINE: lfo = tabRead(gSine, lfoPhase_); break;
    case LFOS_TRI: lfo = 1.0f - 4.0f * absf(lfoPhase_ - 0.5f); break;
    case LFOS_SAW: lfo = 1.0f - 2.0f * lfoPhase_; break;
    case LFOS_SQUARE: lfo = lfoPhase_ < 0.5f ? 1.0f : -1.0f; break;
    default: lfo = lfoRandom_; break;
  }
  lfoValue_ = lfo;
  float depth = clampf(sm[P_LFO_DEPTH] + modWheel_, 0.0f, 1.0f);  // mod wheel adds depth
  float lfoCut = 0.0f, lfoPw = 0.0f;
  p.lfoPitch = 0.0f;
  p.tremGain = 1.0f;
  switch (gParams.step(P_LFO_DEST)) {
    case LFO_PITCH: p.lfoPitch = lfo * depth * depth * 12.0f; break;
    case LFO_CUTOFF: lfoCut = lfo * depth * 4.0f; break;
    case LFO_AMP: p.tremGain = 1.0f - depth * (0.5f - 0.5f * lfo); break;
    default: lfoPw = lfo * depth * 0.4f; break;
  }

  p.pw1 = clampf(mapPulseWidth(sm[P_OSC1_MACRO]) + lfoPw, 0.05f, 0.95f);
  p.pw2 = clampf(0.5f + lfoPw, 0.05f, 0.95f);
  p.mix[0] = sm[P_MIX_OSC1];
  p.mix[1] = sm[P_MIX_OSC2];
  p.mix[2] = sm[P_MIX_SUB];
  p.mix[3] = sm[P_MIX_NOISE];

  // ---- filter ----
  p.ftype = (uint8_t)gParams.step(P_FILT_TYPE);
  p.slope24 = gParams.step(P_FILT_SLOPE) == 1;
  p.cutoffOct = sm[P_CUTOFF] * 10.0f + lfoCut;
  float q = mapResonanceQ(sm[P_RESONANCE]);
  p.k = 1.0f / q;
  p.comp = q > 1.0f ? 1.0f / sqrtf_(q) : 1.0f;  // keeps high resonance from getting far louder
  p.envAmt = mapEnvAmtOct(sm[P_FILT_ENV_AMT]);
  p.keytrack = sm[P_KEYTRACK];
  float dn = sm[P_DRIVE];
  p.driveOn = dn > 0.002f;
  p.driveGain = mapDriveGain(dn);
  p.driveMakeup = 1.0f / (1.0f + 0.09f * (p.driveGain - 1.0f));

  // ---- voice ----
  float glide = mapGlideTime(sm[P_GLIDE]);
  p.glideCoef = glide <= 0.0f ? 1.0f : 1.0f - exp2f_(-1.4427f * 3.0f * blockSec / glide);
  bendSmooth_ += (bend_ - bendSmooth_) * 0.25f;
  p.bendSemis = bendSmooth_ * (float)(gParams.step(P_BEND_RANGE) + kParams[P_BEND_RANGE].intMin);
  p.uniDetune = (float)gParams.step(P_UNI_DETUNE) * 0.01f;
  p.subDiv = gParams.step(P_SUB_OCTAVE) == 0 ? 0.5f : 0.25f;

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
}

void Synth::renderVoice(Voice& v, int index, const Patch& p, float* out, int n) {
  // ---- pitch ----
  v.pitch += (v.target - v.pitch) * p.glideCoef;
  float detune = mode_ == MODE_UNI ? p.uniDetune * ((float)index * (2.0f / (float)(kNumVoices - 1)) - 1.0f) : 0.0f;
  float base = v.pitch + p.bendSemis + p.lfoPitch + detune - 69.0f;
  float invSr = 1.0f / sr_;
  float inc1 = minf(440.0f * exp2f_((base + p.oct1) * (1.0f / 12.0f)) * invSr, 0.45f);
  float inc2 = minf(440.0f * exp2f_((base + p.oct2 + p.detune2) * (1.0f / 12.0f)) * invSr, 0.45f);
  float incSub = inc1 * p.subDiv;

  setTable(v.o1, p.wave1 == WAVE_TRI ? triTableFor(inc1) : sawTableFor(inc1), v.wave1 == p.wave1);
  setTable(v.o2, p.wave2 == WAVE_TRI ? triTableFor(inc2) : sawTableFor(inc2), v.wave2 == p.wave2);
  setTable(v.sub, sawTableFor(incSub), true);
  v.wave1 = p.wave1;
  v.wave2 = p.wave2;

  // ---- filter coefficients for this block ----
  float oct = p.cutoffOct + p.envAmt * v.fenv.level + p.keytrack * (v.pitch - 60.0f) * (1.0f / 12.0f);
  float fc = clampf(20.0f * exp2f_(oct), 20.0f, 0.45f * sr_);
  float g = tanf_(kPi * fc * invSr);
  SvfCoef c2 = svfCoef(g, p.k);
  SvfCoef c1 = svfCoef(g, 1.41421356f);  // first stage of the 24 dB slope: no peak of its own

  float velGain = 1.0f - p.velAmt * (1.0f - v.vel);
  float amp = velGain * p.tremGain * (mode_ == MODE_UNI ? 0.45f : 1.0f);

  bool on1 = p.mix[0] > 1e-4f, on2 = p.mix[1] > 1e-4f, onSub = p.mix[2] > 1e-4f, onNoise = p.mix[3] > 1e-4f;
  float m1 = p.mix[0] * 0.5f, m2 = p.mix[1] * 0.5f, m3 = p.mix[2] * 0.5f, m4 = p.mix[3] * 0.5f;

  for (int i = 0; i < n; ++i) {
    envTick(v.fenv, p.fenv);
    float ae = envTick(v.aenv, p.aenv);

    float x = 0.0f;
    if (on1) x += oscTick(v.o1, p.wave1, inc1, p.pw1) * m1;
    else advance(v.o1, inc1);
    if (on2) x += oscTick(v.o2, p.wave2, inc2, p.pw2) * m2;
    else advance(v.o2, inc2);
    if (onSub) x += pulseRead(v.sub, 0.5f) * m3;
    advance(v.sub, incSub);
    if (onNoise) x += nextNoise(v.rng) * m4;

    if (p.driveOn) x = tanhf_(x * p.driveGain) * p.driveMakeup;
    x *= p.comp;
    if (p.slope24) x = svfTick(v.f1, x, c1, p.ftype);
    x = svfTick(v.f2, x, c2, p.ftype);

    out[i] += x * ae * amp;
  }

  if (v.aenv.stage == ENV_IDLE) {
    v.active = false;
    v.gate = false;
    v.fenv = Env{0.0f, ENV_IDLE};
  }
}

void Synth::process(float* out, int n) {
  while (qRead_ != qWrite_) {
    Event e = queue_[qRead_];
    qRead_ = (qRead_ + 1) % kQueueLen;
    handleEvent(e.type, e.a, e.b);
  }

  uint8_t mode = (uint8_t)gParams.step(P_VOICE_MODE);
  if (mode != mode_) {  // switching voice mode lets go of everything
    releaseAll();
    mode_ = mode;
  }

  int done = 0;
  int active = 0;
  while (done < n) {
    int len = n - done < kCtrlBlock ? n - done : kCtrlBlock;
    float* o = out + done;
    for (int i = 0; i < len; ++i) o[i] = 0.0f;

    Patch p;
    derive(p, len);

    active = 0;
    for (int i = 0; i < kNumVoices; ++i) {
      if (!voices_[i].active) continue;
      renderVoice(voices_[i], i, p, o, len);
      ++active;
    }

    float peak = peak_ * peakDecay_;
    uint32_t pos = scopePos_;
    for (int i = 0; i < len; ++i) {
      float x = softClip(o[i] * p.volume);
      o[i] = x;
      scope_[pos & (kScopeLen - 1)] = x;
      ++pos;
      float a = absf(x);
      if (a > peak) peak = a;
    }
    scopePos_ = pos;
    peak_ = peak < 1e-6f ? 0.0f : peak;
    done += len;
  }
  activeCount_ = active;
}

}  // namespace fw
