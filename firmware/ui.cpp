#include "ui.h"

#include "firmware.h"
#include "fw_math.h"
#include "gfx.h"
#include "midi.h"
#include "params.h"
#include "synth.h"
#include "tables.h"

namespace fw {

Ui gUi;

namespace {

const char* const kModeNames[MB_COUNT] = {"FX 1", "FX 2", "MOD", "ENGINE", "PATCH"};

// The soft-control pages. One parameter per encoder, in encoder order
// (left-top, left-bottom, right-top, right-bottom); kNone leaves a slot empty.
constexpr uint8_t kNone = 255;
struct Page {
  const char* title;
  uint8_t param[4];
};
constexpr int kMaxPages = 2;
const Page kPages[MB_COUNT][kMaxPages] = {
    /* FX 1   */ {{"DELAY", {P_DLY_TIME, P_DLY_TONE, P_DLY_FDBK, P_DLY_MIX}}, {nullptr, {}}},
    /* FX 2   */ {{"CHORUS + REVERB", {P_CHORUS, P_RVB_SIZE, P_RVB_TONE, P_RVB_MIX}}, {nullptr, {}}},
    /* MOD    */ {{"MOD", {P_LFO_FADE, P_VEL_CUTOFF, P_FENV_PITCH, P_LFO_SYNC}}, {nullptr, {}}},
    /* ENGINE */ {{"VOICE", {P_UNI_DETUNE, P_SUB_OCTAVE, P_BEND_RANGE, P_FILT_SLOPE}},
                  {"OSC ENGINES", {P_OSC1_COLOR, kNone, P_OSC2_MACRO, P_OSC2_COLOR}}},
    /* PATCH  */ {{nullptr, {}}, {nullptr, {}}},
};

int pageCount(int mode) {
  int n = 0;
  while (n < kMaxPages && kPages[mode][n].title) ++n;
  return n;
}

// Which oscillator a parameter belongs to (0 or 1), or -1.
int oscOf(int id) {
  switch (id) {
    case P_OSC1_WAVE: case P_OSC1_OCTAVE: case P_OSC1_MACRO: case P_OSC1_COLOR: case P_OSC1_ENGINE: return 0;
    case P_OSC2_WAVE: case P_OSC2_OCTAVE: case P_OSC2_DETUNE: case P_OSC2_MACRO: case P_OSC2_COLOR: case P_OSC2_ENGINE: return 1;
    default: return -1;
  }
}

int engineOf(int osc) { return gParams.step(osc == 0 ? P_OSC1_ENGINE : P_OSC2_ENGINE); }

// A parameter's name on the display. Wave, Macro and Color mean different
// things under each oscillator engine, so their names follow the engine.
// Returns "" for a control the current engine does not use.
const char* paramName(int id) {
  int osc = oscOf(id);
  if (osc >= 0) {
    int e = engineOf(osc);
    if (id == P_OSC1_WAVE || id == P_OSC2_WAVE) return kEngineWaveName[e];
    if (id == P_OSC1_MACRO || id == P_OSC2_MACRO) {
      bool super = e == ENG_VA && gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE) == WAVE_SUPER;
      return super ? "SAW DETUNE" : kEngineMacroName[e];  // under the supersaw, Macro spreads the seven saws
    }
    if (id == P_OSC1_COLOR || id == P_OSC2_COLOR) return kEngineColorName[e];
  }
  return kParams[id].name;
}

// ---- small string builder (no printf in the core) ----
struct Str {
  char b[28];
  int n;
  Str() : n(0) { b[0] = 0; }
  void add(char c) {
    if (n < (int)sizeof(b) - 1) {
      b[n++] = c;
      b[n] = 0;
    }
  }
  void add(const char* s) {
    while (*s) add(*s++);
  }
  void addInt(int v) {
    if (v < 0) {
      add('-');
      v = -v;
    }
    char t[12];
    int k = 0;
    do {
      t[k++] = (char)('0' + v % 10);
      v /= 10;
    } while (v);
    while (k) add(t[--k]);
  }
  void addFixed(float v, int decimals) {
    if (v < 0.0f) {
      add('-');
      v = -v;
    }
    int scale = 1;
    for (int i = 0; i < decimals; ++i) scale *= 10;
    int whole = (int)(v * (float)scale + 0.5f);
    addInt(whole / scale);
    if (decimals > 0) {
      add('.');
      int frac = whole % scale;
      for (int s = scale / 10; s > 0; s /= 10) add((char)('0' + (frac / s) % 10));
    }
  }
  void addTime(float sec) {
    if (sec < 0.9995f) {
      addInt((int)(sec * 1000.0f + 0.5f));
      add("ms");
    } else {
      addFixed(sec, 2);
      add('s');
    }
  }
};

void formatParam(int id, Str& s) {
  const ParamDef& d = kParams[id];
  float n = gParams.norm(id);
  int st = gParams.step(id);
  int osc = oscOf(id);
  int engine = osc >= 0 ? engineOf(osc) : ENG_VA;
  if (id == P_OSC1_WAVE || id == P_OSC2_WAVE) {  // the stops mean something else under each engine
    s.add(kEngineWaveLabels[engine][st < WAVE_COUNT ? st : 0]);
    return;
  }
  bool superSaw = osc >= 0 && engine == ENG_VA && gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE) == WAVE_SUPER;
  if ((id == P_OSC1_MACRO || id == P_OSC2_MACRO) && superSaw) {
    s.addInt((int)((id == P_OSC1_MACRO ? n : gParams.frac(id)) * 100.0f + 0.5f));
    s.add('%');
    return;
  }
  if (id == P_OSC1_MACRO && engine != ENG_VA) {
    s.addInt((int)(n * 100.0f + 0.5f));
    s.add('%');
    return;
  }
  if (id == P_OSC2_MACRO && engine == ENG_VA) {
    s.addInt((int)(mapPulseWidth(gParams.frac(id)) * 100.0f + 0.5f));
    s.add('%');
    return;
  }
  switch (d.unit) {
    case U_STEP: s.add(d.labels[st]); break;
    case U_INT_PERCENT:
      s.addInt((int)(gParams.frac(id) * 100.0f + 0.5f));
      s.add('%');
      break;
    case U_INT_PLAIN: s.addInt(st + d.intMin); break;
    case U_BPM:
      s.addInt((int)mapTempoBpm(n));
      s.add(" BPM");
      break;
    case U_GATE:
      s.addInt((int)(mapArpGate(n) * 100.0f + 0.5f));
      s.add('%');
      break;
    case U_DLY_TIME: s.addTime(mapDelayTime(gParams.frac(id))); break;
    case U_RVB_TIME: s.addTime(mapReverbTime(gParams.frac(id))); break;
    case U_FADE_TIME: {
      float t = mapFadeTime(gParams.frac(id));
      if (t <= 0.0f) s.add("OFF");
      else s.addTime(t);
      break;
    }
    case U_PW:
      s.addInt((int)(mapPulseWidth(n) * 100.0f + 0.5f));
      s.add('%');
      break;
    case U_CENTS: {
      float c = mapDetuneCents(n);
      if (c > 0.0f) s.add('+');
      s.addFixed(c, 1);
      s.add(" ct");
      break;
    }
    case U_CUTOFF: {
      float hz = mapCutoffHz(n);
      if (hz < 1000.0f) {
        s.addInt((int)(hz + 0.5f));
        s.add("Hz");
      } else {
        s.addFixed(hz * 0.001f, hz < 10000.0f ? 2 : 1);
        s.add('k');
      }
      break;
    }
    case U_ENVAMT: {
      float o = mapEnvAmtOct(n);
      if (o > 0.0f) s.add('+');
      s.addFixed(o, 1);
      s.add(" oct");
      break;
    }
    case U_TIME: s.addTime(mapEnvTime(n)); break;
    case U_LFOHZ: {
      float hz = mapLfoHz(n);
      s.addFixed(hz, hz < 10.0f ? 2 : 1);
      s.add("Hz");
      break;
    }
    case U_GLIDE: {
      float t = mapGlideTime(n);
      if (t <= 0.0f) s.add("OFF");
      else s.addTime(t);
      break;
    }
    case U_INT_CENTS:
      s.addInt(st + d.intMin);
      s.add(" ct");
      break;
    case U_INT_SEMI:
      s.addInt(st + d.intMin);
      s.add(" st");
      break;
    default:
      s.addInt((int)(n * 100.0f + 0.5f));
      s.add('%');
      break;
  }
}

// ---- shared pieces ----

// One column of a plotted curve: joins the previous column's height to this
// one's with a vertical stroke. Steep edges (a square wave) come out as one
// clean upright line instead of a diagonal with a kink in it.
void plotColumn(int x, int yFrom, int yTo, Colour c) { gfxPlotLine(x, yFrom, x, yTo, c); }

// Live output scope, triggered on a rising zero crossing so the wave holds still.
void drawScope(int x, int y, int w, int h) {
  const float* buf = gSynth.scope();
  const uint32_t mask = kScopeLen - 1;
  int span = clampi((int)(gSynth.sampleRate() * 0.012f), w, 1024);
  uint32_t start = gSynth.scopeWritePos() - (uint32_t)span - 2u;
  for (int i = 0; i < kScopeLen - span - 8; ++i) {
    uint32_t a = start - (uint32_t)i;
    if (buf[(a - 1) & mask] < 0.0f && buf[a & mask] >= 0.0f) {
      start = a;
      break;
    }
  }
  float peak = 0.0f;
  for (int i = 0; i < span; ++i) peak = maxf(peak, absf(buf[(start + (uint32_t)i) & mask]));
  float gain = 0.9f / maxf(peak, 0.25f) * (float)(h / 2);

  int mid = y + h / 2;
  gfxHLine(x, mid, w, kGrid);
  float step = (float)span / (float)w;
  int py = mid;
  for (int px = 0; px < w; ++px) {
    float s = buf[(start + (uint32_t)((float)px * step)) & mask];
    int yy = clampi(mid - (int)(s * gain), y, y + h - 2);
    plotColumn(x + px, px > 0 ? py : yy, yy, kTeal);
    py = yy;
  }
}

// Bottom strip: voices in use, voice mode, output level, MIDI activity.
void drawStatus() {
  const int y = 222;
  for (int i = 0; i < kNumVoices; ++i) gfxFill(8 + i * 11, y + 3, 8, 8, gSynth.voiceActive(i) ? kTeal : kDarkGrey);
  if (gParams.step(P_HOLD)) gfxText(104, y, "HOLD", kAmber, 2);
  else if (gSynth.chordOn()) gfxText(104, y, "CHRD", kAmber, 2);
  else gfxText(104, y, kParams[P_VOICE_MODE].labels[gParams.step(P_VOICE_MODE)], kGrey, 2);

  float peak = minf(gSynth.peakLevel(), 1.0f);
  gfxFill(162, y + 3, 88, 8, kDarkGrey);
  gfxFill(162, y + 3, (int)(peak * 88.0f), 8, peak > 0.7f ? kAmber : kDimTeal);

  gfxText(258, y, "MIDI", gMidi.idleMs() < 120 ? kAmber : kDarkGrey, 2);  // lights up on every message
}

void drawHeader(const char* section, const char* name, const char* value) {
  gfxText(8, 6, section, kDimTeal, 2);
  gfxText(8, 26, name, kAmber, 2);
  gfxTextRight(312, 12, value, kWhite, 3);
  gfxHLine(8, 46, 304, kGrid);
}

void drawBar(float n) {
  gfxFrame(20, 120, 280, 28, kDarkGrey);
  gfxFill(22, 122, (int)(clampf(n, 0.0f, 1.0f) * 276.0f), 24, kTeal);
}

// A stepped value: every stop, with the selected one lit.
void drawSteps(const ParamDef& d, int cur) {
  int total = 0;
  for (int i = 0; i < d.steps; ++i) total += gfxTextWidth(d.labels[i], 2);
  int gap = (304 - total) / (d.steps + 1);
  int x = 8 + gap;
  for (int i = 0; i < d.steps; ++i) {
    int w = gfxTextWidth(d.labels[i], 2);
    gfxText(x, 126, d.labels[i], i == cur ? kAmber : kGrey, 2);
    if (i == cur) gfxFill(x, 146, w, 3, kAmber);
    x += w + gap;
  }
}

float waveShape(int wave, float ph, float pw) {
  switch (wave) {
    case WAVE_SAW: return 2.0f * ph - 1.0f;
    case WAVE_SQR: return ph < pw ? -1.0f : 1.0f;
    case WAVE_TRI: return ph < 0.25f ? 4.0f * ph : (ph < 0.75f ? 2.0f - 4.0f * ph : 4.0f * ph - 4.0f);
    default: return tabRead(gSine, ph);
  }
}

// One point of an oscillator's wave as it is set now, whichever engine it runs.
float oscShape(int osc, float ph) {
  int engine = engineOf(osc);
  int wave = gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE);
  if (wave >= WAVE_COUNT) wave = 0;
  float macro = osc == 0 ? gParams.norm(P_OSC1_MACRO) : gParams.frac(P_OSC2_MACRO);
  float color = gParams.frac(osc == 0 ? P_OSC1_COLOR : P_OSC2_COLOR);
  switch (engine) {
    case ENG_VA:
      if (wave == WAVE_SUPER) {  // a few of the seven saws, drifting apart as the detune goes up
        float spread = 0.05f + 0.95f * macro * macro;
        float a = 2.0f * ph - 1.0f;
        float p2 = ph * (1.0f + 0.11f * spread) + 0.31f, p3 = ph * (1.0f - 0.11f * spread) + 0.67f;
        p2 -= floorf_(p2);
        p3 -= floorf_(p3);
        return (a + 0.6f * (2.0f * p2 - 1.0f) + 0.6f * (2.0f * p3 - 1.0f)) * 0.5f;
      }
      return waveShape(wave, ph, mapPulseWidth(macro));
    case ENG_FM: {
      static const float kRatios[WAVE_COUNT] = {1.0f, 2.0f, 3.0f, 3.5f, 4.0f};
      float m = sinf_(kTwoPi * ph * kRatios[wave]);
      return sinf_(kTwoPi * ph + macro * macro * 8.0f * m);
    }
    case ENG_WAVETABLE: {
      float pos = macro * (float)(kWtFrames - 1);
      int f0 = pos >= (float)(kWtFrames - 1) ? kWtFrames - 2 : (int)pos;
      int i = (int)(ph * (float)kWtLen) & (kWtLen - 1);
      float a = (float)gWt[wave][f0][0][i], b = (float)gWt[wave][f0 + 1][0][i];
      return (a + (b - a) * (pos - (float)f0)) * (1.0f / 32000.0f);
    }
    default: {  // pluck: a wave that dies away, faster and duller with more damping
      float decay = exp2f_(-ph * (5.0f - 4.0f * color));
      return sinf_(kTwoPi * ph * 4.0f) * decay * (1.0f - 0.5f * macro);
    }
  }
}

void drawOscWave(int osc) {
  const int x0 = 10, w = 300, mid = 138, amp = 64;
  gfxHLine(x0, mid, w, kGrid);
  bool once = engineOf(osc) == ENG_PLUCK;  // a pluck is one event, not a repeating wave
  bool super = engineOf(osc) == ENG_VA && gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE) == WAVE_SUPER;
  int py = 0;
  for (int px = 0; px < w; ++px) {
    float ph = (float)px * ((once ? 1.0f : super ? 4.0f : 2.0f) / (float)w);
    if (!once && !super) ph -= floorf_(ph);
    int yy = clampi(mid - (int)(oscShape(osc, ph) * (float)amp), 56, 216);
    plotColumn(x0 + px, px > 0 ? py : yy, yy, kTeal);
    py = yy;
  }
}

void drawMixer(int selected) {
  static const char* const kNames[4] = {"OSC 1", "OSC 2", "SUB", "NOISE"};
  for (int i = 0; i < 4; ++i) {
    int id = P_MIX_OSC1 + i;
    int x = 30 + i * 72;
    int h = (int)(gParams.norm(id) * 126.0f);
    Colour c = id == selected ? kAmber : kDimTeal;
    gfxFrame(x, 56, 44, 130, kDarkGrey);
    gfxFill(x + 2, 184 - h, 40, h, c);
    gfxTextCentre(x + 22, 194, kNames[i], id == selected ? kAmber : kGrey, 2);
  }
}

float filterStage(float w, float k, int type) {
  float w2 = w * w;
  float den = sqrtf_((1.0f - w2) * (1.0f - w2) + k * k * w2);
  if (den < 1e-9f) den = 1e-9f;
  switch (type) {
    case FILT_LP: return 1.0f / den;
    case FILT_BP: return (k > 1.0f ? k : 1.0f) * w / den;
    default: return w2 / den;
  }
}

// Spread: where each of the eight voices sits between the two speakers.
void drawSpread() {
  static const float kPan[kNumVoices] = {-1.0f, 1.0f, -0.6f, 0.6f, -0.3f, 0.3f, -0.8f, 0.8f};
  const int mid = 160, half = 130, y = 128;
  float spread = gParams.norm(P_SPREAD);
  gfxHLine(mid - half, y + 5, 2 * half + 1, kGrid);
  gfxVLine(mid, y - 30, 70, kGrid);
  for (int i = 0; i < kNumVoices; ++i) {
    int x = mid + (int)(kPan[i] * spread * (float)half);
    gfxFill(x - 5, y - 14 + (i % 4) * 9, 10, 8, gSynth.voiceActive(i) ? kAmber : kTeal);
  }
  gfxText(mid - half, 190, "L", kGrey, 2);
  gfxTextRight(mid + half, 190, "R", kGrey, 2);
}

// ---- the filter pictures ----
// All share one frame: frequency left to right, 20 Hz to 20 kHz, level up and down.
constexpr int kFx0 = 8, kFw = 304, kFy0 = 54, kFh = 156;
constexpr float kFTopDb = 18.0f, kFRangeDb = 66.0f;

int freqX(float hz) { return kFx0 + clampi((int)(log2f_(maxf(hz, 20.0f) / 20.0f) * 0.1f * (float)kFw), 0, kFw - 1); }

void drawFilterAxes() {
  int zero = kFy0 + (int)(kFTopDb / kFRangeDb * (float)kFh);
  gfxHLine(kFx0, zero, kFw, kGrid);
  static const float kMarks[3] = {100.0f, 1000.0f, 10000.0f};
  static const char* const kMarkNames[3] = {"100", "1k", "10k"};
  for (int i = 0; i < 3; ++i) {
    int x = freqX(kMarks[i]);
    gfxVLine(x, kFy0, kFh, kGrid);
    gfxTextCentre(x, kFy0 + kFh + 4, kMarkNames[i], kGrey, 2);
  }
}

// The filter's response with its cutoff at `fc`, everything else as set now.
void plotFilter(float fc, Colour c) {
  float q = mapResonanceQ(gParams.norm(P_RESONANCE));
  float k = 1.0f / q;
  float comp = q > 1.0f ? 1.0f / sqrtf_(q) : 1.0f;
  int type = gParams.step(P_FILT_TYPE);
  int slope = gParams.step(P_FILT_SLOPE);
  float lk = mapLadderFeedback(gParams.norm(P_RESONANCE));
  int py = 0;
  for (int px = 0; px < kFw; ++px) {
    float f = 20.0f * exp2f_(10.0f * (float)px / (float)(kFw - 1));
    float wn = f / fc;
    float mag;
    if (slope == SLOPE_LADDER) {  // four stages and their feedback: (1 + jw)^4 + k
      float a = 1.0f - wn * wn, b = 2.0f * wn;
      float re = a * a - b * b + lk, im = 2.0f * a * b;
      mag = (1.0f + 0.5f * lk) / sqrtf_(maxf(re * re + im * im, 1e-12f));
    } else {
      mag = filterStage(wn, k, type) * comp;
      if (slope == SLOPE_24) mag *= filterStage(wn, 1.41421356f, type);
    }
    float db = 6.0206f * log2f_(maxf(mag, 1e-6f));
    int yy = clampi(kFy0 + (int)((kFTopDb - db) / kFRangeDb * (float)kFh), kFy0, kFy0 + kFh - 2);
    plotColumn(kFx0 + px, px > 0 ? py : yy, yy, c);
    py = yy;
  }
}

float cutoffNow() { return mapCutoffHz(gParams.norm(P_CUTOFF)); }
float clampCutoff(float hz) { return clampf(hz, 20.0f, 20480.0f); }

// Cutoff, Resonance, Type: the response as it is set right now.
void drawFilterCurve() {
  drawFilterAxes();
  plotFilter(cutoffNow(), kTeal);
}

// Env amount: where the filter rests, where the envelope pushes it at its
// peak, and an arrow for the sweep between the two.
void drawFilterEnvAmount() {
  drawFilterAxes();
  float rest = cutoffNow();
  float oct = mapEnvAmtOct(gParams.norm(P_FILT_ENV_AMT));
  float peak = clampCutoff(rest * exp2f_(oct));
  if (oct == 0.0f) {
    plotFilter(rest, kTeal);
    gfxTextCentre(160, kFy0 + 2, "NO SWEEP", kGrey, 2);
    return;
  }
  plotFilter(rest, kDimTeal);
  plotFilter(peak, kAmber);
  int x0 = freqX(rest), x1 = freqX(peak), y = kFy0 + 6;
  if (x1 == x0) {  // the cutoff is already as far as it goes that way: nothing left to sweep
    gfxTextCentre(160, kFy0 + 2, oct > 0.0f ? "CUTOFF ALREADY OPEN" : "CUTOFF ALREADY SHUT", kGrey, 2);
    return;
  }
  int dir = x1 > x0 ? 1 : -1;
  gfxFill(x0 < x1 ? x0 : x1, y, (x1 > x0 ? x1 - x0 : x0 - x1) + 1, 2, kAmber);
  for (int i = 1; i <= 5; ++i) gfxFill(x1 - dir * i, y - i, 2, 2 * i + 2, kAmber);  // arrow head at the peak end
  gfxFill(x0, y - 4, 2, 10, kAmber);                                               // a tick where it rests
}

// Keytrack: the response for a low, a middle and a high key. With no
// tracking the three sit on top of each other; with full tracking the
// filter moves two octaves for two octaves of keys.
void drawFilterKeytrack() {
  drawFilterAxes();
  float kt = gParams.norm(P_KEYTRACK);
  float mid = cutoffNow();
  float low = clampCutoff(mid * exp2f_(-2.0f * kt)), high = clampCutoff(mid * exp2f_(2.0f * kt));
  plotFilter(low, kDimTeal);
  plotFilter(high, kDimTeal);
  plotFilter(mid, kTeal);
  int xl = freqX(low), xm = freqX(mid), xh = freqX(high);
  if (xh - xl < 60) {
    gfxTextCentre(160, kFy0 + 2, kt < 0.02f ? "SAME FOR EVERY KEY" : "LOW AND HIGH KEYS", kGrey, 2);
    return;
  }
  gfxTextCentre(clampi(xl, 20, 300), kFy0 + 2, "C2", kGrey, 2);
  gfxTextCentre(clampi(xh, 20, 300), kFy0 + 2, "C6", kGrey, 2);
  if (xm - xl > 44 && xh - xm > 44) gfxTextCentre(xm, kFy0 + 2, "C4", kTeal, 2);
}

// Drive: a smooth wave going in (dim) and what the two saturators make of
// it (bright). The squarer it gets, the more harmonics are added.
void drawFilterDrive() {
  const int x0 = 10, w = 300, mid = 132, amp = 62;
  gfxHLine(x0, mid, w, kGrid);
  DriveSettings d = mapDrive(gParams.norm(P_DRIVE));
  const float kIn = 0.5f, kBetween = 0.3f;  // typical levels at the two stages
  auto through = [&](float x) {
    return driveStage(driveStage(x, d.pre, d.preTrim, d.mix) * (kBetween / kIn), d.post, d.postTrim, d.mix);
  };
  float full = through(kIn);
  int pyIn = 0, pyOut = 0;
  for (int px = 0; px < w; ++px) {
    float s = sinf_(kTwoPi * 2.0f * (float)px / (float)w);
    float out = through(s * kIn) / full;
    int yIn = mid - (int)(s * (float)amp), yOut = mid - (int)(out * (float)amp);
    plotColumn(x0 + px, px > 0 ? pyIn : yIn, yIn, kGrey);
    pyIn = yIn;
    plotColumn(x0 + px, px > 0 ? pyOut : yOut, yOut, kTeal);
    pyOut = yOut;
  }
  gfxText(8, 204, "IN", kGrey, 2);
  gfxTextRight(312, 204, "OUT", kTeal, 2);
}

void drawAdsr(int firstParam, int selected) {
  float a = gParams.norm(firstParam), d = gParams.norm(firstParam + 1);
  float s = gParams.norm(firstParam + 2), r = gParams.norm(firstParam + 3);
  const int top = 62, bottom = 196, hgt = bottom - top;
  int wA = 16 + (int)(70.0f * a), wD = 16 + (int)(70.0f * d), wS = 50, wR = 16 + (int)(70.0f * r);
  int x = 12;
  gfxHLine(8, bottom + 2, 304, kGrid);

  auto yOf = [&](float level) { return bottom - (int)(level * (float)hgt); };
  auto seg = [&](int width, int which, float (*shape)(float t, float s), float sus) {
    Colour c = (firstParam + which == selected) ? kAmber : kTeal;
    int py = yOf(shape(0.0f, sus));
    for (int i = 1; i <= width; ++i) {
      int yy = yOf(shape((float)i / (float)width, sus));
      plotColumn(x + i, py, yy, c);
      py = yy;
    }
    x += width;
  };
  seg(wA, 0, [](float t, float) { return 1.3f * (1.0f - exp2f_(-2.1155f * t)); }, s);
  seg(wD, 1, [](float t, float sus) { return sus + (1.0f - sus) * exp2f_(-6.0f * t); }, s);
  seg(wS, 2, [](float, float sus) { return sus; }, s);
  seg(wR, 3, [](float t, float sus) { return sus * exp2f_(-6.0f * t); }, s);

  static const char* const kNames[4] = {"A", "D", "S", "R"};
  int lx = 12;
  const int widths[4] = {wA, wD, wS, wR};
  for (int i = 0; i < 4; ++i) {
    gfxTextCentre(lx + widths[i] / 2, bottom + 8, kNames[i], firstParam + i == selected ? kAmber : kGrey, 2);
    lx += widths[i];
  }
}

void drawLfo() {
  static const float kRandom[8] = {0.3f, -0.7f, 0.9f, -0.2f, 0.6f, -0.9f, 0.1f, -0.5f};
  int shape = gParams.step(P_LFO_SHAPE);
  const int x0 = 10, w = 300, mid = 124, amp = 54;
  gfxHLine(x0, mid, w, kGrid);
  int py = 0;
  for (int px = 0; px < w; ++px) {
    float t = (float)px * (2.0f / (float)w);
    float ph = t - floorf_(t);
    float v;
    switch (shape) {
      case LFOS_SINE: v = tabRead(gSine, ph); break;
      case LFOS_TRI: v = 1.0f - 4.0f * absf(ph - 0.5f); break;
      case LFOS_SAW: v = 1.0f - 2.0f * ph; break;
      case LFOS_SQUARE: v = ph < 0.5f ? 1.0f : -1.0f; break;
      default: v = kRandom[(int)(t * 4.0f) & 7]; break;
    }
    int yy = mid - (int)(v * (float)amp);
    plotColumn(x0 + px, px > 0 ? py : yy, yy, kTeal);
    py = yy;
  }
  Str s;
  s.add("TO ");
  s.add(kParams[P_LFO_DEST].labels[gParams.step(P_LFO_DEST)]);
  gfxText(8, 200, s.b, kGrey, 2);
  Str r;
  formatParam(P_LFO_RATE, r);
  gfxTextRight(312, 200, r.b, kGrey, 2);
}

}  // namespace

void Ui::init() {
  mode_ = MB_NONE;
  page_ = 0;
  popupParam_ = -1;
  popupMs_ = 0;
  for (int i = 0; i < 4; ++i) burst_[i] = 0;
  dirty_ = true;
}

// Parameter on encoder `enc` of the page showing now, or -1.
int Ui::pageParam(int enc) const {
  if (mode_ == MB_NONE || enc < 0 || enc >= 4 || page_ >= pageCount(mode_)) return -1;
  uint8_t id = kPages[mode_][page_].param[enc];
  if (id == kNone) return -1;
  if (paramName(id)[0] == 0) return -1;  // the oscillator's engine has no use for this one
  return id;
}

void Ui::paramChanged(int id) {
  // A value already on the page showing now updates in place.
  for (int e = 0; e < 4; ++e)
    if (pageParam(e) == id) {
      dirty_ = true;
      return;
    }
  popupParam_ = (int16_t)id;
  popupMs_ = kPopupMs;
  dirty_ = true;
}

void Ui::modeButton(int which) {
  if (which < 0 || which >= MB_COUNT) return;
  if (mode_ != which) {
    mode_ = (int8_t)which;
    page_ = 0;
  } else if (page_ + 1 < pageCount(which)) {
    ++page_;  // the lit button steps to the mode's next page
  } else {
    mode_ = MB_NONE;  // and after the last page, home
    page_ = 0;
  }
  popupParam_ = -1;
  popupMs_ = 0;
  dirty_ = true;
}

void Ui::encoderTurn(int enc, int delta) {
  int id = pageParam(enc);
  if (id < 0) return;
  // Long ranges would take many turns one click at a time: a fast spin
  // (several clicks within one display tick) moves in bigger steps.
  if (kParams[id].steps > 30) {
    int n = burst_[enc] < 250 ? ++burst_[enc] : 250;
    if (n >= 4) delta *= 5;
    else if (n >= 2) delta *= 2;
  }
  if (gParams.set(id, gParams.step(id) + delta)) onParamChanged(id);
}

void Ui::encoderPush(int enc) {
  int id = pageParam(enc);
  if (id < 0) return;
  if (gParams.set(id, kParams[id].def)) onParamChanged(id);  // push = back to default
}

bool Ui::tick(uint32_t dtMs) {
  for (int i = 0; i < 4; ++i) burst_[i] = 0;

  bool popup = popupMs_ > 0;
  if (popup) {
    if (popupMs_ > dtMs) {
      popupMs_ -= dtMs;
    } else {
      popupMs_ = 0;
      popupParam_ = -1;
      popup = false;
      dirty_ = true;
    }
  }

  // Pop-ups are static; the other screens carry the live scope and status.
  if (popup && !dirty_) return false;
  dirty_ = false;

  gfxClear(kBlack);
  if (popup && popupParam_ >= 0) drawParamPopup(popupParam_);
  else if (mode_ != MB_NONE) drawModePage();
  else drawHome();
  return true;
}

void Ui::drawHome() {
  gfxTextCentre(160, 10, "001 INIT", kAmber, 2);
  drawScope(8, 40, 304, 168);
  drawStatus();
}

void Ui::drawModePage() {
  int pages = pageCount(mode_);
  if (pages == 0) {
    gfxTextCentre(160, 6, kModeNames[mode_], kAmber, 2);
    gfxTextCentre(160, 104, "NOT BUILT YET", kGrey, 2);
    drawStatus();
    return;
  }
  Str title;
  title.add(kPages[mode_][page_].title);
  if (pages > 1) {
    title.add(' ');
    title.addInt(page_ + 1);
    title.add('/');
    title.addInt(pages);
  }
  gfxTextCentre(160, 6, title.b, kAmber, 2);

  // one value per encoder, at the corner nearest that encoder
  static const int kLabelY[2] = {30, 164};
  for (int e = 0; e < 4; ++e) {
    int id = pageParam(e);
    int y = kLabelY[e & 1];
    Str label, v;
    if (id < 0) {
      label.add("--");
    } else {
      int osc = oscOf(id);
      if (osc >= 0) label.add(osc == 0 ? "O1 " : "O2 ");
      label.add(paramName(id));
      formatParam(id, v);
    }
    if (e < 2) {
      gfxText(8, y, label.b, kDimTeal, 2);
      gfxText(8, y + 16, v.b, kWhite, 3);
    } else {
      gfxTextRight(312, y, label.b, kDimTeal, 2);
      gfxTextRight(312, y + 16, v.b, kWhite, 3);
    }
  }
  drawScope(8, 74, 304, 82);
  drawStatus();
}

void Ui::drawParamPopup(int id) {
  const ParamDef& d = kParams[id];
  Str v;
  formatParam(id, v);
  const char* name = paramName(id);
  drawHeader(kSectionNames[d.section], name[0] ? name : d.name, v.b);

  bool engineSwitch = id == P_OSC1_ENGINE || id == P_OSC2_ENGINE;
  switch (engineSwitch ? (int)SEC_COUNT : (int)d.section) {
    case SEC_OSC1:
      drawOscWave(0);
      break;
    case SEC_OSC2:
      if (id == P_XMOD) drawBar(gParams.norm(id));
      else if (id == P_SYNC) drawSteps(d, gParams.step(id));
      else drawOscWave(1);
      break;
    case SEC_MIXER:
      drawMixer(id);
      break;
    case SEC_FILTER:
      if (id == P_FILT_ENV_AMT) drawFilterEnvAmount();
      else if (id == P_KEYTRACK) drawFilterKeytrack();
      else if (id == P_DRIVE) drawFilterDrive();
      else drawFilterCurve();
      break;
    case SEC_FENV:
      drawAdsr(P_FENV_A, id);
      break;
    case SEC_AENV:
      if (id == P_VELOCITY) drawBar(gParams.norm(id));
      else drawAdsr(P_AENV_A, id);
      break;
    case SEC_LFO:
      drawLfo();
      break;
    default:
      if (id == P_SPREAD) drawSpread();
      else if (d.kind == PK_STEP && d.labels) drawSteps(d, gParams.step(id));
      else if (d.kind == PK_STEP) drawBar(gParams.frac(id));
      else drawBar(gParams.norm(id));
      break;
  }
}

}  // namespace fw
