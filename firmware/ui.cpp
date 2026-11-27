#include "ui.h"

#include "draw.h"
#include "firmware.h"
#include "fw_math.h"
#include "gfx.h"
#include "midi.h"
#include "params.h"
#include "shapes.h"
#include "synth.h"
#include "tables.h"
#include "vocoder.h"

namespace fw {

Ui gUi;

namespace {

// The soft-control pages. One parameter per encoder, in encoder order
// (left-top, left-bottom, right-top, right-bottom); kNone leaves a slot empty.
constexpr uint8_t kNone = 255;
struct Page {
  const char* title;
  uint8_t param[4];
};
constexpr int kMaxPages = 5;
const Page kPages[MB_COUNT][kMaxPages] = {
    /* FX 1   */ {{"DELAY", {P_DLY_TIME, P_DLY_TONE, P_DLY_FDBK, P_DLY_MIX}}, {nullptr, {}}},
    /* FX 2   */ {{"COMPRESSOR + REVERB", {P_COMPRESS, P_RVB_SIZE, P_RVB_TONE, P_RVB_MIX}},
                  {"VOCODER", {P_VOCODER, P_VOC_FORMANT, P_VOC_CONSONANTS, P_VOC_SMOOTH}}},
    /* MOD    */ {{"MOD", {P_LFO_FADE, P_VEL_CUTOFF, P_FENV_PITCH, P_DUCK}}, {nullptr, {}}},
    /* ENGINE */ {{"VOICE", {P_UNI_DETUNE, P_SUB_DETUNE, P_BEND_RANGE, P_FILT_SLOPE}},
                  {"OSC ENGINES", {P_OSC1_COLOR, kNone, P_OSC2_MACRO, P_OSC2_COLOR}},
                  // The DRAW page edits the patch's drawing: handled by drawTurn and drawPush.
                  {"DRAW", {kNone, kNone, kNone, kNone}}},
    // The Patch pages have actions, not parameters: they are handled by patchTurn and patchPush.
    /* PATCH  */ {{"PATCH", {kNone, kNone, kNone, kNone}}, {"NAME", {kNone, kNone, kNone, kNone}},
                  {"SETUP", {kNone, kNone, kNone, kNone}}, {"MIDI", {kNone, kNone, kNone, kNone}},
                  {"INPUT", {kNone, kNone, kNone, kNone}}},
};

// The last two Patch pages hold settings (settings.h), one per encoder.
constexpr int kFirstSettingsPage = 3;
const uint8_t kSettingPages[2][4] = {
    {SET_CLOCK, SET_CHANNEL, SET_THRU, SET_CLOCK_OUT},
    {SET_INPUT_SOURCE, SET_INPUT_GAIN, SET_INPUT_TO, kNone},
};

constexpr uint16_t kArmMs = 3000, kNoteMs = 2000;
const char* const kGuideLabels[GUIDE_COUNT] = {"OFF", "TINT", "ON"};

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
      if (e != ENG_VA) return kEngineMacroName[e];
      // Under the plain engine Macro shapes whichever wave is selected.
      static const char* const kVaMacro[WAVE_COUNT] = {"SOFTEN", "PULSE WIDTH", "SHARK FIN", "SQUARE UP", "SPREAD", "SOFTEN"};
      int wave = gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE);
      return kVaMacro[wave < WAVE_COUNT ? wave : 0];
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
  if (id == P_OSC1_MACRO || id == P_OSC2_MACRO) {
    // pulse width on the plain engine's square; elsewhere how far Macro is turned
    float m = id == P_OSC1_MACRO ? n : gParams.frac(id);
    bool square = engine == ENG_VA && gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE) == WAVE_SQR;
    s.addInt((int)((square ? mapPulseWidth(m) : m) * 100.0f + 0.5f));
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
    plotColumn(x + px, px > 0 ? py : yy, yy, kLine);
    py = yy;
  }
}

// Bottom strip: voices in use, voice mode, output level, MIDI activity.
void drawStatus() {
  const int y = 222;
  // one square per voice; while the processor is overloaded, the voices it has given up are left out
  for (int i = 0; i < gSynth.voiceLimit(); ++i) gfxFill(8 + i * 11, y + 3, 8, 8, gSynth.voiceActive(i) ? kLine : kDarkGrey);
  bool hold = gParams.step(P_HOLD) != 0, chord = gSynth.chordOn();
  if (hold || chord) gfxText(104, y, hold && chord ? "H+CH" : hold ? "HOLD" : "CHRD", kAccent, 2);
  else gfxText(104, y, kParams[P_VOICE_MODE].labels[gParams.step(P_VOICE_MODE)], kGrey, 2);

  float peak = minf(gSynth.peakLevel(), 1.0f);
  gfxFill(162, y + 3, 88, 8, kDarkGrey);
  gfxFill(162, y + 3, (int)(peak * 88.0f), 8, peak > 0.7f ? kAccent : kLineDim);

  gfxText(258, y, "MIDI", gMidi.idleMs() < 120 ? kAccent : kDarkGrey, 2);  // lights up on every message
}

void drawHeader(const char* section, const char* name, const char* value) {
  gfxText(8, 6, section, kLineDim, 2);
  gfxText(8, 26, name, kAccent, 2);
  gfxTextRight(312, 12, value, kInk, 3);
  gfxHLine(8, 46, 304, kGrid);
}

void drawBar(float n) {
  gfxFrame(20, 120, 280, 28, kDarkGrey);
  gfxFill(22, 122, (int)(clampf(n, 0.0f, 1.0f) * 276.0f), 24, kLine);
}

// A stepped value: every stop, with the selected one lit.
void drawSteps(const ParamDef& d, int cur, int y = 126) {
  int total = 0;
  for (int i = 0; i < d.steps; ++i) total += gfxTextWidth(d.labels[i], 2);
  int gap = (304 - total) / (d.steps + 1);
  int x = 8 + gap;
  for (int i = 0; i < d.steps; ++i) {
    int w = gfxTextWidth(d.labels[i], 2);
    gfxText(x, y, d.labels[i], i == cur ? kAccent : kGrey, 2);
    if (i == cur) gfxFill(x, y + 20, w, 3, kAccent);
    x += w + gap;
  }
}

// Where a picture is drawn. Pop-ups use the whole area under the header;
// the encoder pages use the band in the middle, where the scope sits.
struct Box {
  int x, y, w, h;
};
constexpr Box kPic = {8, 54, 304, 158};
constexpr Box kMini = {8, 76, 304, 78};

// The part of x after the point, always below 1. (For a tiny negative x,
// x - floor(x) rounds to exactly 1, which is one past the end of a table.)
float frac01(float x) {
  x -= floorf_(x);
  return x >= 1.0f ? 0.0f : x;
}

// A line of explanation over a picture, on a black strip so it never tangles with a curve.
void hint(int y, const char* text) {
  int w = gfxTextWidth(text, 2);
  gfxFill(160 - w / 2 - 4, y - 2, w + 8, 18, kBlack);
  gfxTextCentre(160, y, text, kGrey, 2);
}

float waveShape(int wave, float ph, float pw) {
  switch (wave) {
    case WAVE_SAW: return 2.0f * ph - 1.0f;
    case WAVE_SQR: return ph < pw ? -1.0f : 1.0f;
    case WAVE_TRI: return ph < 0.25f ? 4.0f * ph : (ph < 0.75f ? 2.0f - 4.0f * ph : 4.0f * ph - 4.0f);
    default: return tabRead(gSine, ph);
  }
}

int waveOf(int osc) {
  int wave = gParams.step(osc == 0 ? P_OSC1_WAVE : P_OSC2_WAVE);
  return wave < WAVE_COUNT ? wave : 0;
}
float octaveOf(int osc) { return (float)(gParams.step(osc == 0 ? P_OSC1_OCTAVE : P_OSC2_OCTAVE) - 2); }
bool isSuper(int osc) { return engineOf(osc) == ENG_VA && waveOf(osc) == WAVE_SUPER; }

// The drawing as a wave, 0..1 through its cycle: its points joined by straight lines.
float drawnAt(const int8_t* pts, float ph) {
  float pos = frac01(ph) * (float)kDrawPoints;
  int i = (int)pos;
  float a = (float)pts[i & (kDrawPoints - 1)], b = (float)pts[(i + 1) & (kDrawPoints - 1)];
  return (a + (b - a) * (pos - (float)i)) * (1.0f / 127.0f);
}

// Does this oscillator play the drawing? (Under FM the sixth stop is a ratio.)
bool playsDrawing(int osc) { return waveOf(osc) == WAVE_DRAW && engineOf(osc) != ENG_FM; }

// One cycle of a plain-engine wave as Macro shapes it, worked out by running
// the oscillator's own shaping on it (shapes.h), at 256 points a cycle.
// Kept for each oscillator until its wave, Macro or the drawing changes.
constexpr int kCycle = 256;
float gCycle[2][kCycle];
float gCycleMacro[2] = {-1.0f, -1.0f};
int gCycleWave[2] = {-1, -1};
uint32_t gCycleDraw[2];

uint32_t drawingHash() {
  uint32_t h = 2166136261u;
  for (int i = 0; i < kDrawPoints; ++i) h = (h ^ (uint8_t)gDraw.point(i)) * 16777619u;
  return h;
}

const float* shapedCycle(int osc, int wave, float macro) {
  uint32_t dh = wave == WAVE_DRAW ? drawingHash() : 0;
  if (gCycleWave[osc] == wave && gCycleMacro[osc] == macro && gCycleDraw[osc] == dh) return gCycle[osc];
  gCycleWave[osc] = wave;
  gCycleMacro[osc] = macro;
  gCycleDraw[osc] = dh;
  float mean = 0.0f;
  if (wave == WAVE_DRAW) {  // a drawing is heard with its mean taken off
    for (int i = 0; i < kDrawPoints; ++i) mean += (float)gDraw.point(i) * (1.0f / 127.0f);
    mean *= 1.0f / (float)kDrawPoints;
  }
  const float inc = 1.0f / (float)kCycle;
  float coef[kSquarePoly];
  if (wave == WAVE_SIN) squareUp(macro, inc, coef);
  bool shaped = macro > 0.0f;
  bool fin = wave == WAVE_TRI;
  float a = shaped ? onePole(inc, fin ? finCutoff(macro) : softCutoff(macro)) : 1.0f;
  float g = !shaped ? 1.0f : fin ? finGain(macro) : softGain(macro);
  float z = 0.0f;
  for (int rep = 0; rep < 4; ++rep)  // a few cycles, so the filter has settled by the last
    for (int i = 0; i < kCycle; ++i) {
      float ph = (float)i * inc, x;
      switch (wave) {
        case WAVE_SAW: x = 2.0f * ph - 1.0f; break;
        case WAVE_TRI: x = ph < 0.25f ? 4.0f * ph : (ph < 0.75f ? 2.0f - 4.0f * ph : 4.0f * ph - 4.0f); break;
        case WAVE_DRAW: x = drawnAt(gDraw.points(), ph) - mean; break;
        default: {
          float s = tabRead(gSine, ph);
          x = shaped ? squareUpAt(coef, s) : s;
          break;
        }
      }
      float y = x;
      if (shaped && wave != WAVE_SIN) {
        z += a * (x - z);
        y = (fin ? x - z : z) * g;
      }
      gCycle[osc][i] = y;
    }
  return gCycle[osc];
}
// How many times oscillator 2 goes round for one cycle of oscillator 1.
float oscRatio() { return exp2f_(octaveOf(1) - octaveOf(0) + mapDetuneCents(gParams.norm(P_OSC2_DETUNE)) * (1.0f / 1200.0f)); }

// One point of an oscillator's wave as it is set now, whichever engine it
// runs. `ph` counts cycles; a pluck is one event spread over 0..1.
float oscShape(int osc, float ph) {
  int engine = engineOf(osc);
  int wave = waveOf(osc);
  float macro = osc == 0 ? gParams.norm(P_OSC1_MACRO) : gParams.frac(P_OSC2_MACRO);
  float color = gParams.frac(osc == 0 ? P_OSC1_COLOR : P_OSC2_COLOR);
  float cyc = frac01(ph);
  switch (engine) {
    case ENG_VA:
      if (wave == WAVE_SUPER) {  // a few of the seven saws, drifting apart as the detune goes up
        float spread = 0.05f + 0.95f * macro * macro;
        float p2 = frac01(ph * (1.0f + 0.11f * spread) + 0.31f), p3 = frac01(ph * (1.0f - 0.11f * spread) + 0.67f);
        return ((2.0f * cyc - 1.0f) + 0.6f * (2.0f * p2 - 1.0f) + 0.6f * (2.0f * p3 - 1.0f)) * 0.5f;
      }
      if (wave == WAVE_SQR) return waveShape(wave, cyc, mapPulseWidth(macro));
      return shapedCycle(osc, wave, macro)[(int)(cyc * (float)kCycle) & (kCycle - 1)];
    case ENG_FM: {
      static const float kRatios[WAVE_COUNT] = {1.0f, 2.0f, 3.0f, 3.5f, 4.0f, 0.5f};
      // Feedback bends the modulator with its own output; two rounds of that are close enough to draw.
      float th = kTwoPi * ph * kRatios[wave], fb = color * 1.25f;
      float m = sinf_(th + fb * sinf_(th + fb * sinf_(th)));
      return sinf_(kTwoPi * ph + macro * macro * 8.0f * m);
    }
    case ENG_WAVETABLE: {
      float pos = macro * (float)(kWtFrames - 1);
      int f0 = pos >= (float)(kWtFrames - 1) ? kWtFrames - 2 : (int)pos;
      int i = (int)(cyc * (float)kWtLen) & (kWtLen - 1);
      const int16_t* fa = wave == WAVE_DRAW ? gDraw.live().wt[f0][0] : gWt[wave][f0][0];  // table 6 is the drawing
      const int16_t* fb = wave == WAVE_DRAW ? gDraw.live().wt[f0 + 1][0] : gWt[wave][f0 + 1][0];
      float a = (float)fa[i], b = (float)fb[i];
      return (a + (b - a) * (pos - (float)f0)) * (1.0f / 32000.0f);
    }
    default: {  // pluck: a wave that dies away, faster and duller with more damping
      float t = clampf(ph, 0.0f, 1.0f);
      float decay = exp2f_(-t * (5.0f - 4.0f * color));
      float th = kTwoPi * t * 4.0f * exp2f_(octaveOf(osc));
      float body = sinf_(th) * decay * (1.0f - 0.5f * macro);
      // What sets the string off is drawn at the start, where it is heard: a
      // burst of hiss, a softer one, a short snap, a ramp, or a thin bright pick.
      uint32_t h = (uint32_t)(t * 3000.0f) * 2654435761u;
      h ^= h >> 15;
      h *= 2246822519u;
      float hiss = (float)((h >> 12) & 0xFFFF) * (2.0f / 65535.0f) - 1.0f;
      float burst = exp2f_(-t * 40.0f);
      switch (wave) {
        case 0: return body + 0.5f * hiss * burst;
        case 1: return body + 0.2f * hiss * exp2f_(-t * 16.0f);
        case 2: return body + (t < 0.03f ? 0.9f * hiss : 0.0f);
        case 3: return body + 0.5f * (2.0f * frac01(t * 14.0f) - 1.0f) * burst;
        case WAVE_DRAW: return body + 0.5f * drawnAt(gDraw.points(), t * 14.0f) * burst;  // struck with the drawing
        default: return body + 0.35f * sinf_(th * 6.0f) * burst;
      }
    }
  }
}

// Plots `fn(cycles)` across a box: `span` cycles from left to right.
template <typename F>
void plotWave(const Box& b, float span, Colour c, F fn, float scale = 1.0f) {
  int mid = b.y + b.h / 2, amp = b.h / 2 - 6;
  int py = 0;
  for (int px = 0; px < b.w; ++px) {
    float v = fn((float)px * span / (float)b.w) * scale;
    int yy = clampi(mid - (int)(v * (float)amp), b.y, b.y + b.h - 2);
    plotColumn(b.x + px, px > 0 ? py : yy, yy, c);
    py = yy;
  }
}

void midLine(const Box& b) { gfxHLine(b.x, b.y + b.h / 2, b.w, kGrid); }

// An oscillator's wave. A higher octave shows as more cycles in the same width.
void drawOscWave(int osc, const Box& b = kPic) {
  midLine(b);
  bool once = engineOf(osc) == ENG_PLUCK;  // a pluck is one event, not a repeating wave
  float span = once ? 1.0f : (isSuper(osc) ? 4.0f : 2.0f) * exp2f_(octaveOf(osc));
  plotWave(b, span, kLine, [osc](float t) { return oscShape(osc, t); });
}

// Shown over an oscillator's picture when it plays the drawing and the drawing is flat.
void drawFlatHint(int osc, const Box& b) {
  if (playsDrawing(osc) && engineOf(osc) == ENG_VA && gDraw.silent()) hint(b.y + 2, "THE DRAWING IS FLAT: SILENT");
}

// Detune: two oscillators a little apart beat against each other. The
// picture is one second of the pair at A3: the swell and fade of the beats.
void drawBeats(float cents, const char* inTune, const Box& b = kPic) {
  float beats = absf(220.0f * (exp2f_(cents * (1.0f / 1200.0f)) - 1.0f));
  int mid = b.y + b.h / 2 + 6, amp = b.h / 2 - 22;
  int pyT = 0, pyB = 0;
  for (int px = 0; px < b.w; ++px) {
    float env = absf(cosf_(kPi * beats * (float)px / (float)b.w));
    int e = (int)(env * (float)amp);
    if ((px & 3) == 0) gfxVLine(b.x + px, mid - e, 2 * e + 1, kLineDim);
    int yT = mid - e, yB = mid + e;
    plotColumn(b.x + px, px > 0 ? pyT : yT, yT, kLine);
    plotColumn(b.x + px, px > 0 ? pyB : yB, yB, kLine);
    pyT = yT;
    pyB = yB;
  }
  Str s;
  if (beats < 0.05f) {
    s.add(inTune);
  } else {
    s.addFixed(beats, 1);
    s.add(" BEATS A SECOND");
  }
  hint(b.y + 2, s.b);
}

void drawDetune() { drawBeats(mapDetuneCents(gParams.norm(P_OSC2_DETUNE)), "IN TUNE WITH OSC 1"); }

// X-Mod: oscillator 1 as it is (dim) and as oscillator 2 bends it (bright).
void drawXmod(const Box& b = kPic) {
  midLine(b);
  float x = gParams.norm(P_XMOD);
  float amount = x * x * 0.6f, ratio = oscRatio();
  // A plucked string is not bent by X-Mod, and with oscillator 1's fader down there is nothing to hear.
  bool pluck = engineOf(0) == ENG_PLUCK, silent = gParams.norm(P_MIX_OSC1) < 0.0001f;
  if (pluck || silent) amount = 0.0f;
  plotWave(b, 2.0f, kLineDim, [](float t) { return oscShape(0, t); });
  plotWave(b, 2.0f, kLine, [amount, ratio](float t) { return oscShape(0, t + amount * oscShape(1, t * ratio)); });
  if (pluck) hint(b.y + 2, "NO EFFECT ON A PLUCK");
  else if (silent) hint(b.y + 2, "OSC 1 FADER IS DOWN");
}

// Sync: oscillator 2 over two cycles of oscillator 1. With Sync on it
// starts again at every cycle of oscillator 1 (the marks along the top).
void drawSync(const Box& b = kPic) {
  midLine(b);
  // A plucked string has no cycle to restart, so the engine leaves it alone.
  bool pluck = engineOf(1) == ENG_PLUCK;
  bool on = gParams.step(P_SYNC) != 0 && !pluck;
  float ratio = oscRatio();
  // Running at a whole multiple of oscillator 1 (the same pitch, an octave
  // up, two octaves up), oscillator 2 restarts where it would have anyway.
  float whole = floorf_(ratio + 0.5f);
  bool inStep = whole >= 1.0f && absf(ratio - whole) < 0.012f * whole;
  for (int k = 0; k < 2; ++k) {
    int x = b.x + k * b.w / 2;
    gfxVLine(x, b.y + 18, b.h - 18, kGrid);
    if (on) gfxFill(x, b.y + 18, 3, 12, kAccent);
  }
  plotWave(b, 2.0f, kLine, [on, ratio](float t) { return oscShape(1, on ? frac01(t) * ratio : t * ratio); }, 0.8f);
  const char* note = pluck ? "NO EFFECT ON A PLUCK" : !on ? "OSC 2 RUNS FREE" : inStep ? "IN STEP: DETUNE TO HEAR" : "RESTARTS WITH OSC 1";
  hint(b.y + 2, note);
}

// Mixer: the four levels, and the wave they add up to.
void drawMixer(int selected) {
  static const char* const kNames[4] = {"OSC 1", "OSC 2", "SUB", "NOISE"};
  float lvl[4];
  for (int i = 0; i < 4; ++i) {
    int id = P_MIX_OSC1 + i;
    lvl[i] = gParams.norm(id);
    int x = 30 + i * 72;
    int h = (int)(lvl[i] * 46.0f);
    gfxFrame(x, 54, 44, 50, kDarkGrey);
    gfxFill(x + 2, 102 - h, 40, h, id == selected ? kAccent : kLineDim);
    gfxTextCentre(x + 22, 108, kNames[i], id == selected ? kAccent : kGrey, 2);
  }
  const Box wave = {8, 130, 304, 84};
  midLine(wave);
  float ratio = oscRatio(), sub = mapSubRatio(gParams.step(P_SUB_OCTAVE), 0.0f);
  float span = 2.0f * exp2f_(octaveOf(0));
  plotWave(wave, span, kLine, [&](float t) {
    uint32_t h = (uint32_t)(t * 4096.0f) * 2654435761u;  // the same "noise" every time, so the picture holds still
    h ^= h >> 15;
    h *= 2246822519u;
    h ^= h >> 13;
    float noise = (float)(h & 0xFFFF) * (2.0f / 65535.0f) - 1.0f;
    return (lvl[0] * oscShape(0, t) + lvl[1] * oscShape(1, t * ratio) + lvl[2] * (frac01(t * sub) < 0.5f ? 1.0f : -1.0f) +
            lvl[3] * 0.7f * noise) * 0.5f;
  });
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
  bool mono = gParams.step(P_VOICE_MODE) == MODE_MONO;  // a lone mono voice stays in the middle
  float spread = mono ? 0.0f : gParams.norm(P_SPREAD);
  if (mono) hint(58, "MONO STAYS IN THE MIDDLE");
  gfxHLine(mid - half, y + 5, 2 * half + 1, kGrid);
  gfxVLine(mid, y - 30, 70, kGrid);
  for (int i = 0; i < kNumVoices; ++i) {
    int x = mid + (int)(kPan[i] * spread * (float)half);
    gfxFill(x - 5, y - 14 + (i % 4) * 9, 10, 8, gSynth.voiceActive(i) ? kAccent : kLine);
  }
  gfxText(mid - half, 190, "L", kGrey, 2);
  gfxTextRight(mid + half, 190, "R", kGrey, 2);
}

// ---- the filter pictures ----
// All share one frame: frequency left to right, 20 Hz to 20 kHz, level up and down.
// The frame is movable so the same curve can be drawn small on an encoder page.
Box gF = {8, 54, 304, 156};
constexpr float kFTopDb = 18.0f, kFRangeDb = 66.0f;

int freqX(float hz) { return gF.x + clampi((int)(log2f_(maxf(hz, 20.0f) / 20.0f) * 0.1f * (float)gF.w), 0, gF.w - 1); }

void drawFilterAxes() {
  int zero = gF.y + (int)(kFTopDb / kFRangeDb * (float)gF.h);
  gfxHLine(gF.x, zero, gF.w, kGrid);
  static const float kMarks[3] = {100.0f, 1000.0f, 10000.0f};
  static const char* const kMarkNames[3] = {"100", "1k", "10k"};
  for (int i = 0; i < 3; ++i) {
    int x = freqX(kMarks[i]);
    gfxVLine(x, gF.y, gF.h, kGrid);
    if (gF.h > 100) gfxTextCentre(x, gF.y + gF.h + 4, kMarkNames[i], kGrey, 2);
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
  for (int px = 0; px < gF.w; ++px) {
    float f = 20.0f * exp2f_(10.0f * (float)px / (float)(gF.w - 1));
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
    int yy = clampi(gF.y + (int)((kFTopDb - db) / kFRangeDb * (float)gF.h), gF.y, gF.y + gF.h - 2);
    plotColumn(gF.x + px, px > 0 ? py : yy, yy, c);
    py = yy;
  }
}

float cutoffNow() { return mapCutoffHz(gParams.norm(P_CUTOFF)); }
float clampCutoff(float hz) { return clampf(hz, 20.0f, 20480.0f); }

// Cutoff, Resonance, Type: the response as it is set right now.
void drawFilterCurve() {
  drawFilterAxes();
  plotFilter(cutoffNow(), kLine);
}

// Env amount: where the filter rests, where the envelope pushes it at its
// peak, and an arrow for the sweep between the two.
void drawFilterEnvAmount() {
  drawFilterAxes();
  float rest = cutoffNow();
  float oct = mapEnvAmtOct(gParams.norm(P_FILT_ENV_AMT));
  float peak = clampCutoff(rest * exp2f_(oct));
  if (oct == 0.0f) {
    plotFilter(rest, kLine);
    gfxTextCentre(160, gF.y + 2, "NO SWEEP", kGrey, 2);
    return;
  }
  plotFilter(rest, kLineDim);
  plotFilter(peak, kAccent);
  int x0 = freqX(rest), x1 = freqX(peak), y = gF.y + 6;
  if (x1 == x0) {  // the cutoff is already as far as it goes that way: nothing left to sweep
    gfxTextCentre(160, gF.y + 2, oct > 0.0f ? "CUTOFF ALREADY OPEN" : "CUTOFF ALREADY SHUT", kGrey, 2);
    return;
  }
  int dir = x1 > x0 ? 1 : -1;
  gfxFill(x0 < x1 ? x0 : x1, y, (x1 > x0 ? x1 - x0 : x0 - x1) + 1, 2, kAccent);
  for (int i = 1; i <= 5; ++i) gfxFill(x1 - dir * i, y - i, 2, 2 * i + 2, kAccent);  // arrow head at the peak end
  gfxFill(x0, y - 4, 2, 10, kAccent);                                               // a tick where it rests
}

// Keytrack: the response for a low, a middle and a high key. With no
// tracking the three sit on top of each other; with full tracking the
// filter moves two octaves for two octaves of keys.
void drawFilterKeytrack() {
  drawFilterAxes();
  float kt = gParams.norm(P_KEYTRACK);
  float mid = cutoffNow();
  float low = clampCutoff(mid * exp2f_(-2.0f * kt)), high = clampCutoff(mid * exp2f_(2.0f * kt));
  plotFilter(low, kLineDim);
  plotFilter(high, kLineDim);
  plotFilter(mid, kLine);
  int xl = freqX(low), xm = freqX(mid), xh = freqX(high);
  if (xh - xl < 60) {
    gfxTextCentre(160, gF.y + 2, kt < 0.02f ? "SAME FOR EVERY KEY" : "LOW AND HIGH KEYS", kGrey, 2);
    return;
  }
  gfxTextCentre(clampi(xl, 20, 300), gF.y + 2, "C2", kGrey, 2);
  gfxTextCentre(clampi(xh, 20, 300), gF.y + 2, "C6", kGrey, 2);
  if (xm - xl > 44 && xh - xm > 44) gfxTextCentre(xm, gF.y + 2, "C4", kLine, 2);
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
    plotColumn(x0 + px, px > 0 ? pyOut : yOut, yOut, kLine);
    pyOut = yOut;
  }
  gfxText(8, 204, "IN", kGrey, 2);
  gfxTextRight(312, 204, "OUT", kLine, 2);
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
    Colour c = (firstParam + which == selected) ? kAccent : kLine;
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
    gfxTextCentre(lx + widths[i] / 2, bottom + 8, kNames[i], firstParam + i == selected ? kAccent : kGrey, 2);
    lx += widths[i];
  }
}

// LFO: its shape, with more cycles as the rate goes up and its height
// following the depth. The dim copy is the shape at full depth, so Shape
// and Rate still show while Depth is at zero.
void drawLfo() {
  static const float kRandom[8] = {0.3f, -0.7f, 0.9f, -0.2f, 0.6f, -0.9f, 0.1f, -0.5f};
  int shape = gParams.step(P_LFO_SHAPE);
  const Box b = {10, 62, 300, 124};
  midLine(b);
  float cycles = 0.5f * exp2f_(gParams.norm(P_LFO_RATE) * 5.0f);  // half a cycle at the slowest, sixteen at the fastest
  float depth = gParams.norm(P_LFO_DEPTH);
  auto wave = [shape](float t) {
    float ph = frac01(t);
    switch (shape) {
      case LFOS_SINE: return tabRead(gSine, ph);
      case LFOS_TRI: return 1.0f - 4.0f * absf(ph - 0.5f);
      case LFOS_SAW: return 1.0f - 2.0f * ph;
      case LFOS_SQUARE: return ph < 0.5f ? 1.0f : -1.0f;
      default: return kRandom[(int)(t * 2.0f) & 7];
    }
  };
  if (depth < 0.98f) plotWave(b, cycles, kLineDim, wave);
  plotWave(b, cycles, kLine, wave, depth);
  Str s;
  s.add("TO ");
  s.add(kParams[P_LFO_DEST].labels[gParams.step(P_LFO_DEST)]);
  gfxText(8, 200, s.b, kGrey, 2);
  Str r;
  formatParam(P_LFO_RATE, r);
  gfxTextRight(312, 200, r.b, kGrey, 2);
}

// Glide: the pitch from one key to the next. The longer the glide, the longer the slope.
void drawGlide(const Box& b = kPic) {
  float n = gParams.norm(P_GLIDE);
  bool off = mapGlideTime(n) <= 0.0f;
  int yLow = b.y + b.h * 3 / 4, yHigh = b.y + b.h / 4;
  int x0 = b.x + 56, slope = off ? 0 : 6 + (int)(n * (float)(b.w - 110));
  gfxPlotLine(b.x, yLow, x0, yLow, kLine);
  int py = yLow;
  for (int px = x0; px < b.x + b.w; ++px) {
    float done = slope > 0 ? 1.0f - exp2f_(-7.0f * (float)(px - x0) / (float)slope) : 1.0f;
    int yy = yLow - (int)(done * (float)(yLow - yHigh));
    plotColumn(px, py, yy, kLine);
    py = yy;
  }
  gfxVLine(x0, b.y + 20, b.h - 20, kGrid);
  gfxText(b.x, yLow + 8, "KEY 1", kGrey, 2);
  gfxTextRight(b.x + b.w, yHigh - 22, "KEY 2", kGrey, 2);
}

// Voice mode: three keys pressed one after another, and what sounds.
void drawVoiceMode() {
  int mode = gParams.step(P_VOICE_MODE);
  static const int kKeyX[3] = {30, 110, 190};
  static const int kKeyY[3] = {150, 118, 86};
  int fan = 2 + gParams.step(P_UNI_DETUNE) / 8;  // unison: how far the eight voices are spread
  for (int k = 0; k < 3; ++k) {
    gfxVLine(kKeyX[k], 62, 110, kGrid);
    int xEnd = mode == MODE_POLY || k == 2 ? 306 : kKeyX[k + 1];  // in mono and unison the next key takes over
    if (mode == MODE_UNI) {
      for (int v = 0; v < 8; ++v) gfxHLine(kKeyX[k], kKeyY[k] + (v - 4) * fan / 2 + (v >= 4), xEnd - kKeyX[k], v == 3 || v == 4 ? kLine : kLineDim);
    } else {
      gfxFill(kKeyX[k], kKeyY[k] - 1, xEnd - kKeyX[k], 3, kLine);
    }
    if (mode != MODE_POLY && k < 2) gfxVLine(xEnd, kKeyY[k + 1], kKeyY[k] - kKeyY[k + 1], kLineDim);
  }
  static const char* const kSays[3] = {"EVERY KEY ITS OWN VOICE", "ONE VOICE, LAST KEY WINS", "EIGHT VOICES ON ONE KEY"};
  hint(56, kSays[mode < 3 ? mode : 0]);
  drawSteps(kParams[P_VOICE_MODE], mode, 184);
}

// Drift: the pitch of each of the eight voices over a few seconds. At zero
// they are one line; turned up they sit apart and wander.
void drawDrift(const Box& b = kPic) {
  static const float kOff[kNumVoices] = {-1.0f, 1.0f, -0.6f, 0.6f, -0.3f, 0.3f, -0.8f, 0.8f};
  float drift = gParams.norm(P_DRIFT);
  int mid = b.y + b.h / 2 + 6;
  gfxHLine(b.x, mid, b.w, kGrid);
  for (int v = 0; v < kNumVoices; ++v) {
    int py = 0;
    for (int px = 0; px < b.w; ++px) {
      float t = (float)px / (float)b.w;
      float wander = 0.6f * sinf_(kTwoPi * (t * 1.3f + 0.137f * (float)v)) + 0.4f * sinf_(kTwoPi * (t * 3.1f + 0.291f * (float)v));
      int yy = mid - (int)(drift * (kOff[v] * 16.0f + wander * 40.0f));
      plotColumn(b.x + px, px > 0 ? py : yy, yy, v == 0 ? kLine : kLineDim);
      py = yy;
    }
  }
  hint(b.y + 2, drift < 0.004f ? "EVERY VOICE IN TUNE" : "EACH VOICE WANDERS");
}

// How much comes out (up) against how hard the key is hit (across). The
// engine squares the key's velocity, so a middling hit gives a quarter, not
// a half; the curve shows that. Used for Velocity and for velocity to cutoff.
void drawVelocityLine(float atSoft, float atHard, const Box& b = kPic) {
  int x0 = b.x + 14, x1 = b.x + b.w - 14, base = b.y + b.h - 22, top = b.y + 22;
  bool small = b.h < 100;
  if (small) {
    base = b.y + b.h - 4;
    top = b.y + 16;
  }
  gfxHLine(b.x, base, b.w, kGrid);
  auto yAt = [&](float hit) { return base - (int)((atSoft + (atHard - atSoft) * hit * hit) * (float)(base - top)); };
  for (int k = 0; k <= 6; ++k) {  // seven keystrokes, soft to hard
    int x = x0 + (x1 - x0) * k / 6, y = yAt((float)k / 6.0f);
    gfxFill(x - 2, y, 5, base - y, kLineDim);
  }
  int py = yAt(0.0f);
  for (int x = x0; x <= x1; ++x) {
    int yy = yAt((float)(x - x0) / (float)(x1 - x0));
    plotColumn(x, py, yy, kLine);
    py = yy;
  }
  gfxText(b.x, small ? b.y : base + 6, "SOFT", kGrey, 2);
  gfxTextRight(b.x + b.w, small ? b.y : base + 6, "HARD", kGrey, 2);
}

// Volume: a plain wave, as tall as the output is loud.
void drawLevelWave(float level, const Box& b = kPic) {
  midLine(b);
  plotWave(b, 3.0f, kLine, [](float t) { return tabRead(gSine, frac01(t)); }, level);
}

// The arpeggiator: two seconds of what it plays from a three-note chord.
// Mode sets the order, Octaves the height, Tempo how many steps fit, Gate
// how long each one lasts. With the arp off the pattern is drawn dim.
void drawArp(int id, const Box& b = kPic) {
  int mode = gParams.step(P_ARP_MODE), octs = gParams.step(P_ARP_OCTAVES) + 1;
  int clock = gSynth.clockState();
  // following a clock from outside, the steps are as wide as that clock makes them
  float bpm = clock == CLK_LOCKED ? gSynth.tempoBpm() : mapTempoBpm(gParams.norm(P_ARP_TEMPO));
  float gate = mapArpGate(gParams.norm(P_ARP_GATE));
  static const int kChord[3] = {0, 4, 7}, kPlayed[3] = {4, 0, 7};
  int shown = mode == ARP_OFF ? ARP_UP : mode;
  int seq[24], n = 0;
  for (int o = 0; o < octs; ++o)
    for (int k = 0; k < 3; ++k) seq[n++] = (shown == ARP_PLAYED ? kPlayed[k] : kChord[k]) + 12 * o;
  if (shown == ARP_DOWN)
    for (int i = 0; i < n / 2; ++i) {
      int t = seq[i];
      seq[i] = seq[n - 1 - i];
      seq[n - 1 - i] = t;
    }
  int len = n;
  if (shown == ARP_UPDOWN)
    for (int i = n - 2; i >= 1; --i) seq[len++] = seq[i];  // back down without repeating the ends
  int top = 12 * (octs - 1) + 7;
  float stepW = (float)b.w * 15.0f / (2.0f * bpm);  // sixteenth notes in a two-second window
  uint32_t rng = 0x2F6E2B1u;
  Colour c = mode == ARP_OFF ? kLineDim : kLine;
  for (int i = 0;; ++i) {
    int x = b.x + (int)((float)i * stepW);
    if (x >= b.x + b.w) break;
    rng = rng * 1664525u + 1013904223u;
    int note = shown == ARP_RANDOM ? seq[(rng >> 16) % (uint32_t)n] : seq[i % len];
    int y = b.y + b.h - 10 - note * (b.h - 44) / top;
    int w = clampi((int)(stepW * gate), 2, b.x + b.w - x);
    gfxFill(x, y, w, 4, c);
  }
  if (mode == ARP_OFF) hint(b.y + 2, "ARP IS OFF");
  else if (clock == CLK_STOPPED) hint(b.y + 2, "CLOCK STOPPED: ARP WAITS");
  else if (id == P_ARP_TEMPO && clock == CLK_LOCKED) hint(b.y + 2, "THE CLOCK SETS THE TEMPO");
  else if (id == P_ARP_TEMPO && clock == CLK_WAITING) hint(b.y + 2, "NO CLOCK: THE KNOB IS USED");
}

// ---- pictures for the values on the encoder pages ----

// Unison detune: the eight voices fanning apart.
void drawUnison(const Box& b) {
  static const float kOff[kNumVoices] = {-1.0f, 1.0f, -0.71f, 0.71f, -0.43f, 0.43f, -0.14f, 0.14f};
  float spread = (float)gParams.step(P_UNI_DETUNE) / 50.0f;
  int mid = b.y + b.h / 2;
  for (int v = 0; v < kNumVoices; ++v)
    gfxPlotLine(b.x + 20, mid, b.x + b.w - 4, mid - (int)(kOff[v] * spread * (float)(b.h / 2 - 4)), v < 2 ? kLine : kLineDim);
}

// Sub pitch: oscillator 1 (dim) and the sub's square against it: two
// octaves or one below, a fifth above, or an octave above.
void drawSub(const Box& b) {
  midLine(b);
  float sub = mapSubRatio(gParams.step(P_SUB_OCTAVE), 0.0f);
  plotWave(b, 4.0f, kLineDim, [](float t) { return oscShape(0, t) * 0.6f; });
  plotWave(b, 4.0f, kLine, [sub](float t) { return frac01(t * sub) < 0.5f ? 0.9f : -0.9f; });
}

// Sub detune: the beating it makes against oscillator 1.
void drawSubDetune(const Box& b) {
  drawBeats((float)(gParams.step(P_SUB_DETUNE) + kParams[P_SUB_DETUNE].intMin), "SUB IS IN TUNE", b);
}

// Compressor: what comes out (up) against what goes in (across). The dim
// line is no compression; the bright one bends over above the threshold
// and sits higher at low levels, where the quiet parts are being lifted.
void drawCompress(const Box& b) {
  float amount = gParams.frac(P_COMPRESS);
  CompSettings c = mapCompress(amount);
  int base = b.y + b.h - 2, top = b.y + 4, x0 = b.x + 40, w = b.w - 80;
  gfxHLine(b.x, base, b.w, kGrid);
  gfxLine(x0, base, x0 + w, top, kLineDim);
  int py = base;
  for (int px = 0; px <= w; ++px) {
    float in = (float)px / (float)w;
    float out = minf(in * compressGain(c, in), 1.0f);
    int yy = base - (int)(out * (float)(base - top));
    plotColumn(x0 + px, py, yy, kLine);
    py = yy;
  }
  if (amount < 0.005f) hint(b.y + 2, "COMPRESSOR IS OFF");
}

// Duck: the level over two beats at the arp's Tempo. It dips just after
// each beat (the marks) and comes back before the next.
void drawDuck(const Box& b) {
  float depth = gParams.frac(P_DUCK);
  // the beat is the clock's while one is followed, otherwise the Tempo knob's
  float beatSec = 60.0f / (gSynth.clockState() == CLK_LOCKED ? gSynth.tempoBpm() : mapTempoBpm(gParams.norm(P_ARP_TEMPO)));
  int base = b.y + b.h - 2, top = b.y + 14;
  gfxHLine(b.x, base, b.w, kGrid);
  for (int k = 0; k < 2; ++k) gfxFill(b.x + k * b.w / 2, b.y, 3, 10, kAccent);
  int py = top;
  for (int px = 0; px < b.w; ++px) {
    float level = 1.0f - 0.92f * depth * duckShape(frac01((float)px * 2.0f / (float)b.w), beatSec);
    int yy = base - (int)(level * (float)(base - top));
    plotColumn(b.x + px, px > 0 ? py : yy, yy, kLine);
    py = yy;
  }
  if (depth < 0.005f) hint(b.y + 2, "DUCK IS OFF");
}

// Bend range: how far the pitch goes with the wheel pushed right up, and pulled right down.
void drawBend(const Box& b) {
  midLine(b);
  float reach = (float)(gParams.step(P_BEND_RANGE) + 1) / 12.0f;
  auto push = [](float t) {
    float s = sinf_(kPi * clampf((t - 0.15f) / 0.7f, 0.0f, 1.0f));
    return s * s;
  };
  plotWave(b, 1.0f, kLineDim, [&](float t) { return -push(t) * reach; });
  plotWave(b, 1.0f, kLine, [&](float t) { return push(t) * reach; });
}

// Delay: the note and its repeats. Time sets the gaps, Feedback how slowly
// they fade, Mix their height, Tone how blunt each repeat becomes.
void drawDelay(const Box& b) {
  float time = mapDelayTime(gParams.frac(P_DLY_TIME)), tone = gParams.frac(P_DLY_TONE);
  float fdbk = 0.9f * gParams.frac(P_DLY_FDBK), mix = gParams.frac(P_DLY_MIX);
  int base = b.y + b.h - 2, full = b.h - 6;
  gfxHLine(b.x, base, b.w, kGrid);
  gfxFill(b.x + 6, base - full, 4, full, kInk);  // the note itself
  float gap = time * (float)(b.w - 12) / 2.25f;   // the window is two and a quarter seconds
  float level = mix;
  for (int k = 1; k < 40 && level > 0.01f; ++k) {
    int x = b.x + 8 + (int)((float)k * gap);
    if (x >= b.x + b.w - 2) break;
    int h = (int)(level * (float)full);
    int half = 1 + (int)((1.0f - tone) * (float)k * 1.5f);  // duller repeats spread out
    for (int d = -half; d <= half; ++d) {
      int hh = h * (half + 1 - (d < 0 ? -d : d)) / (half + 1);
      if (x + d < b.x + b.w) gfxVLine(x + d, base - hh, hh, d == 0 ? kLine : kLineDim);
    }
    level *= fdbk;
  }
  if (mix < 0.005f) hint(b.y + 2, "DELAY IS OFF: MIX IS 0");
}

// Chorus: the two copies wobbling against each other. Ensemble: three,
// each a third of a cycle apart, with a quick shiver on top of the slow sweep.
void drawChorus(const Box& b) {
  float amount = gParams.norm(P_CHORUS);
  midLine(b);
  if (gParams.step(P_CHORUS_TYPE) == CHORUS_ENSEMBLE) {
    for (int k = 2; k >= 0; --k) {
      float third = (float)k * (1.0f / 3.0f);
      plotWave(b, 2.0f, k == 0 ? kLine : kLineDim, [amount, third, k](float t) {
        return 0.3f * amount * (float)(1 - k) +
               amount * (0.5f * tabRead(gSine, frac01(t + third)) + 0.14f * tabRead(gSine, frac01(t * 10.0f + third)));
      });
    }
  } else {
    plotWave(b, 2.0f, kLine, [amount](float t) { return 0.3f * amount + 0.6f * amount * tabRead(gSine, frac01(t)); });
    plotWave(b, 2.0f, kLineDim, [amount](float t) { return -0.3f * amount + 0.6f * amount * tabRead(gSine, frac01(t + 0.25f)); });
  }
  if (amount < 0.005f) hint(b.y + 2, "CHORUS IS OFF");
}

// Reverb: how long the tail takes to die away. The slope reaches the floor
// at the reverb time (eight seconds at the right-hand edge, with the short
// times stretched out); Mix sets where it starts; the dim line is the high
// end, which Tone lets ring or cuts short.
void drawReverb(const Box& b) {
  float time = mapReverbTime(gParams.frac(P_RVB_SIZE)), tone = gParams.frac(P_RVB_TONE), mix = gParams.frac(P_RVB_MIX);
  int base = b.y + b.h - 2, full = b.h - 8;
  gfxHLine(b.x, base, b.w, kGrid);
  int start = base - (int)(mix * (float)full);
  int reach = (int)(sqrtf_(time / 8.0f) * (float)b.w), reachHigh = (int)((float)reach * (0.25f + 0.75f * tone));
  gfxPlotLine(b.x, start, b.x + clampi(reachHigh, 1, b.w - 2), base, kLineDim);
  gfxPlotLine(b.x, start, b.x + clampi(reach, 1, b.w - 2), base, kLine);
  if (mix < 0.005f) hint(b.y + 2, "REVERB IS OFF: MIX IS 0");
}

// LFO fade: the LFO growing to full strength after a key is pressed (the window is five seconds).
void drawLfoFade(const Box& b) {
  midLine(b);
  float fade = mapFadeTime(gParams.frac(P_LFO_FADE));
  plotWave(b, 12.0f, kLine, [fade](float t) {
    float sec = t * (5.0f / 12.0f);
    float env = fade <= 0.0f ? 1.0f : clampf(sec / fade, 0.0f, 1.0f);
    return env * tabRead(gSine, frac01(t));
  });
}

// Envelope to pitch: the pitch rises with the filter envelope and settles
// wherever that envelope's Sustain holds it, until the key is let go.
void drawEnvPitch(const Box& b) {
  int base = b.y + b.h - 6;
  gfxHLine(b.x, base, b.w, kGrid);
  float semis = (float)gParams.step(P_FENV_PITCH) / 24.0f;
  int py = 0;
  for (int px = 0; px < b.w; ++px) {
    float t = (float)px / (float)b.w;
    float sus = gParams.norm(P_FENV_S);
    float env = t < 0.04f ? t / 0.04f : t < 0.72f ? sus + (1.0f - sus) * exp2f_(-12.0f * (t - 0.04f)) : sus * exp2f_(-22.0f * (t - 0.72f));
    int yy = base - (int)(env * semis * (float)(b.h - 12));
    plotColumn(b.x + px, px > 0 ? py : yy, yy, kLine);
    py = yy;
  }
}

// LFO sync. Three keys, drawn as bars along the top: the second is played
// while the first is still down, the third after a silence. FREE: one LFO
// that takes no notice of the keys. KEY: it starts again when a note begins
// from silence (the first and the third), not for a note that joins others
// already sounding. VOICE: every voice has its own LFO, each a little out
// of step and a little off the others' pace, so a chord shimmers instead
// of moving as one.
void drawLfoSync(const Box& b) {
  midLine(b);
  int mode = gParams.step(P_LFO_SYNC);
  if (mode == LFOSYNC_VOICE) {
    static const float kRate[4] = {1.0f, 0.94f, 1.07f, 0.97f};
    for (int v = 3; v >= 0; --v)
      plotWave(b, 1.0f, v == 0 ? kLine : kLineDim, [v](float t) { return 0.75f * tabRead(gSine, frac01(t * 5.3f * kRate[v] + 0.27f * (float)v)); });
    hint(b.y + 2, "ONE LFO FOR EACH VOICE");
    return;
  }
  bool key = mode == LFOSYNC_KEY;
  static const float kDown[3] = {0.08f, 0.30f, 0.70f}, kUp[3] = {0.46f, 0.56f, 0.94f};
  for (int k = 0; k < 3; ++k) {
    int x0 = b.x + (int)(kDown[k] * (float)b.w), x1 = b.x + (int)(kUp[k] * (float)b.w);
    gfxFill(x0, b.y + (k == 1 ? 5 : 0), x1 - x0, 3, kAccent);
    if (k != 1) gfxVLine(x0, b.y, b.h, kGrid);
  }
  plotWave(b, 1.0f, kLine, [key](float t) {
    float since = t;
    if (key) since = t >= kDown[2] ? t - kDown[2] : t >= kDown[0] ? t - kDown[0] : t;
    return 0.75f * tabRead(gSine, frac01(since * 5.3f));
  });
}

// The filter's curve, drawn in a box of any size.
void drawFilterIn(const Box& b) {
  Box keep = gF;
  gF = b;
  drawFilterCurve();
  gF = keep;
}

// ---- the vocoder's pictures ----

// Says why the vocoder cannot be heard, if it cannot. True if it said something.
bool vocoderSilent(const Box& b) {
  if (gParams.step(P_VOCODER) == 0) hint(b.y + 2, "VOCODER IS OFF");
  else if (gSynth.inputLevel() < 0.003f) hint(b.y + 2, "NO SIGNAL AT THE INPUT");
  else if (gSynth.activeVoices() == 0) hint(b.y + 2, "PLAY A NOTE TO HEAR IT");
  else return false;
  return true;
}

// The level of each of the voice's sixteen bands, live: low on the left.
// The column at the right is the amount: how much of the synth's sound the
// vocoded sound has replaced.
void drawVocoder(const Box& b) {
  int base = b.y + b.h - 2, top = b.y + 20, w = (b.w - 20) / kVocBands;
  gfxHLine(b.x, base, b.w, kGrid);
  for (int k = 0; k < kVocBands; ++k) {
    int h = (int)(gVocoder.band(k) * (float)(base - top));
    gfxFill(b.x + k * w + 2, base - (h > 2 ? h : 2), w - 4, h > 2 ? h : 2, h > 2 ? kLine : kDarkGrey);
  }
  int amount = (int)(gParams.frac(P_VOCODER) * (float)(base - top - 2));
  gfxFrame(b.x + b.w - 12, top, 12, base - top, kDarkGrey);
  gfxFill(b.x + b.w - 11, base - 1 - amount, 10, amount, kAccent);
  vocoderSilent(b);
}

// Formant: where each band of the voice (top row) lands in the synth's sound (bottom row).
void drawFormant(const Box& b) {
  float shift = exp2f_((float)(gParams.step(P_VOC_FORMANT) + kParams[P_VOC_FORMANT].intMin) * (1.0f / 12.0f));
  int x0 = b.x + 70, w = b.w - 74, yTop = b.y + 26, yBot = b.y + b.h - 8;
  auto xOf = [&](float hz) { return x0 + (int)(clampf(log2f_(hz * (1.0f / 60.0f)) * (1.0f / 8.2f), 0.0f, 1.0f) * (float)w); };
  gfxText(b.x, yTop - 6, "VOICE", kLineDim, 2);
  gfxText(b.x, yBot - 8, "SYNTH", kAccent, 2);
  for (int k = 0; k < kVocBands; ++k) {
    int xv = xOf(vocBandHz(k)), xs = xOf(minf(vocBandHz(k) * shift, 0.45f * gSynth.sampleRate()));
    gfxLine(xv, yTop + 4, xs, yBot - 4, kDarkGrey);
    gfxFill(xv - 1, yTop - 4, 3, 8, kLine);
    gfxFill(xs - 1, yBot - 4, 3, 8, kAccent);
  }
  if (gParams.step(P_VOCODER) == 0) hint(b.y + 2, "VOCODER IS OFF");
}

// Consonants: the sixteen bands, with hiss in the top three as tall as the setting.
void drawConsonants(const Box& b) {
  float amount = gParams.frac(P_VOC_CONSONANTS);
  int base = b.y + b.h - 2, top = b.y + 22, w = b.w / kVocBands;
  gfxHLine(b.x, base, b.w, kGrid);
  uint32_t rng = 0x3C6EF35Fu;
  for (int k = 0; k < kVocBands; ++k) {
    int x = b.x + k * w + 2;
    if (k < kVocBands - kVocHissBands) {
      gfxFill(x, base - 3, w - 4, 3, kDarkGrey);
      continue;
    }
    for (int px = 0; px < w - 4; px += 2) {
      rng = rng * 1664525u + 1013904223u;
      int h = (int)(amount * (float)(base - top) * (0.45f + 0.55f * (float)(rng >> 24) * (1.0f / 255.0f)));
      gfxFill(x + px, base - (h > 3 ? h : 3), 2, h > 3 ? h : 3, h > 3 ? kLine : kDarkGrey);
    }
  }
  if (gParams.step(P_VOCODER) == 0) hint(b.y + 2, "VOCODER IS OFF");
  else if (amount < 0.005f) hint(b.y + 2, "NO HISS: S AND T ARE LOST");
}

// Smooth: three short sounds from the voice (blocks) and how a band's level follows them.
void drawSmooth(const Box& b) {
  float fallSec = mapVocRelease(gParams.frac(P_VOC_SMOOTH));
  int base = b.y + b.h - 2, top = b.y + 22;
  const float window = 0.8f;
  static const float kSound[3][2] = {{0.04f, 0.14f}, {0.30f, 0.40f}, {0.56f, 0.62f}};
  gfxHLine(b.x, base, b.w, kGrid);
  for (const float* sound : kSound)
    gfxFill(b.x + (int)(sound[0] / window * (float)b.w), top, (int)((sound[1] - sound[0]) / window * (float)b.w), base - top, kDarkGrey);
  float dt = window / (float)b.w, keep = exp2f_(-1.4427f * dt / fallSec), level = 0.0f;
  int py = base;
  for (int px = 0; px < b.w; ++px) {
    float t = (float)px * dt;
    bool sounding = false;
    for (const float* sound : kSound) sounding = sounding || (t >= sound[0] && t < sound[1]);
    level = sounding ? 1.0f : level * keep;
    int yy = base - (int)(level * (float)(base - top));
    plotColumn(b.x + px, px > 0 ? py : yy, yy, kLine);
    py = yy;
  }
  if (gParams.step(P_VOCODER) == 0) hint(b.y + 2, "VOCODER IS OFF");
}

// The picture for a value that lives on an encoder page. False if it has none.
bool drawSoftPicture(int id, const Box& b) {
  switch (id) {
    case P_VOCODER: drawVocoder(b); return true;
    case P_VOC_FORMANT: drawFormant(b); return true;
    case P_VOC_CONSONANTS: drawConsonants(b); return true;
    case P_VOC_SMOOTH: drawSmooth(b); return true;
    case P_UNI_DETUNE: drawUnison(b); return true;
    case P_SUB_OCTAVE: drawSub(b); return true;
    case P_SUB_DETUNE: drawSubDetune(b); return true;
    case P_COMPRESS: drawCompress(b); return true;
    case P_DUCK: drawDuck(b); return true;
    case P_BEND_RANGE: drawBend(b); return true;
    case P_FILT_SLOPE: drawFilterIn(b); return true;
    case P_DLY_TIME: case P_DLY_TONE: case P_DLY_FDBK: case P_DLY_MIX: drawDelay(b); return true;
    case P_CHORUS: case P_CHORUS_TYPE: drawChorus(b); return true;
    case P_RVB_SIZE: case P_RVB_TONE: case P_RVB_MIX: drawReverb(b); return true;
    case P_LFO_FADE: drawLfoFade(b); return true;
    case P_VEL_CUTOFF: drawVelocityLine(0.0f, gParams.frac(P_VEL_CUTOFF), b); return true;
    case P_FENV_PITCH: drawEnvPitch(b); return true;
    case P_LFO_SYNC: drawLfoSync(b); return true;
    case P_OSC1_COLOR:
    case P_OSC2_COLOR: {
      int osc = id == P_OSC1_COLOR ? 0 : 1;
      drawOscWave(osc, b);
      // FM feedback roughens the modulator: with no FM amount there is nothing for it to roughen
      float amount = osc == 0 ? gParams.norm(P_OSC1_MACRO) : gParams.frac(P_OSC2_MACRO);
      if (engineOf(osc) == ENG_FM && amount < 0.004f) hint(b.y + 2, "NO EFFECT: FM AMOUNT IS 0");
      return true;
    }
    case P_OSC2_MACRO:
      drawOscWave(1, b);
      drawFlatHint(1, b);
      return true;
    default: return false;
  }
}

}  // namespace

void Ui::init() {
  mode_ = MB_NONE;
  page_ = 0;
  popupParam_ = -1;
  popupMs_ = 0;
  popupPickup_ = false;
  softParam_ = -1;
  softMs_ = 0;
  for (int i = 0; i < 4; ++i) burst_[i] = 0;
  browse_ = 0;
  drawCursor_ = 0;
  drawPaint_ = false;
  drawShape_ = DRAWSHAPE_SAW;
  cursor_ = 0;
  armMs_ = 0;
  noteMs_ = 0;
  note_ = "";
  dirty_ = true;
}

void Ui::say(const char* text) {
  note_ = text;
  noteMs_ = kNoteMs;
  dirty_ = true;
}

void Ui::pickup(int id) {
  popupParam_ = (int16_t)id;
  popupPickup_ = true;
  popupMs_ = kPopupMs;
  dirty_ = true;
}

void Ui::patchLoaded() {
  browse_ = (int8_t)(gPatch.slot() >= 0 ? gPatch.slot() : 0);
  armMs_ = 0;
  popupParam_ = -1;
  popupMs_ = 0;
  say("LOADED");
}

// The Patch pages. Page 1: encoder 1 picks a slot and loads it when pushed,
// encoder 2 saves, encoder 3 sets what the label lights do, encoder 4 drops
// the patch and makes the panel live. Page 2 edits the name.
void Ui::patchTurn(int enc, int delta) {
  popupParam_ = -1;  // using the page brings it back in front of any control's picture
  popupMs_ = 0;
  if (page_ == 0) {
    if (enc == 0) {
      browse_ = (int8_t)clampi(browse_ + delta, 0, kPatchSlots - 1);
      armMs_ = 0;
      noteMs_ = 0;
    } else if (enc == 2) {
      gPatch.setGuide(gPatch.guide() + delta);
    }
  } else if (page_ == 1) {
    if (enc == 0) {
      cursor_ = (uint8_t)clampi((int)cursor_ + delta, 0, kPatchNameLen - 1);
    } else if (enc == 2) {
      int count = 0;
      while (kNameChars[count]) ++count;
      int i = (nameCharIndex(gPatch.name()[cursor_]) + delta) % count;
      gPatch.setNameChar(cursor_, kNameChars[i < 0 ? i + count : i]);
    }
  } else if (page_ >= kFirstSettingsPage) {  // MIDI and Input: one setting to an encoder
    int id = kSettingPages[page_ - kFirstSettingsPage][enc];
    if (id != kNone) {
      if (kSettings[id].count > 30) {  // a long range: a fast spin moves in bigger steps
        int n = burst_[enc] < 250 ? ++burst_[enc] : 250;
        if (n >= 4) delta *= 5;
        else if (n >= 2) delta *= 2;
      }
      settingSet(id, setting(id) + delta);
    }
  } else if (enc == 0) {  // Setup: the display's colours
    int now = gfxPalette(), next = clampi(now + delta, 0, PALETTE_COUNT - 1);
    if (next != now) {
      gfxSetPalette(next);
      stateTouched();
    }
  } else if (enc == 2) {  // Setup: what the label lights do
    lightsSetMode(lightsMode() + delta);
  }
  dirty_ = true;
}

void Ui::patchPush(int enc) {
  popupParam_ = -1;
  popupMs_ = 0;
  if (page_ == 0) {
    if (enc == 0) {
      armMs_ = 0;
      if (gPatch.load(browse_)) say("LOADED");
      else say("NOTHING STORED HERE");
    } else if (enc == 1) {
      if (gPatch.slotUsed(browse_) && armMs_ == 0) {  // never overwrite on a single push
        armMs_ = kArmMs;
        say("PUSH AGAIN TO OVERWRITE");
      } else {
        armMs_ = 0;
        gPatch.save(browse_);
        say("SAVED");
      }
    } else if (enc == 2) {
      gPatch.setGuide((gPatch.guide() + 1) % GUIDE_COUNT);
    } else if (enc == 3) {
      gPatch.manual();
      say("PANEL IS LIVE");
    }
  } else if (page_ == 1) {
    if (enc == 0) cursor_ = (uint8_t)((cursor_ + 1) % kPatchNameLen);  // push steps along the name
  } else if (page_ >= kFirstSettingsPage) {
    int id = kSettingPages[page_ - kFirstSettingsPage][enc];
    if (id != kNone) {  // a short list steps round; a long one goes back to where it started
      int count = kSettings[id].count;
      settingSet(id, count <= 3 ? (setting(id) + 1) % count : kSettings[id].def);
    }
  } else if (enc == 0) {
    gfxSetPalette((gfxPalette() + 1) % PALETTE_COUNT);
    stateTouched();
  } else if (enc == 2) {
    lightsSetMode((lightsMode() + 1) % LIGHTS_COUNT);
  }
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
      softParam_ = (int16_t)id;  // its picture takes the scope's place for a moment
      softMs_ = kPopupMs;
      dirty_ = true;
      return;
    }
  popupParam_ = (int16_t)id;
  popupPickup_ = false;
  popupMs_ = kPopupMs;
  dirty_ = true;
}

void Ui::modeButton(int which) {
  if (which < 0 || which >= MB_COUNT) return;
  if (mode_ != which) {
    mode_ = (int8_t)which;
    page_ = 0;
    if (which == MB_PATCH) {
      browse_ = (int8_t)(gPatch.slot() >= 0 ? gPatch.slot() : 0);
      armMs_ = 0;
      noteMs_ = 0;
    }
  } else if (page_ + 1 < pageCount(which)) {
    ++page_;  // the lit button steps to the mode's next page
  } else {
    mode_ = MB_NONE;  // and after the last page, home
    page_ = 0;
  }
  popupParam_ = -1;
  popupMs_ = 0;
  softMs_ = 0;
  dirty_ = true;
}

bool Ui::onDrawPage() const { return mode_ == MB_ENGINE && page_ == 2; }

void Ui::encoderTurn(int enc, int delta) {
  if (mode_ == MB_PATCH && enc >= 0 && enc < 4) {
    patchTurn(enc, delta);
    return;
  }
  if (onDrawPage() && enc >= 0 && enc < 4) {
    drawTurn(enc, delta);
    return;
  }
  int id = pageParam(enc);
  if (id < 0) return;
  popupParam_ = -1;  // the page comes back in front of any control's picture, so the value being turned is seen
  popupMs_ = 0;
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
  if (mode_ == MB_PATCH && enc >= 0 && enc < 4) {
    patchPush(enc);
    return;
  }
  if (onDrawPage() && enc >= 0 && enc < 4) {
    drawPush(enc);
    return;
  }
  int id = pageParam(enc);
  if (id < 0) return;
  if (gParams.set(id, kParams[id].def)) onParamChanged(id);  // push = back to default
}

bool Ui::tick(uint32_t dtMs) {
  for (int i = 0; i < 4; ++i) burst_[i] = 0;
  armMs_ = armMs_ > dtMs ? (uint16_t)(armMs_ - dtMs) : 0;
  noteMs_ = noteMs_ > dtMs ? (uint16_t)(noteMs_ - dtMs) : 0;
  softMs_ = softMs_ > dtMs ? (uint16_t)(softMs_ - dtMs) : 0;

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

// "07 BRASS" for a stored patch, "-- INIT" for a sound that has no slot yet.
static void patchTitle(Str& s, int slot, const char* name) {
  if (slot < 0) {
    s.add("--");
  } else {
    if (slot + 1 < 10) s.add('0');
    s.addInt(slot + 1);
  }
  s.add(' ');
  int len = kPatchNameLen;
  while (len > 0 && name[len - 1] == ' ') --len;
  for (int i = 0; i < len; ++i) s.add(name[i]);
}

void Ui::drawHome() {
  Str title;
  patchTitle(title, gPatch.slot(), gPatch.name());
  if (gPatch.edited()) title.add(" *");  // changed since it was loaded or saved
  gfxTextCentre(160, 10, title.b, kAccent, 2);
  drawScope(8, 40, 304, 168);
  drawStatus();
}

// The four corner captions of a Patch page, one per encoder.
static void drawCorners(const char* const label[4], const char* const value[4]) {
  static const int kLabelY[2] = {30, 164};
  for (int e = 0; e < 4; ++e) {
    int y = kLabelY[e & 1];
    if (e < 2) {
      gfxText(8, y, label[e], kLineDim, 2);
      gfxText(8, y + 16, value[e], kInk, 3);
    } else {
      gfxTextRight(312, y, label[e], kLineDim, 2);
      gfxTextRight(312, y + 16, value[e], kInk, 3);
    }
  }
}

void Ui::drawPatchPage() {
  gfxTextCentre(160, 6, "PATCH 1/5", kAccent, 2);
  Str slotNo;
  if (browse_ + 1 < 10) slotNo.add('0');
  slotNo.addInt(browse_ + 1);
  int following = gPatch.followCount();
  const char* const label[4] = {"SLOT/LOAD", "SAVE HERE", "GUIDE", "MANUAL"};
  const char* const value[4] = {slotNo.b, armMs_ > 0 ? "SURE?" : "PUSH", kGuideLabels[gPatch.guide()], following > 0 ? "PUSH" : "--"};
  drawCorners(label, value);

  bool used = gPatch.slotUsed(browse_);
  bool current = used && browse_ == gPatch.slot();
  Str big;
  if (used) patchTitle(big, browse_, gPatch.slotName(browse_));
  else {
    big.add(slotNo.b);
    big.add(" EMPTY");
  }
  gfxTextCentre(160, 80, big.b, current ? kAccent : (used ? kInk : kGrey), 3);

  const char* line = noteMs_ > 0 ? note_ : !used ? "PUSH SAVE TO STORE HERE" : !current ? "PUSH SLOT TO LOAD" : gPatch.edited() ? "LOADED, CHANGED SINCE" : "LOADED";
  gfxTextCentre(160, 112, line, noteMs_ > 0 ? kAccent : kGrey, 2);
  Str f;
  if (following > 0) {
    f.addInt(following);
    f.add(following == 1 ? " CONTROL FOLLOWS PATCH" : " CONTROLS FOLLOW PATCH");
  } else {
    f.add("EVERY CONTROL IS LIVE");
  }
  gfxTextCentre(160, 134, f.b, following > 0 ? kLine : kLineDim, 2);
  drawStatus();
}

void Ui::drawNamePage() {
  gfxTextCentre(160, 6, "NAME 2/5", kAccent, 2);
  Str pos;
  pos.addInt(cursor_ + 1);
  char c = gPatch.name()[cursor_];
  char letter[2] = {c == ' ' ? '_' : c, 0};
  const char* const label[4] = {"CURSOR", "--", "LETTER", "--"};
  const char* const value[4] = {pos.b, "", letter, ""};
  drawCorners(label, value);

  const int scale = 4, pitch = 6 * scale, x0 = 160 - kPatchNameLen * pitch / 2, y = 88;
  for (int i = 0; i < kPatchNameLen; ++i) {
    char one[2] = {gPatch.name()[i], 0};
    gfxText(x0 + i * pitch, y, one, i == cursor_ ? kAccent : kInk, scale);
    gfxFill(x0 + i * pitch, y + 8 * scale + 2, 5 * scale, i == cursor_ ? 3 : 1, i == cursor_ ? kAccent : kDarkGrey);
  }
  gfxTextCentre(160, 140, "SAVE ON PAGE 1 TO KEEP IT", kGrey, 2);
  drawStatus();
}

// Setup: things about the instrument itself, not the sound.
void Ui::drawSetupPage() {
  gfxTextCentre(160, 6, "SETUP 3/5", kAccent, 2);
  const char* const label[4] = {"DISPLAY", "--", "LIGHTS", "--"};
  const char* const value[4] = {kPaletteNames[gfxPalette()], "", kLightsNames[lightsMode()], ""};
  drawCorners(label, value);
  gfxTextCentre(160, 138, lightsMode() == LIGHTS_DANCE ? "LABELS FOLLOW THE SOUND" : "LABELS STAY PLAIN", kGrey, 2);
  // every colour in use, so the two sets can be judged side by side with the panel
  static const Colour kSwatch[6] = {kInk, kAccent, kLine, kLineDim, kGrey, kDarkGrey};
  for (int i = 0; i < 6; ++i) gfxFill(40 + i * 41, 86, 35, 22, kSwatch[i]);
  gfxTextCentre(160, 118, gfxPalette() == PALETTE_WHITE ? "FOR BEHIND AMBER ACRYLIC" : "FOR A BARE DISPLAY", kGrey, 2);
  drawStatus();
}

// The four settings of a settings page, at the corners.
static void drawSettingCorners(int page) {
  const char* label[4];
  const char* value[4];
  char text[4][8];
  for (int e = 0; e < 4; ++e) {
    int id = kSettingPages[page][e];
    label[e] = id == kNone ? "--" : kSettings[id].name;
    value[e] = id == kNone ? "" : settingText(id, text[e]);
  }
  drawCorners(label, value);
}

// MIDI: how the synth connects to other gear, and what is keeping time now.
void Ui::drawMidiPage() {
  gfxTextCentre(160, 6, "MIDI 4/5", kAccent, 2);
  drawSettingCorners(0);
  int clock = gSynth.clockState();
  Str bpm;
  bpm.addInt((int)(gSynth.tempoBpm() + 0.5f));
  bpm.add(" BPM");
  gfxTextCentre(160, 72, bpm.b, clock == CLK_STOPPED ? kGrey : kInk, 3);
  static const char* const kSays[4] = {"THE SYNTH KEEPS TIME", "", "NO CLOCK: TEMPO KNOB USED", "STOPPED: WAITS FOR START"};
  const char* says = clock == CLK_LOCKED ? (setting(SET_CLOCK) == CLOCK_USB ? "FOLLOWING USB CLOCK" : "FOLLOWING MIDI CLOCK")
                     : kSays[clock == CLK_INTERNAL ? 0 : clock == CLK_WAITING ? 2 : 3];
  gfxTextCentre(160, 97, says, clock == CLK_LOCKED ? kAccent : kGrey, 2);
  // the beat, a quarter at a time
  int quarter = clock == CLK_STOPPED ? -1 : clampi((int)(gSynth.beatNow() * 4.0f), 0, 3);
  for (int i = 0; i < 4; ++i) gfxFill(88 + i * 38, 115, 30, 8, i == quarter ? (i == 0 ? kAccent : kLine) : kDarkGrey);
  // what Channel, Thru and Clock Out amount to, in words
  Str plays;
  if (setting(SET_CHANNEL) == 0) {
    plays.add("PLAYS EVERY CHANNEL");
  } else {
    plays.add("PLAYS CHANNEL ");
    plays.addInt(setting(SET_CHANNEL));
    plays.add(" ONLY");
  }
  gfxTextCentre(160, 129, plays.b, kGrey, 2);
  bool thru = setting(SET_THRU) != 0, clockOut = setting(SET_CLOCK_OUT) != 0;
  gfxTextCentre(160, 146, thru && clockOut ? "OUT: NOTES AND CLOCK" : thru ? "OUT: NOTES PASSED ON" : clockOut ? "OUT: CLOCK ONLY" : "OUT: NOTHING IS SENT",
                kGrey, 2);
  drawStatus();
}

// ---- the DRAW page (Engine, third page): the patch's drawing ----

// A change to the drawing: like any change to the sound, it marks the patch edited and is saved.
void Ui::drawEdited() {
  gPatch.markEdited();
  stateTouched();
  dirty_ = true;
}

// Encoder 1 moves along the points (in paint mode, carrying the height
// with it); encoder 2 raises or lowers the point; encoder 3 picks a
// starting shape. Encoder 4 only pushes.
void Ui::drawTurn(int enc, int delta) {
  popupParam_ = -1;
  popupMs_ = 0;
  bool changed = false;
  if (enc == 0) {
    int h = gDraw.point(drawCursor_);
    int step = delta > 0 ? 1 : -1;
    for (int n = delta > 0 ? delta : -delta; n > 0; --n) {
      int next = clampi((int)drawCursor_ + step, 0, kDrawPoints - 1);
      if (next == drawCursor_) break;
      drawCursor_ = (uint8_t)next;
      if (drawPaint_) changed |= gDraw.setPoint(drawCursor_, h);
    }
  } else if (enc == 1) {
    int n = burst_[enc] < 250 ? ++burst_[enc] : 250;  // a fast spin moves further
    int step = n >= 4 ? 10 : n >= 2 ? 4 : 2;
    changed = gDraw.setPoint(drawCursor_, gDraw.point(drawCursor_) + delta * step);
  } else if (enc == 2) {
    int s = ((int)drawShape_ + delta) % DRAWSHAPE_COUNT;
    drawShape_ = (uint8_t)(s < 0 ? s + DRAWSHAPE_COUNT : s);
  }
  if (changed) drawEdited();
  dirty_ = true;
}

void Ui::drawPush(int enc) {
  popupParam_ = -1;
  popupMs_ = 0;
  bool changed = false;
  if (enc == 0) drawPaint_ = !drawPaint_;
  else if (enc == 1) changed = gDraw.setPoint(drawCursor_, 0);
  else if (enc == 2) changed = gDraw.startShape(drawShape_);
  else changed = gDraw.smooth();
  if (changed) drawEdited();
  dirty_ = true;
}

void Ui::drawDrawPage() {
  int pages = pageCount(MB_ENGINE);
  Str title;
  title.add("DRAW ");
  title.addInt(page_ + 1);
  title.add('/');
  title.addInt(pages);
  gfxTextCentre(160, 6, title.b, kAccent, 2);

  Str point, height;
  if (drawCursor_ + 1 < 10) point.add('0');
  point.addInt(drawCursor_ + 1);
  int h = gDraw.point(drawCursor_);
  if (h > 0) height.add('+');
  height.addInt(h);
  const char* const label[4] = {drawPaint_ ? "PAINT" : "POINT", "HEIGHT", "START", "SMOOTH"};
  const char* const value[4] = {point.b, height.b, kDrawShapeNames[drawShape_], "PUSH"};
  drawCorners(label, value);

  // The drawing, one cycle across the box and back to its first point, the point being edited picked out.
  const int x0 = 16, w = 288, top = 76, bottom = 134, mid = (top + bottom) / 2, amp = (bottom - top) / 2;
  gfxHLine(x0, mid, w, kGrid);
  auto xAt = [&](int i) { return x0 + i * w / kDrawPoints; };
  auto yAt = [&](int i) { return mid - gDraw.point(i & (kDrawPoints - 1)) * amp / 127; };
  for (int i = 0; i < kDrawPoints; ++i) gfxPlotLine(xAt(i), yAt(i), xAt(i + 1), yAt(i + 1), drawPaint_ ? kAccent : kLine);
  for (int i = 0; i < kDrawPoints; ++i) {
    bool here = i == drawCursor_;
    int s = here ? 7 : 3;
    gfxFill(xAt(i) - s / 2, yAt(i) - s / 2, s, s, here ? kInk : kLineDim);
  }
  gfxVLine(xAt(drawCursor_), top - 4, 3, kAccent);

  bool heard = playsDrawing(0) || playsDrawing(1);
  if (!heard) hint(141, "NO OSCILLATOR PLAYS IT");
  else if (gDraw.silent()) hint(141, "FLAT: IT MAKES NO SOUND");
  drawStatus();
}

// Input: the audio input's level against the converter's limit, and where it is sent.
void Ui::drawInputPage() {
  gfxTextCentre(160, 6, "INPUT 5/5", kAccent, 2);
  drawSettingCorners(1);
  const int x0 = 20, w = 280, y = 98;
  float level = gSynth.inputLevel();
  bool clipped = gSynth.inputClipped();
  // 48 dB of range, so a quiet microphone still shows
  float shown = level > 0.0f ? clampf(1.0f + log2f_(level) * (6.0206f / 48.0f), 0.0f, 1.0f) : 0.0f;
  gfxFrame(x0 - 2, y - 2, w + 4, 20, kDarkGrey);
  gfxFill(x0, y, (int)(shown * (float)w), 16, clipped ? kAccent : kLine);
  gfxFill(x0 + w - 3, y - 6, 3, 28, clipped ? kAccent : kGrey);  // the converter's limit
  gfxText(x0, y - 21, "LEVEL", kLineDim, 2);
  gfxTextRight(x0 + w, y - 21, clipped ? "CLIP" : "", kAccent, 2);
  static const char* const kSays[INPUT_COUNT] = {"THE INPUT IS NOT IN USE", "JOINS AFTER THE EFFECTS", "GOES THROUGH THE EFFECTS"};
  gfxTextCentre(160, 126, clipped ? "TOO LOUD: TURN GAIN DOWN" : kSays[setting(SET_INPUT_TO)], clipped ? kAccent : kGrey, 2);
  gfxTextCentre(160, 144, setting(SET_INPUT_SOURCE) == INSRC_MIC ? "MICROPHONE: ONE CHANNEL" : "LINE: STEREO", kGrey, 2);
  drawStatus();
}

// A control that is following the patch was moved but has not taken over:
// a track with the patch's value and the control's position on it, and
// which way to go.
void Ui::drawPickup(int id) {
  int index = -1;
  for (int i = 0; i < kPanelCount; ++i)
    if (kPanelParams[i] == id) index = i;
  if (index < 0) return;
  const int x0 = 20, w = 280, y = 104;
  int xPatch = x0 + (int)(gParams.norm(id) * (float)(w - 1));
  int xPanel = x0 + gPatch.position(index) * (w - 1) / kPotMax;
  bool up = xPatch > xPanel;
  gfxTextCentre(160, 56, "FOLLOWING THE PATCH", kGrey, 2);
  gfxFrame(x0 - 2, y, w + 4, 20, kDarkGrey);
  gfxFill(up ? xPanel : xPatch, y + 7, (up ? xPatch - xPanel : xPanel - xPatch) + 1, 6, kLineDim);  // the distance still to go
  gfxFill(xPatch - 2, y - 8, 5, 36, kAccent);
  gfxFill(xPanel - 3, y + 2, 7, 16, kInk);
  gfxTextCentre(clampi(xPatch, 38, 282), y - 28, "PATCH", kAccent, 2);
  gfxTextCentre(clampi(xPanel, 38, 282), y + 34, "PANEL", kInk, 2);
  gfxTextCentre(160, 166, up ? "HIGHER >>" : "<< LOWER", up ? kAccent : kLine, 3);
  gfxTextCentre(160, 198, "TO TAKE OVER", kGrey, 2);
}

void Ui::drawModePage() {
  if (onDrawPage()) {
    drawDrawPage();
    return;
  }
  if (mode_ == MB_PATCH) {
    if (page_ == 0) drawPatchPage();
    else if (page_ == 1) drawNamePage();
    else if (page_ == 2) drawSetupPage();
    else if (page_ == 3) drawMidiPage();
    else drawInputPage();
    return;
  }
  int pages = pageCount(mode_);
  Str title;
  title.add(kPages[mode_][page_].title);
  if (pages > 1) {
    title.add(' ');
    title.addInt(page_ + 1);
    title.add('/');
    title.addInt(pages);
  }
  gfxTextCentre(160, 6, title.b, kAccent, 2);

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
      gfxText(8, y, label.b, kLineDim, 2);
      gfxText(8, y + 16, v.b, kInk, 3);
    } else {
      gfxTextRight(312, y, label.b, kLineDim, 2);
      gfxTextRight(312, y + 16, v.b, kInk, 3);
    }
  }
  // The middle band is the live scope, except just after an encoder is
  // turned, when it shows what that value does.
  if (softMs_ > 0 && softParam_ >= 0 && drawSoftPicture(softParam_, kMini)) {
    // the picture of the value just turned
  } else if (pageParam(0) == P_VOCODER && gParams.step(P_VOCODER) > 0) {
    drawVocoder(kMini);  // the Vocoder page, with the vocoder on: the voice's bands, live
  } else {
    drawScope(8, 74, 304, 82);
  }
  drawStatus();
}

void Ui::drawParamPopup(int id) {
  const ParamDef& d = kParams[id];
  Str v;
  formatParam(id, v);
  if (id == P_ARP_TEMPO && gSynth.clockState() == CLK_LOCKED && !popupPickup_) {  // the knob is not what sets it
    v = Str();
    v.add("EXT ");
    v.addInt((int)(gSynth.tempoBpm() + 0.5f));
  }
  const char* name = paramName(id);
  drawHeader(kSectionNames[d.section], name[0] ? name : d.name, v.b);
  if (popupPickup_) {
    drawPickup(id);
    return;
  }

  switch (id) {
    // ---- oscillators ----
    case P_OSC1_WAVE: case P_OSC1_OCTAVE: case P_OSC1_MACRO: case P_OSC1_COLOR: case P_OSC1_ENGINE:
      drawOscWave(0);
      drawFlatHint(0, kPic);
      return;
    case P_OSC2_WAVE: case P_OSC2_OCTAVE: case P_OSC2_MACRO: case P_OSC2_COLOR: case P_OSC2_ENGINE:
      drawOscWave(1);
      drawFlatHint(1, kPic);
      return;
    case P_OSC2_DETUNE: drawDetune(); return;
    // ---- panel controls whose section would otherwise give them another control's picture ----
    case P_SUB_OCTAVE: drawSub(kPic); return;
    case P_LFO_SYNC: drawLfoSync(kPic); return;
    case P_CHORUS: case P_CHORUS_TYPE: drawChorus(kPic); return;
    case P_XMOD: drawXmod(); return;
    case P_SYNC: drawSync(); return;
    // ---- voice ----
    case P_GLIDE: drawGlide(); return;
    case P_VOICE_MODE: drawVoiceMode(); return;
    case P_DRIFT: drawDrift(); return;
    case P_SPREAD: drawSpread(); return;
    // ---- filter ----
    case P_FILT_ENV_AMT: drawFilterEnvAmount(); return;
    case P_KEYTRACK: drawFilterKeytrack(); return;
    case P_DRIVE: drawFilterDrive(); return;
    case P_CUTOFF: case P_RESONANCE: case P_FILT_TYPE: case P_FILT_SLOPE:
      drawFilterCurve();
      if (id == P_FILT_TYPE && gParams.step(P_FILT_SLOPE) == SLOPE_LADDER) hint(kPic.y + 2, "THE LADDER IS LOW-PASS ONLY");
      return;
    // ---- the rest of the panel ----
    case P_VELOCITY: drawVelocityLine(1.0f - gParams.norm(P_VELOCITY), 1.0f); return;
    case P_VOLUME: drawLevelWave(gParams.norm(P_VOLUME) * gParams.norm(P_VOLUME)); return;
    case P_ARP_MODE: case P_ARP_OCTAVES: case P_ARP_TEMPO: case P_ARP_GATE: drawArp(id); return;
    default: break;
  }
  switch (d.section) {
    case SEC_MIXER: drawMixer(id); return;
    case SEC_FENV: drawAdsr(P_FENV_A, id); return;
    case SEC_AENV: drawAdsr(P_AENV_A, id); return;
    case SEC_LFO: drawLfo(); return;
    default: break;
  }
  // a value from an encoder page, moved from outside (by MIDI): its page picture, full size
  if (drawSoftPicture(id, kPic)) return;
  if (d.kind == PK_STEP && d.labels) drawSteps(d, gParams.step(id));
  else if (d.kind == PK_STEP) drawBar(gParams.frac(id));
  else drawBar(gParams.norm(id));
}

}  // namespace fw
