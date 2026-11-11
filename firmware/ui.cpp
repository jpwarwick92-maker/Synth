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

const char* const kModeNames[MB_COUNT] = {"FX 1", "FX 2", "MOD", "ARP", "ENGINE", "PATCH"};

// Engine page: one parameter per encoder, in encoder order
// (left-top, left-bottom, right-top, right-bottom).
const uint8_t kEnginePage[4] = {P_UNI_DETUNE, P_SUB_OCTAVE, P_BEND_RANGE, P_FILT_SLOPE};

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
  switch (d.unit) {
    case U_STEP: s.add(d.labels[st]); break;
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
    case U_LFOSHAPE: s.add(kLfoShapeNames[mapLfoShape(n)]); break;
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
    if (px > 0) gfxPlotLine(x + px - 1, py, x + px, yy, kTeal);
    py = yy;
  }
}

// Bottom strip: voices in use, voice mode, output level, MIDI activity.
void drawStatus() {
  const int y = 222;
  for (int i = 0; i < kNumVoices; ++i) gfxFill(8 + i * 10, y, 7, 7, gSynth.voiceActive(i) ? kTeal : kDarkGrey);
  gfxText(96, y, kParams[P_VOICE_MODE].labels[gParams.step(P_VOICE_MODE)], kGrey, 1);

  float peak = minf(gSynth.peakLevel(), 1.0f);
  gfxText(150, y, "LEVEL", kGrey, 1);
  gfxFill(186, y, 70, 7, kDarkGrey);
  gfxFill(186, y, (int)(peak * 70.0f), 7, peak > 0.7f ? kAmber : kDimTeal);

  gfxText(274, y, "MIDI", kGrey, 1);
  gfxFill(304, y, 7, 7, gMidi.idleMs() < 120 ? kAmber : kDarkGrey);
}

void drawHeader(const char* section, const char* name, const char* value) {
  gfxText(8, 6, section, kDimTeal, 1);
  gfxText(8, 20, name, kAmber, 2);
  gfxTextRight(312, 10, value, kWhite, 3);
  gfxHLine(8, 44, 304, kGrid);
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

void drawOscWave(int wave, float pw) {
  const int x0 = 10, w = 300, mid = 138, amp = 64;
  gfxHLine(x0, mid, w, kGrid);
  int py = 0;
  for (int px = 0; px < w; ++px) {
    float ph = (float)px * (2.0f / (float)w);
    ph -= floorf_(ph);
    int yy = mid - (int)(waveShape(wave, ph, pw) * (float)amp);
    if (px > 0) gfxPlotLine(x0 + px - 1, py, x0 + px, yy, kTeal);
    py = yy;
  }
}

void drawMixer(int selected) {
  static const char* const kNames[4] = {"OSC 1", "OSC 2", "SUB", "NOISE"};
  for (int i = 0; i < 4; ++i) {
    int id = P_MIX_OSC1 + i;
    int x = 30 + i * 72;
    int h = (int)(gParams.norm(id) * 136.0f);
    uint16_t c = id == selected ? kAmber : kDimTeal;
    gfxFrame(x, 58, 44, 140, kDarkGrey);
    gfxFill(x + 2, 196 - h, 40, h, c);
    gfxTextCentre(x + 22, 204, kNames[i], id == selected ? kAmber : kGrey, 1);
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

// The filter's frequency response, 20 Hz to 20 kHz, as it is set right now.
void drawFilterCurve() {
  const int x0 = 8, w = 304, y0 = 54, h = 160;
  const float topDb = 18.0f, rangeDb = 66.0f;
  int zero = y0 + (int)(topDb / rangeDb * (float)h);
  gfxHLine(x0, zero, w, kGrid);
  static const float kMarks[3] = {100.0f, 1000.0f, 10000.0f};
  static const char* const kMarkNames[3] = {"100", "1k", "10k"};
  for (int i = 0; i < 3; ++i) {
    int x = x0 + (int)(log2f_(kMarks[i] / 20.0f) * 0.1f * (float)w);
    gfxVLine(x, y0, h, kGrid);
    gfxTextCentre(x, y0 + h + 3, kMarkNames[i], kGrey, 1);
  }

  float fc = mapCutoffHz(gParams.norm(P_CUTOFF));
  float q = mapResonanceQ(gParams.norm(P_RESONANCE));
  float k = 1.0f / q;
  float comp = q > 1.0f ? 1.0f / sqrtf_(q) : 1.0f;
  int type = gParams.step(P_FILT_TYPE);
  bool slope24 = gParams.step(P_FILT_SLOPE) == 1;

  int py = 0;
  for (int px = 0; px < w; ++px) {
    float f = 20.0f * exp2f_(10.0f * (float)px / (float)(w - 1));
    float wn = f / fc;
    float mag = filterStage(wn, k, type) * comp;
    if (slope24) mag *= filterStage(wn, 1.41421356f, type);
    float db = 6.0206f * log2f_(maxf(mag, 1e-6f));
    int yy = clampi(y0 + (int)((topDb - db) / rangeDb * (float)h), y0, y0 + h - 2);
    if (px > 0) gfxPlotLine(x0 + px - 1, py, x0 + px, yy, kTeal);
    py = yy;
  }
}

void drawAdsr(int firstParam, int selected) {
  float a = gParams.norm(firstParam), d = gParams.norm(firstParam + 1);
  float s = gParams.norm(firstParam + 2), r = gParams.norm(firstParam + 3);
  const int top = 62, bottom = 208, hgt = bottom - top;
  int wA = 16 + (int)(70.0f * a), wD = 16 + (int)(70.0f * d), wS = 50, wR = 16 + (int)(70.0f * r);
  int x = 12;
  gfxHLine(8, bottom + 2, 304, kGrid);

  auto yOf = [&](float level) { return bottom - (int)(level * (float)hgt); };
  auto seg = [&](int width, int which, float (*shape)(float t, float s), float sus) {
    uint16_t c = (firstParam + which == selected) ? kAmber : kTeal;
    int px = x, py = yOf(shape(0.0f, sus));
    for (int i = 1; i <= width; ++i) {
      int yy = yOf(shape((float)i / (float)width, sus));
      gfxPlotLine(px, py, x + i, yy, c);
      px = x + i;
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
    gfxTextCentre(lx + widths[i] / 2, bottom + 8, kNames[i], firstParam + i == selected ? kAmber : kGrey, 1);
    lx += widths[i];
  }
}

void drawLfo() {
  static const float kRandom[8] = {0.3f, -0.7f, 0.9f, -0.2f, 0.6f, -0.9f, 0.1f, -0.5f};
  int shape = mapLfoShape(gParams.norm(P_LFO_SHAPE));
  const int x0 = 10, w = 300, mid = 128, amp = 56;
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
    if (px > 0) gfxPlotLine(x0 + px - 1, py, x0 + px, yy, kTeal);
    py = yy;
  }
  Str s;
  s.add("TO ");
  s.add(kParams[P_LFO_DEST].labels[gParams.step(P_LFO_DEST)]);
  gfxText(8, 204, s.b, kGrey, 1);
  Str r;
  formatParam(P_LFO_RATE, r);
  gfxTextRight(312, 204, r.b, kGrey, 1);
}

}  // namespace

void Ui::init() {
  mode_ = MB_NONE;
  popupParam_ = -1;
  popupOsc_ = -1;
  popupMs_ = 0;
  dirty_ = true;
}

void Ui::paramChanged(int id) {
  // Engine-page values are already on screen while that page is open.
  if (mode_ == MB_ENGINE && kParams[id].section == SEC_ENGINE) {
    dirty_ = true;
    return;
  }
  popupParam_ = (int16_t)id;
  popupOsc_ = -1;
  popupMs_ = kPopupMs;
  dirty_ = true;
}

void Ui::modeButton(int which) {
  if (which < 0 || which >= MB_COUNT) return;
  mode_ = mode_ == which ? (int8_t)MB_NONE : (int8_t)which;  // pressing the lit button goes home
  popupParam_ = -1;
  popupOsc_ = -1;
  popupMs_ = 0;
  dirty_ = true;
}

void Ui::oscEngineButton(int osc) {
  popupOsc_ = (int8_t)osc;
  popupParam_ = -1;
  popupMs_ = kPopupMs;
  dirty_ = true;
}

void Ui::encoderTurn(int enc, int delta) {
  if (mode_ != MB_ENGINE || enc < 0 || enc >= 4) return;  // only the Engine page is live
  int id = kEnginePage[enc];
  if (gParams.set(id, gParams.step(id) + delta)) onParamChanged(id);
}

void Ui::encoderPush(int enc) {
  if (mode_ != MB_ENGINE || enc < 0 || enc >= 4) return;
  int id = kEnginePage[enc];
  if (gParams.set(id, kParams[id].def)) onParamChanged(id);  // push = back to default
}

bool Ui::tick(uint32_t dtMs) {
  bool popup = popupMs_ > 0;
  if (popup) {
    if (popupMs_ > dtMs) {
      popupMs_ -= dtMs;
    } else {
      popupMs_ = 0;
      popupParam_ = -1;
      popupOsc_ = -1;
      popup = false;
      dirty_ = true;
    }
  }

  // Pop-ups are static; the other screens carry the live scope and status.
  if (popup && !dirty_) return false;
  dirty_ = false;

  gfxClear(kBlack);
  if (popup && popupParam_ >= 0) drawParamPopup(popupParam_);
  else if (popup && popupOsc_ >= 0) drawEnginePopup(popupOsc_);
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
  gfxTextCentre(160, 10, kModeNames[mode_], kAmber, 2);
  if (mode_ != MB_ENGINE) {
    gfxTextCentre(160, 104, "NOT BUILT YET", kGrey, 2);
    drawStatus();
    return;
  }
  // one value per encoder, at the corner nearest that encoder
  static const int kLabelY[2] = {40, 176};
  for (int e = 0; e < 4; ++e) {
    int id = kEnginePage[e];
    Str v;
    formatParam(id, v);
    int y = kLabelY[e & 1];
    if (e < 2) {
      gfxText(8, y, kParams[id].name, kDimTeal, 1);
      gfxText(8, y + 12, v.b, kWhite, 2);
    } else {
      gfxTextRight(312, y, kParams[id].name, kDimTeal, 1);
      gfxTextRight(312, y + 12, v.b, kWhite, 2);
    }
  }
  drawScope(8, 78, 304, 88);
  drawStatus();
}

void Ui::drawParamPopup(int id) {
  const ParamDef& d = kParams[id];
  Str v;
  formatParam(id, v);
  drawHeader(kSectionNames[d.section], d.name, v.b);

  switch (d.section) {
    case SEC_OSC1:
      drawOscWave(gParams.step(P_OSC1_WAVE), mapPulseWidth(gParams.norm(P_OSC1_MACRO)));
      break;
    case SEC_OSC2:
      drawOscWave(gParams.step(P_OSC2_WAVE), 0.5f);
      break;
    case SEC_MIXER:
      drawMixer(id);
      break;
    case SEC_FILTER:
      drawFilterCurve();
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
      if (d.kind == PK_STEP && d.labels) drawSteps(d, gParams.step(id));
      else if (d.kind == PK_STEP) drawBar((float)gParams.step(id) / (float)(d.steps - 1));
      else drawBar(gParams.norm(id));
      break;
  }
}

void Ui::drawEnginePopup(int osc) {
  drawHeader(osc == 0 ? "OSC 1" : "OSC 2", "ENGINE", "VA");
  gfxTextCentre(160, 96, "VA ONLY", kWhite, 3);
  gfxTextCentre(160, 140, "FM / WAVETABLE / PLUCK", kGrey, 1);
  gfxTextCentre(160, 154, "NOT BUILT YET", kGrey, 1);
}

}  // namespace fw
