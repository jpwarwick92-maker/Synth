#include "params.h"

#include "fw_math.h"

namespace fw {

namespace {

const char* const kWaveLabels[] = {"SAW", "SQR", "TRI", "SIN"};
const char* const kOctaveLabels[] = {"-2", "-1", "0", "+1", "+2"};
const char* const kFilterLabels[] = {"LP", "BP", "HP"};
const char* const kModeLabels[] = {"POLY", "MONO", "UNI"};
const char* const kDestLabels[] = {"PITCH", "CUTOFF", "AMP", "PW"};
const char* const kSubLabels[] = {"-1 OCT", "-2 OCT"};
const char* const kSlopeLabels[] = {"12 dB", "24 dB"};

constexpr uint16_t pot(float x) { return (uint16_t)(x * (float)kPotMax + 0.5f); }

}  // namespace

const char* const kSectionNames[SEC_COUNT] = {
    "OSC 1", "OSC 2", "MIXER", "FILTER", "VOICE", "FILTER ENV", "AMP ENV", "LFO", "OUTPUT", "ENGINE",
};

const char* const kLfoShapeNames[LFOS_COUNT] = {"SINE", "TRI", "SAW", "SQUARE", "RANDOM"};

// Controller numbers: the usual MIDI number where one exists (1 mod wheel
// and 64 sustain are handled separately), otherwise from the unassigned
// ranges 20-31, 85-90 and 102-119.
const ParamDef kParams[P_COUNT] = {
    // key            name          section     kind     steps def        cc   unit          labels         intMin
    {"osc1_wave",     "WAVE",       SEC_OSC1,   PK_STEP, 4,    0,         20,  U_STEP,       kWaveLabels,   0},
    {"osc1_octave",   "OCTAVE",     SEC_OSC1,   PK_STEP, 5,    2,         21,  U_STEP,       kOctaveLabels, 0},
    {"osc1_macro",    "PULSE WIDTH", SEC_OSC1,  PK_POT,  0,    pot(0.0f), 22,  U_PW,         nullptr,       0},
    {"osc2_wave",     "WAVE",       SEC_OSC2,   PK_STEP, 4,    0,         23,  U_STEP,       kWaveLabels,   0},
    {"osc2_octave",   "OCTAVE",     SEC_OSC2,   PK_STEP, 5,    2,         24,  U_STEP,       kOctaveLabels, 0},
    {"osc2_detune",   "DETUNE",     SEC_OSC2,   PK_POT,  0,    pot(0.62f), 25, U_CENTS,      nullptr,       0},
    {"mix_osc1",      "OSC 1",      SEC_MIXER,  PK_POT,  0,    pot(0.8f), 26,  U_PERCENT,    nullptr,       0},
    {"mix_osc2",      "OSC 2",      SEC_MIXER,  PK_POT,  0,    pot(0.6f), 27,  U_PERCENT,    nullptr,       0},
    {"mix_sub",       "SUB",        SEC_MIXER,  PK_POT,  0,    pot(0.0f), 28,  U_PERCENT,    nullptr,       0},
    {"mix_noise",     "NOISE",      SEC_MIXER,  PK_POT,  0,    pot(0.0f), 29,  U_PERCENT,    nullptr,       0},
    {"filter_cutoff", "CUTOFF",     SEC_FILTER, PK_POT,  0,    pot(0.6f), 74,  U_CUTOFF,     nullptr,       0},
    {"filter_resonance", "RESONANCE", SEC_FILTER, PK_POT, 0,   pot(0.2f), 71,  U_PERCENT,    nullptr,       0},
    {"filter_env_amount", "ENV AMOUNT", SEC_FILTER, PK_POT, 0, pot(0.7f), 85,  U_ENVAMT,     nullptr,       0},
    {"filter_keytrack", "KEYTRACK", SEC_FILTER, PK_POT,  0,    pot(0.5f), 86,  U_PERCENT,    nullptr,       0},
    {"filter_drive",  "DRIVE",      SEC_FILTER, PK_POT,  0,    pot(0.0f), 87,  U_PERCENT,    nullptr,       0},
    {"filter_type",   "TYPE",       SEC_FILTER, PK_STEP, 3,    0,         88,  U_STEP,       kFilterLabels, 0},
    {"voice_glide",   "GLIDE",      SEC_VOICE,  PK_POT,  0,    pot(0.0f), 5,   U_GLIDE,      nullptr,       0},
    {"voice_mode",    "MODE",       SEC_VOICE,  PK_STEP, 3,    0,         89,  U_STEP,       kModeLabels,   0},
    {"fenv_attack",   "ATTACK",     SEC_FENV,   PK_POT,  0,    pot(0.05f), 102, U_TIME,      nullptr,       0},
    {"fenv_decay",    "DECAY",      SEC_FENV,   PK_POT,  0,    pot(0.6f), 103, U_TIME,       nullptr,       0},
    {"fenv_sustain",  "SUSTAIN",    SEC_FENV,   PK_POT,  0,    pot(0.3f), 104, U_PERCENT,    nullptr,       0},
    {"fenv_release",  "RELEASE",    SEC_FENV,   PK_POT,  0,    pot(0.6f), 105, U_TIME,       nullptr,       0},
    {"aenv_attack",   "ATTACK",     SEC_AENV,   PK_POT,  0,    pot(0.1f), 73,  U_TIME,       nullptr,       0},
    {"aenv_decay",    "DECAY",      SEC_AENV,   PK_POT,  0,    pot(0.6f), 75,  U_TIME,       nullptr,       0},
    {"aenv_sustain",  "SUSTAIN",    SEC_AENV,   PK_POT,  0,    pot(0.8f), 79,  U_PERCENT,    nullptr,       0},
    {"aenv_release",  "RELEASE",    SEC_AENV,   PK_POT,  0,    pot(0.6f), 72,  U_TIME,       nullptr,       0},
    {"aenv_velocity", "VELOCITY",   SEC_AENV,   PK_POT,  0,    pot(0.6f), 90,  U_PERCENT,    nullptr,       0},
    {"lfo_rate",      "RATE",       SEC_LFO,    PK_POT,  0,    pot(0.7f), 76,  U_LFOHZ,      nullptr,       0},
    {"lfo_depth",     "DEPTH",      SEC_LFO,    PK_POT,  0,    pot(0.0f), 77,  U_PERCENT,    nullptr,       0},
    {"lfo_shape",     "SHAPE",      SEC_LFO,    PK_POT,  0,    pot(0.0f), 108, U_LFOSHAPE,   nullptr,       0},
    {"lfo_destination", "DESTINATION", SEC_LFO, PK_STEP, 4,    0,         109, U_STEP,       kDestLabels,   0},
    {"volume",        "VOLUME",     SEC_OUTPUT, PK_POT,  0,    pot(0.75f), 7,  U_PERCENT,    nullptr,       0},
    {"unison_detune", "UNI DETUNE", SEC_ENGINE, PK_STEP, 51,   15,        110, U_INT_CENTS,  nullptr,       0},
    {"sub_octave",    "SUB OCTAVE", SEC_ENGINE, PK_STEP, 2,    0,         111, U_STEP,       kSubLabels,    0},
    {"bend_range",    "BEND RANGE", SEC_ENGINE, PK_STEP, 12,   1,         112, U_INT_SEMI,   nullptr,       1},
    {"filter_slope",  "FILT SLOPE", SEC_ENGINE, PK_STEP, 2,    1,         113, U_STEP,       kSlopeLabels,  0},
};

Params gParams;

void Params::reset() {
  for (int i = 0; i < P_COUNT; ++i) val[i] = kParams[i].def;
}

bool Params::set(int id, int raw) {
  if (id < 0 || id >= P_COUNT) return false;
  const ParamDef& d = kParams[id];
  int hi = d.kind == PK_STEP ? d.steps - 1 : kPotMax;
  uint16_t v = (uint16_t)clampi(raw, 0, hi);
  if (val[id] == v) return false;
  val[id] = v;
  return true;
}

bool Params::setFromCC(int id, int value7) {
  if (id < 0 || id >= P_COUNT) return false;
  const ParamDef& d = kParams[id];
  value7 = clampi(value7, 0, 127);
  if (d.kind == PK_STEP) return set(id, value7 * d.steps / 128);
  return set(id, (value7 * kPotMax + 63) / 127);
}

int paramForCC(int cc) {
  for (int i = 0; i < P_COUNT; ++i)
    if (kParams[i].cc == cc) return i;
  return -1;
}

float mapCutoffHz(float n) { return 20.0f * exp2f_(n * 10.0f); }

float mapResonanceQ(float n) { return 0.5f * exp2f_(n * 4.585f); }

float mapEnvAmtOct(float n) {
  float u = 2.0f * n - 1.0f;
  float a = absf(u);
  if (a < 0.04f) return 0.0f;
  float o = (a - 0.04f) * (6.0f / 0.96f);
  return u < 0.0f ? -o : o;
}

float mapEnvTime(float n) { return 0.001f * exp2f_(n * 13.2877f); }

float mapLfoHz(float n) { return 0.05f * exp2f_(n * 9.2288f); }

float mapGlideTime(float n) { return n < 0.01f ? 0.0f : 0.005f * exp2f_(n * 9.23f); }

float mapDetuneCents(float n) {
  float u = 2.0f * n - 1.0f;
  if (absf(u) < 0.02f) return 0.0f;
  return 100.0f * u * absf(u);
}

float mapPulseWidth(float n) { return 0.5f + 0.45f * n; }

float mapDriveGain(float n) { return 1.0f + 15.0f * n * n; }

float mapVolume(float n) { return 1.4f * n * n; }

int mapLfoShape(float n) { return clampi((int)(n * (float)LFOS_COUNT), 0, LFOS_COUNT - 1); }

}  // namespace fw
