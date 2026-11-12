#include "params.h"

#include "fw_math.h"

namespace fw {

namespace {

const char* const kWaveLabels[] = {"SAW", "SQR", "TRI", "SIN", "SUPER"};
const char* const kArpOctaveLabels[] = {"1", "2", "3", "4"};
const char* const kOctaveLabels[] = {"-2", "-1", "0", "+1", "+2"};
const char* const kFilterLabels[] = {"LP", "BP", "HP"};
const char* const kModeLabels[] = {"POLY", "MONO", "UNI"};
const char* const kDestLabels[] = {"PITCH", "CUTOFF", "AMP", "PW"};
const char* const kSubLabels[] = {"-1 OCT", "-2 OCT"};
const char* const kSlopeLabels[] = {"12 dB", "24 dB", "LADDER"};
const char* const kSyncLabels[] = {"FREE", "KEY"};
const char* const kArpLabels[] = {"OFF", "UP", "DOWN", "UP-DOWN", "RANDOM", "PLAYED"};
const char* const kEngineLabels[] = {"VA", "FM", "WAVETABLE", "PLUCK"};
const char* const kOnOffLabels[] = {"OFF", "ON"};

constexpr uint16_t pot(float x) { return (uint16_t)(x * (float)kPotMax + 0.5f); }

}  // namespace

const char* const kSectionNames[SEC_COUNT] = {
    "OSC 1", "OSC 2", "MIXER", "FILTER", "VOICE", "FILTER ENV", "AMP ENV", "LFO", "OUTPUT", "ENGINE",
    "FX 1", "FX 2", "MOD", "ARP",
};

const char* const kEngineWaveName[ENG_COUNT] = {"WAVE", "RATIO", "TABLE", "EXCITER"};
const char* const kEngineMacroName[ENG_COUNT] = {"PULSE WIDTH", "FM AMOUNT", "POSITION", "DAMPING"};
const char* const kEngineColorName[ENG_COUNT] = {"", "FEEDBACK", "", "DECAY"};
const char* const kEngineWaveLabels[ENG_COUNT][WAVE_COUNT] = {
    {"SAW", "SQR", "TRI", "SIN", "SUPER"},
    {"1:1", "2:1", "3:1", "3.5:1", "4:1"},
    {"TABLE 1", "TABLE 2", "TABLE 3", "TABLE 4", "TABLE 5"},
    {"NOISE", "SOFT", "SNAP", "STRING", "PICK"},
};

// Control order. The first 32 are as in the first build; later panel
// controls follow in the order they were added.
const uint8_t kPanelParams[kPanelCount] = {
    P_OSC1_WAVE, P_OSC1_OCTAVE, P_OSC1_MACRO, P_OSC2_WAVE, P_OSC2_OCTAVE, P_OSC2_DETUNE,
    P_MIX_OSC1, P_MIX_OSC2, P_MIX_SUB, P_MIX_NOISE,
    P_CUTOFF, P_RESONANCE, P_FILT_ENV_AMT, P_KEYTRACK, P_DRIVE, P_FILT_TYPE,
    P_GLIDE, P_VOICE_MODE,
    P_FENV_A, P_FENV_D, P_FENV_S, P_FENV_R, P_AENV_A, P_AENV_D, P_AENV_S, P_AENV_R, P_VELOCITY,
    P_LFO_RATE, P_LFO_DEPTH, P_LFO_SHAPE, P_LFO_DEST,
    P_VOLUME,
    P_ARP_MODE, P_ARP_OCTAVES, P_ARP_TEMPO, P_ARP_GATE,
    P_DRIFT, P_SPREAD, P_XMOD, P_SYNC,
};

bool isPanelParam(int id) {
  for (int i = 0; i < kPanelCount; ++i)
    if (kPanelParams[i] == id) return true;
  return false;
}

const char* const kLfoShapeNames[LFOS_COUNT] = {"SINE", "TRI", "SAW", "SQUARE", "RANDOM"};

// Controller numbers: the usual MIDI number where one exists (1 mod wheel
// and 64 sustain are handled separately), otherwise from the unassigned
// ranges 14-31, 46-63, 85-90 and 102-119. Hold is 69, the standard "hold 2".
const ParamDef kParams[P_COUNT] = {
    // key            name          section     kind     steps def        cc   unit          labels         intMin
    {"osc1_wave",     "WAVE",       SEC_OSC1,   PK_STEP, 5,    0,         20,  U_STEP,       kWaveLabels,   0},
    {"osc1_octave",   "OCTAVE",     SEC_OSC1,   PK_STEP, 5,    2,         21,  U_STEP,       kOctaveLabels, 0},
    {"osc1_macro",    "PULSE WIDTH", SEC_OSC1,  PK_POT,  0,    pot(0.0f), 22,  U_PW,         nullptr,       0},
    {"osc2_wave",     "WAVE",       SEC_OSC2,   PK_STEP, 5,    0,         23,  U_STEP,       kWaveLabels,   0},
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
    {"lfo_shape",     "SHAPE",      SEC_LFO,    PK_STEP, 5,    0,         108, U_STEP,       kLfoShapeNames, 0},
    {"lfo_destination", "DESTINATION", SEC_LFO, PK_STEP, 4,    0,         109, U_STEP,       kDestLabels,   0},
    {"volume",        "VOLUME",     SEC_OUTPUT, PK_POT,  0,    pot(0.75f), 7,  U_PERCENT,    nullptr,       0},
    {"unison_detune", "UNI DETUNE", SEC_ENGINE, PK_STEP, 51,   15,        110, U_INT_CENTS,  nullptr,       0},
    {"sub_octave",    "SUB OCTAVE", SEC_ENGINE, PK_STEP, 2,    0,         111, U_STEP,       kSubLabels,    0},
    {"bend_range",    "BEND RANGE", SEC_ENGINE, PK_STEP, 12,   1,         112, U_INT_SEMI,   nullptr,       1},
    {"filter_slope",  "FILT SLOPE", SEC_ENGINE, PK_STEP, 3,    1,         113, U_STEP,       kSlopeLabels,  0},
    {"delay_time",    "TIME",       SEC_FX1,    PK_STEP, 101,  55,        46,  U_DLY_TIME,   nullptr,       0},
    {"delay_tone",    "TONE",       SEC_FX1,    PK_STEP, 101,  60,        47,  U_INT_PERCENT, nullptr,      0},
    {"delay_feedback", "FEEDBACK",  SEC_FX1,    PK_STEP, 101,  35,        48,  U_INT_PERCENT, nullptr,      0},
    {"delay_mix",     "MIX",        SEC_FX1,    PK_STEP, 101,  0,         49,  U_INT_PERCENT, nullptr,      0},
    {"chorus",        "CHORUS",     SEC_FX2,    PK_STEP, 101,  0,         50,  U_INT_PERCENT, nullptr,      0},
    {"reverb_size",   "RVB SIZE",   SEC_FX2,    PK_STEP, 101,  50,        51,  U_RVB_TIME,   nullptr,       0},
    {"reverb_tone",   "RVB TONE",   SEC_FX2,    PK_STEP, 101,  50,        52,  U_INT_PERCENT, nullptr,      0},
    {"reverb_mix",    "RVB MIX",    SEC_FX2,    PK_STEP, 101,  0,         53,  U_INT_PERCENT, nullptr,      0},
    {"lfo_fade",      "LFO FADE",   SEC_MOD,    PK_STEP, 101,  0,         54,  U_FADE_TIME,  nullptr,       0},
    {"vel_cutoff",    "VEL>CUTOFF", SEC_MOD,    PK_STEP, 101,  0,         55,  U_INT_PERCENT, nullptr,      0},
    {"fenv_pitch",    "ENV>PITCH",  SEC_MOD,    PK_STEP, 25,   0,         56,  U_INT_SEMI,   nullptr,       0},
    {"lfo_sync",      "LFO SYNC",   SEC_MOD,    PK_STEP, 2,    0,         57,  U_STEP,       kSyncLabels,   0},
    {"arp_mode",      "MODE",       SEC_ARP,    PK_STEP, 6,    0,         58,  U_STEP,       kArpLabels,    0},
    {"arp_tempo",     "TEMPO",      SEC_ARP,    PK_POT,  0,    pot(0.4f), 59,  U_BPM,        nullptr,       0},
    {"arp_octaves",   "OCTAVES",    SEC_ARP,    PK_STEP, 4,    0,         60,  U_STEP,       kArpOctaveLabels, 0},
    {"arp_gate",      "GATE",       SEC_ARP,    PK_POT,  0,    pot(0.474f), 61, U_GATE,      nullptr,       0},
    {"osc1_color",    "COLOR",      SEC_OSC1,   PK_STEP, 101,  0,         62,  U_INT_PERCENT, nullptr,      0},
    {"osc2_macro",    "MACRO",      SEC_OSC2,   PK_STEP, 101,  0,         63,  U_INT_PERCENT, nullptr,      0},
    {"osc2_color",    "COLOR",      SEC_OSC2,   PK_STEP, 101,  0,         114, U_INT_PERCENT, nullptr,      0},
    {"osc1_engine",   "ENGINE",     SEC_OSC1,   PK_STEP, 4,    0,         115, U_STEP,       kEngineLabels, 0},
    {"osc2_engine",   "ENGINE",     SEC_OSC2,   PK_STEP, 4,    0,         116, U_STEP,       kEngineLabels, 0},
    {"hold",          "HOLD",       SEC_VOICE,  PK_STEP, 2,    0,         69,  U_STEP,       kOnOffLabels,  0},
    {"voice_drift",   "DRIFT",      SEC_VOICE,  PK_POT,  0,    pot(0.0f), 14,  U_PERCENT,    nullptr,       0},
    {"voice_spread",  "SPREAD",     SEC_VOICE,  PK_POT,  0,    pot(0.0f), 15,  U_PERCENT,    nullptr,       0},
    {"osc_xmod",      "X-MOD",      SEC_OSC2,   PK_POT,  0,    pot(0.0f), 16,  U_PERCENT,    nullptr,       0},
    {"osc_sync",      "SYNC",       SEC_OSC2,   PK_STEP, 2,    0,         17,  U_STEP,       kOnOffLabels,  0},
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

float Params::frac(int id) const {
  int n = kParams[id].steps;
  return n > 1 ? (float)val[id] / (float)(n - 1) : 0.0f;
}

int paramForCC(int cc) {
  for (int i = 0; i < P_COUNT; ++i)
    if (kParams[i].cc == cc) return i;
  return -1;
}

float mapCutoffHz(float n) { return 20.0f * exp2f_(n * 10.0f); }

float mapResonanceQ(float n) { return 0.5f * exp2f_(n * 4.585f); }

float mapLadderFeedback(float n) { return 3.9f * n; }

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

DriveSettings mapDrive(float n) {
  // Each stage's trim keeps a signal of typical size at the same peak
  // level, so turning Drive up changes the tone and thickens the sound
  // without a jump in loudness.
  const float kPreLevel = 0.5f, kPostLevel = 0.3f;
  DriveSettings d;
  d.pre = 1.0f + 5.0f * n;
  d.preTrim = tanhf_(kPreLevel) / tanhf_(kPreLevel * d.pre);
  d.post = 1.0f + 3.0f * n + 12.0f * n * n;
  d.postTrim = tanhf_(kPostLevel) / tanhf_(kPostLevel * d.post);
  // Fully clean at zero, and no step in the sound as the knob leaves zero.
  d.mix = clampf(n * 5.0f, 0.0f, 1.0f);
  return d;
}

float driveStage(float x, float gain, float trim, float mix) { return x + mix * (tanhf_(x * gain) * trim - x); }

float mapVolume(float n) { return 1.4f * n * n; }

int mapLfoShape(float n) { return clampi((int)(n * (float)LFOS_COUNT), 0, LFOS_COUNT - 1); }

float mapDelayTime(float f) { return 0.02f * exp2f_(f * 5.2288f); }

float mapTempoBpm(float n) { return floorf_(40.5f + 200.0f * n); }

float mapArpGate(float n) { return 0.05f + 0.95f * n; }

float mapReverbTime(float f) { return 0.3f * exp2f_(f * 4.737f); }

float mapFadeTime(float f) { return f < 0.005f ? 0.0f : 0.05f * exp2f_(f * 6.644f); }

}  // namespace fw
