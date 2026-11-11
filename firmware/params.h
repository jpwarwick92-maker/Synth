// The parameter table: the single list of everything the synth can be set to.
//
// The engine, the panel logic, the display and the MIDI controller mapping
// all read from this one table. A parameter is either a pot (a 12-bit raw
// position, 0..4095, exactly what an ADC would report) or a stepped value
// (an index, 0..steps-1, exactly what a rotary switch would report).
#pragma once

#include "fw_config.h"

namespace fw {

enum ParamId : uint8_t {
  // ---- front panel, layout A2 ----
  P_OSC1_WAVE,
  P_OSC1_OCTAVE,
  P_OSC1_MACRO,
  P_OSC2_WAVE,
  P_OSC2_OCTAVE,
  P_OSC2_DETUNE,
  P_MIX_OSC1,
  P_MIX_OSC2,
  P_MIX_SUB,
  P_MIX_NOISE,
  P_CUTOFF,
  P_RESONANCE,
  P_FILT_ENV_AMT,
  P_KEYTRACK,
  P_DRIVE,
  P_FILT_TYPE,
  P_GLIDE,
  P_VOICE_MODE,
  P_FENV_A,
  P_FENV_D,
  P_FENV_S,
  P_FENV_R,
  P_AENV_A,
  P_AENV_D,
  P_AENV_S,
  P_AENV_R,
  P_VELOCITY,
  P_LFO_RATE,
  P_LFO_DEPTH,
  P_LFO_SHAPE,
  P_LFO_DEST,
  // ---- rear panel ----
  P_VOLUME,
  // ---- soft controls (Engine page) ----
  P_UNI_DETUNE,
  P_SUB_OCTAVE,
  P_BEND_RANGE,
  P_FILT_SLOPE,

  P_COUNT,
  P_PANEL_COUNT = P_UNI_DETUNE,  // params with their own physical control
};

enum Section : uint8_t {
  SEC_OSC1,
  SEC_OSC2,
  SEC_MIXER,
  SEC_FILTER,
  SEC_VOICE,
  SEC_FENV,
  SEC_AENV,
  SEC_LFO,
  SEC_OUTPUT,
  SEC_ENGINE,
  SEC_COUNT,
};

enum ParamKind : uint8_t { PK_POT, PK_STEP };

// How a value is shown on the display.
enum Unit : uint8_t {
  U_PERCENT,    // 0..100%
  U_STEP,       // one of the step labels
  U_PW,         // pulse width %
  U_CENTS,      // bipolar detune in cents
  U_CUTOFF,     // Hz
  U_ENVAMT,     // bipolar octaves
  U_TIME,       // envelope time
  U_LFOHZ,      // LFO rate
  U_GLIDE,      // glide time
  U_LFOSHAPE,   // one of five shapes
  U_INT_CENTS,  // integer steps shown as cents
  U_INT_SEMI,   // integer steps shown as semitones
};

enum Wave : uint8_t { WAVE_SAW, WAVE_SQR, WAVE_TRI, WAVE_SIN };
enum FilterType : uint8_t { FILT_LP, FILT_BP, FILT_HP };
enum VoiceMode : uint8_t { MODE_POLY, MODE_MONO, MODE_UNI };
enum LfoDest : uint8_t { LFO_PITCH, LFO_CUTOFF, LFO_AMP, LFO_PW };
enum LfoShape : uint8_t { LFOS_SINE, LFOS_TRI, LFOS_SAW, LFOS_SQUARE, LFOS_RANDOM, LFOS_COUNT };

constexpr int kPotMax = 4095;

struct ParamDef {
  const char* key;      // stable id; also the name of the physical control
  const char* name;     // shown on the display
  uint8_t section;
  uint8_t kind;         // ParamKind
  uint8_t steps;        // PK_STEP only
  uint16_t def;         // default raw value or step index
  uint8_t cc;           // MIDI controller number
  uint8_t unit;         // Unit
  const char* const* labels;  // step labels, or null
  int8_t intMin;        // U_INT_*: value shown for step 0
};

extern const ParamDef kParams[P_COUNT];
extern const char* const kSectionNames[SEC_COUNT];
extern const char* const kLfoShapeNames[LFOS_COUNT];

struct Params {
  volatile uint16_t val[P_COUNT];

  void reset();
  // Both return true when the stored value changed.
  bool set(int id, int raw);
  bool setFromCC(int id, int value7);

  float norm(int id) const { return (float)val[id] * (1.0f / (float)kPotMax); }
  int step(int id) const { return val[id]; }
};

extern Params gParams;

// Parameter with this MIDI controller number, or -1.
int paramForCC(int cc);

// ---- value mappings, shared by the engine and the display ----
float mapCutoffHz(float n);       // 20 Hz .. 20.5 kHz
float mapResonanceQ(float n);     // 0.5 .. 12
float mapEnvAmtOct(float n);      // -6 .. +6 octaves, dead zone at centre
float mapEnvTime(float n);        // 1 ms .. 10 s
float mapLfoHz(float n);          // 0.05 .. 30 Hz
float mapGlideTime(float n);      // 0 .. 3 s
float mapDetuneCents(float n);    // -100 .. +100 cents, fine near centre
float mapPulseWidth(float n);     // 0.5 .. 0.95
float mapDriveGain(float n);      // 1 .. 16
float mapVolume(float n);         // output gain
int mapLfoShape(float n);         // LfoShape

}  // namespace fw
