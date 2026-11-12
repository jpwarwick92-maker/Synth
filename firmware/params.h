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
  // ---- soft controls. New ones are only ever added at the end, so saved
  // state from an older build still lines up. ----
  // Engine page 1: voice
  P_UNI_DETUNE,
  P_SUB_OCTAVE,
  P_BEND_RANGE,
  P_FILT_SLOPE,
  // FX 1: delay
  P_DLY_TIME,
  P_DLY_TONE,
  P_DLY_FDBK,
  P_DLY_MIX,
  // FX 2: chorus and reverb
  P_CHORUS,
  P_RVB_SIZE,
  P_RVB_TONE,
  P_RVB_MIX,
  // Mod
  P_LFO_FADE,
  P_VEL_CUTOFF,
  P_FENV_PITCH,
  P_LFO_SYNC,
  // Arp
  P_ARP_MODE,
  P_ARP_TEMPO,
  P_ARP_OCTAVES,
  P_ARP_GATE,
  // Engine page 2: oscillator engines
  P_OSC1_COLOR,
  P_OSC2_MACRO,
  P_OSC2_COLOR,
  // set by buttons
  P_OSC1_ENGINE,
  P_OSC2_ENGINE,
  P_HOLD,
  // panel controls added with layout A3
  P_DRIFT,
  P_SPREAD,
  P_XMOD,
  P_SYNC,

  P_COUNT,
};

// The parameters that have their own knob, fader, selector or switch on the
// panel, in control order. A control's index is its place in this list.
constexpr int kPanelCount = 40;
extern const uint8_t kPanelParams[kPanelCount];
bool isPanelParam(int id);

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
  SEC_FX1,
  SEC_FX2,
  SEC_MOD,
  SEC_ARP,
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
  U_INT_CENTS,  // integer steps shown as cents
  U_INT_SEMI,   // integer steps shown as semitones
  U_INT_PERCENT,  // stepped 0..100%
  U_INT_PLAIN,    // integer
  U_BPM,
  U_GATE,         // arp gate, 5..100%
  U_DLY_TIME,
  U_RVB_TIME,
  U_FADE_TIME,
};

enum Wave : uint8_t { WAVE_SAW, WAVE_SQR, WAVE_TRI, WAVE_SIN, WAVE_SUPER, WAVE_COUNT };
enum FilterType : uint8_t { FILT_LP, FILT_BP, FILT_HP };
enum VoiceMode : uint8_t { MODE_POLY, MODE_MONO, MODE_UNI };
enum LfoDest : uint8_t { LFO_PITCH, LFO_CUTOFF, LFO_AMP, LFO_PW };
enum LfoShape : uint8_t { LFOS_SINE, LFOS_TRI, LFOS_SAW, LFOS_SQUARE, LFOS_RANDOM, LFOS_COUNT };
enum Engine : uint8_t { ENG_VA, ENG_FM, ENG_WAVETABLE, ENG_PLUCK, ENG_COUNT };
enum ArpMode : uint8_t { ARP_OFF, ARP_UP, ARP_DOWN, ARP_UPDOWN, ARP_RANDOM, ARP_PLAYED };

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
  // A stepped parameter as a fraction of its range, 0..1.
  float frac(int id) const;
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
// Drive saturates the sound twice: going into the filter, and again coming
// out of it, where the harmonics it adds are not filtered away.
struct DriveSettings {
  float pre, preTrim;    // gain into the first saturator, and the level trim after it
  float post, postTrim;  // the same for the one after the filter
  float mix;             // 0..1: fades the saturators in over the first part of the knob
};
DriveSettings mapDrive(float n);
// One sample through a saturator stage as mapDrive describes it.
float driveStage(float x, float gain, float trim, float mix);
float mapVolume(float n);         // output gain
int mapLfoShape(float n);         // LfoShape from an old pot position (the Shape control was once a knob)
float mapDelayTime(float f);      // 20 ms .. 750 ms
float mapReverbTime(float f);     // 0.3 .. 8 s
float mapFadeTime(float f);       // 0 .. 5 s

// What an oscillator's Wave selector, Macro and Color mean under each engine.
extern const char* const kEngineWaveName[ENG_COUNT];
extern const char* const kEngineMacroName[ENG_COUNT];
extern const char* const kEngineColorName[ENG_COUNT];   // "" = this engine has none
extern const char* const kEngineWaveLabels[ENG_COUNT][WAVE_COUNT];
float mapTempoBpm(float n);       // 40 .. 240
float mapArpGate(float n);        // 0.05 .. 1 of a step

}  // namespace fw
