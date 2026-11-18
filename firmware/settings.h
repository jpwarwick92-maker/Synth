// Settings about the instrument itself and how it connects to other gear.
//
// These are not part of a patch: loading a patch never changes them. They
// are kept with the saved state, one byte each, in the order below. New
// ones are only ever added at the end, so older saved state still loads.
#pragma once

#include "fw_config.h"

namespace fw {

enum SettingId : uint8_t {
  SET_CLOCK,         // what keeps time: the synth itself, or clock arriving at a MIDI input
  SET_CHANNEL,       // the MIDI channel the synth plays: 0 = every channel, 1..16
  SET_THRU,          // pass what arrives at the MIDI inputs on to the MIDI outputs
  SET_CLOCK_OUT,     // send the clock the synth is running on to the MIDI outputs
  SET_INPUT_SOURCE,  // the audio input: line level or a microphone
  SET_INPUT_GAIN,    // gain in front of the input's converter, in dB
  SET_INPUT_TO,      // where the audio input goes
  SET_COUNT,
};

// The two MIDI connections. Each is an input and an output.
enum MidiPort : uint8_t { PORT_MIDI, PORT_USB, PORT_COUNT };  // the MIDI jacks, and USB

enum ClockSource : uint8_t { CLOCK_INTERNAL, CLOCK_MIDI, CLOCK_USB, CLOCK_COUNT };
enum InputSource : uint8_t { INSRC_LINE, INSRC_MIC, INSRC_COUNT };
enum InputTo : uint8_t {
  INPUT_OFF,
  INPUT_MIX,  // straight to the output, beside the synth's own sound
  INPUT_FX,   // into the effects, with the synth's own sound
  INPUT_COUNT,
};
constexpr int kInputGainMaxDb = 60;

struct SettingDef {
  const char* name;
  uint8_t count;   // how many values it has
  uint8_t def;
  const char* const* labels;  // one per value, or nullptr for a number
};
extern const SettingDef kSettings[SET_COUNT];

// Every setting at its default.
void settingsReset();
int setting(int id);
// Clamps to the setting's range. Returns true if the value changed (the saved state is then marked for saving).
bool settingSet(int id, int value);
// The value as the display shows it. `dst` holds at least 8 characters.
const char* settingText(int id, char* dst);

}  // namespace fw
