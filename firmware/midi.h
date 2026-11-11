// MIDI input: turns a raw byte stream into calls on the engine and the
// parameter table. Listens on every channel.
#pragma once

#include "fw_config.h"

namespace fw {

class MidiIn {
 public:
  void init();
  // Feed one byte as it arrives from the keyboard.
  void byte(uint8_t b);
  // Milliseconds since the last message, for the activity light.
  uint32_t idleMs() const { return idleMs_; }
  void tick(uint32_t dtMs);

 private:
  void message(uint8_t status, uint8_t d1, uint8_t d2);
  void controlChange(uint8_t cc, uint8_t value);

  uint8_t status_;   // running status
  uint8_t data_[2];
  uint8_t have_;     // data bytes collected
  uint8_t need_;     // data bytes this status takes
  bool inSysex_;
  uint32_t idleMs_;
};

extern MidiIn gMidi;

}  // namespace fw
