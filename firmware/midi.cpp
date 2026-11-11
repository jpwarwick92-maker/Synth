#include "midi.h"

#include "params.h"
#include "synth.h"
#include "firmware.h"

namespace fw {

MidiIn gMidi;

void MidiIn::init() {
  status_ = 0;
  have_ = 0;
  need_ = 0;
  inSysex_ = false;
  idleMs_ = 100000;
}

void MidiIn::tick(uint32_t dtMs) {
  if (idleMs_ < 100000) idleMs_ += dtMs;
}

void MidiIn::byte(uint8_t b) {
  if (b >= 0xF8) return;  // real-time bytes can appear anywhere; ignore them
  if (b & 0x80) {
    if (b == 0xF0) {
      inSysex_ = true;
      status_ = 0;
      return;
    }
    inSysex_ = false;
    if (b >= 0xF0) {  // other system messages: not used, and they cancel running status
      status_ = 0;
      return;
    }
    status_ = b;
    have_ = 0;
    uint8_t type = b & 0xF0;
    need_ = (type == 0xC0 || type == 0xD0) ? 1 : 2;
    return;
  }
  if (inSysex_ || status_ == 0) return;
  data_[have_++] = b;
  if (have_ < need_) return;
  have_ = 0;  // running status: the next data bytes start a new message
  message(status_, data_[0], need_ == 2 ? data_[1] : 0);
}

void MidiIn::message(uint8_t status, uint8_t d1, uint8_t d2) {
  idleMs_ = 0;
  switch (status & 0xF0) {
    case 0x90:
      if (d2 > 0) gSynth.post(EV_NOTE_ON, d1, d2);
      else gSynth.post(EV_NOTE_OFF, d1);
      break;
    case 0x80: gSynth.post(EV_NOTE_OFF, d1); break;
    case 0xE0: gSynth.post(EV_BEND, d1, d2); break;
    case 0xB0: controlChange(d1, d2); break;
    default: break;
  }
}

void MidiIn::controlChange(uint8_t cc, uint8_t value) {
  switch (cc) {
    case 1: gSynth.post(EV_MOD_WHEEL, value); return;
    case 64: gSynth.post(EV_SUSTAIN, value); return;
    case 120: gSynth.post(EV_ALL_SOUND_OFF); return;
    case 123: gSynth.post(EV_ALL_NOTES_OFF); return;
    default: break;
  }
  int id = paramForCC(cc);
  if (id < 0) return;
  if (gParams.setFromCC(id, value)) onParamChanged(id);
}

}  // namespace fw
