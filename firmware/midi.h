// MIDI: turns the raw byte streams arriving at the two inputs (the MIDI
// jack and USB) into calls on the engine and the parameter table, and
// keeps the bytes waiting to leave by the two outputs.
//
// In: notes and controllers are played if they are on the synth's channel
// (or on any, when the channel setting is ALL). Clock, start, stop,
// continue and song position are followed only from the input the Clock
// setting names.
//
// Out: with Thru on, every complete note or controller message that
// arrives is passed on: both inputs to the MIDI out jack, and the MIDI
// jack's input to USB (never USB back to USB). With Clock Out on, the clock
// the synth is running on is sent too: its own when it keeps time itself,
// the incoming one when it is following. System-exclusive messages are not
// passed on.
#pragma once

#include "fw_config.h"
#include "settings.h"

namespace fw {

class Midi {
 public:
  void init();
  // One byte as it arrives at an input.
  void byte(int port, uint8_t b);
  // Milliseconds since the last message the synth played, for the activity light.
  uint32_t idleMs() const { return idleMs_; }
  void tick(uint32_t dtMs);

  // Copies bytes waiting to leave by an output into dst (at most max) and returns how many.
  uint32_t outRead(int port, uint8_t* dst, uint32_t max);
  // Audio context: the synth's own clock has ticked (24 to a beat).
  void clockTick();

 private:
  struct Parser {
    uint8_t status;  // running status
    uint8_t data[2];
    uint8_t have;    // data bytes collected
    uint8_t need;    // data bytes this status takes
    bool inSysex;
  };
  // Bytes waiting to leave. One side only writes, the other only reads.
  static constexpr uint32_t kRingLen = 256;
  struct Ring {
    uint8_t buf[kRingLen];
    volatile uint32_t w, r;
  };

  void realTime(int port, uint8_t b);
  void message(int port, uint8_t status, uint8_t d1, uint8_t d2, int len);
  void controlChange(uint8_t cc, uint8_t value);
  void pass(int fromPort, const uint8_t* bytes, int n, bool clock);
  static bool put(Ring& ring, const uint8_t* bytes, int n);

  Parser in_[PORT_COUNT];
  Ring thru_[PORT_COUNT];   // written by the control context: what is passed on
  Ring clock_[PORT_COUNT];  // written by the audio context: the synth's own clock
  uint32_t idleMs_;
};

extern Midi gMidi;

}  // namespace fw
