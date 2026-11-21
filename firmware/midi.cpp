#include "midi.h"

#include "firmware.h"
#include "params.h"
#include "synth.h"

namespace fw {

Midi gMidi;

void Midi::init() {
  for (int p = 0; p < PORT_COUNT; ++p) {
    in_[p].status = 0;
    in_[p].have = 0;
    in_[p].need = 0;
    in_[p].inSysex = false;
    thru_[p].w = thru_[p].r = 0;
    clock_[p].w = clock_[p].r = 0;
    lost_[p] = false;
  }
  idleMs_ = 100000;
}

void Midi::tick(uint32_t dtMs) {
  if (idleMs_ < 100000) idleMs_ += dtMs;
  // An output that lost a message may have lost a note-off. Once it has
  // room again, every channel is told to let its notes go.
  for (int p = 0; p < PORT_COUNT; ++p) {
    if (!lost_[p] || kRingLen - (thru_[p].w - thru_[p].r) < 48u) continue;
    for (int ch = 0; ch < 16; ++ch) {
      const uint8_t off[3] = {(uint8_t)(0xB0 | ch), 123, 0};
      put(thru_[p], off, 3);
    }
    lost_[p] = false;
  }
}

void Midi::thruEnded() {
  for (int p = 0; p < PORT_COUNT; ++p) lost_[p] = true;  // tick() sends the all-notes-off
}

// All of the bytes or none: half a message must never go out.
bool Midi::put(Ring& ring, const uint8_t* bytes, int n) {
  uint32_t w = ring.w;
  if (kRingLen - (w - ring.r) < (uint32_t)n) return false;
  for (int i = 0; i < n; ++i) ring.buf[(w + (uint32_t)i) & (kRingLen - 1)] = bytes[i];
  ring.w = w + (uint32_t)n;
  return true;
}

// Sends something that arrived at an input on to the outputs. The MIDI out
// jack takes it from either input. USB takes it from the MIDI jack only:
// sending a computer its own messages back would have them going round in circles.
void Midi::pass(int fromPort, const uint8_t* bytes, int n, bool clock) {
  if (!setting(clock ? SET_CLOCK_OUT : SET_THRU)) return;
  if (!put(thru_[PORT_MIDI], bytes, n) && !clock) lost_[PORT_MIDI] = true;
  if (fromPort != PORT_USB && !put(thru_[PORT_USB], bytes, n) && !clock) lost_[PORT_USB] = true;
}

uint32_t Midi::outRead(int port, uint8_t* dst, uint32_t max) {
  if (port < 0 || port >= PORT_COUNT) return 0;
  uint32_t n = 0;
  // Clock first: a clock byte may sit anywhere in the stream, and it is the one that must not wait.
  Ring* rings[2] = {&clock_[port], &thru_[port]};
  for (Ring* ring : rings) {
    uint32_t r = ring->r, w = ring->w;
    while (r != w && n < max) dst[n++] = ring->buf[r++ & (kRingLen - 1)];
    ring->r = r;
  }
  return n;
}

void Midi::clockTick() {
  const uint8_t b = 0xF8;
  for (int p = 0; p < PORT_COUNT; ++p) put(clock_[p], &b, 1);
}

// Clock, start, continue and stop. Only the input the Clock setting names
// is listened to; clock arriving anywhere else is dropped, so that two
// clocks can never reach the outputs at once.
void Midi::realTime(int port, uint8_t b) {
  int source = setting(SET_CLOCK);
  if (!((source == CLOCK_MIDI && port == PORT_MIDI) || (source == CLOCK_USB && port == PORT_USB))) return;
  switch (b) {
    case 0xF8: gSynth.post(EV_CLOCK); break;
    case 0xFA: gSynth.post(EV_START); break;
    case 0xFB: gSynth.post(EV_CONTINUE); break;
    case 0xFC: gSynth.post(EV_STOP); break;
    default: return;  // active sensing, reset: not used
  }
  pass(port, &b, 1, true);
}

void Midi::byte(int port, uint8_t b) {
  if (port < 0 || port >= PORT_COUNT) return;
  if (b >= 0xF8) {  // real-time bytes can appear anywhere, even inside another message
    realTime(port, b);
    return;
  }
  Parser& in = in_[port];
  if (b & 0x80) {
    in.inSysex = b == 0xF0;
    in.status = 0;  // system messages cancel running status
    in.have = 0;
    if (b == 0xF2) {  // song position: two data bytes
      in.status = b;
      in.need = 2;
    } else if (b < 0xF0) {
      in.status = b;
      uint8_t type = b & 0xF0;
      in.need = (type == 0xC0 || type == 0xD0) ? 1 : 2;
    }
    return;
  }
  if (in.inSysex || in.status == 0) return;
  in.data[in.have++] = b;
  if (in.have < in.need) return;
  in.have = 0;  // running status: the next data bytes start a new message
  uint8_t status = in.status;
  if (status == 0xF2) in.status = 0;  // which system messages do not have
  message(port, status, in.data[0], in.need == 2 ? in.data[1] : 0, 1 + in.need);
}

void Midi::message(int port, uint8_t status, uint8_t d1, uint8_t d2, int len) {
  const uint8_t bytes[3] = {status, d1, d2};
  if (status == 0xF2) {  // song position, counted in sixteenth notes
    int source = setting(SET_CLOCK);
    if (!((source == CLOCK_MIDI && port == PORT_MIDI) || (source == CLOCK_USB && port == PORT_USB))) return;
    gSynth.post(EV_SONG_POS, d1, d2);
    pass(port, bytes, len, true);
    return;
  }
  pass(port, bytes, len, false);  // every channel is passed on, whichever one the synth plays
  int channel = setting(SET_CHANNEL);
  if (channel != 0 && (status & 0x0F) != channel - 1) return;
  idleMs_ = 0;
  switch (status & 0xF0) {
    case 0x90:
      if (d2 > 0) gSynth.post(EV_NOTE_ON, d1, d2);
      else gSynth.post(EV_NOTE_OFF, d1);
      break;
    case 0x80: gSynth.post(EV_NOTE_OFF, d1); break;
    case 0xE0: gSynth.setBend((float)(((int)d1 | ((int)d2 << 7)) - 8192) * (1.0f / 8192.0f)); break;
    case 0xB0: controlChange(d1, d2); break;
    case 0xC0: patchLoad(d1); break;  // program change: load that patch slot, if it holds one
    default: break;
  }
}

void Midi::controlChange(uint8_t cc, uint8_t value) {
  switch (cc) {
    case 1: gSynth.setModWheel((float)value * (1.0f / 127.0f)); return;
    case 64: gSynth.post(EV_SUSTAIN, value); return;
    case 120: gSynth.post(EV_ALL_SOUND_OFF); return;
    case 123: gSynth.post(EV_ALL_NOTES_OFF); return;
    default: break;
  }
  int id = paramForCC(cc);
  if (id < 0) return;
  if (gParams.setFromCC(id, value)) {
    gPatch.remoteSet(id);  // the control itself has not moved: it follows this value until touched
    onParamChanged(id);
  }
}

}  // namespace fw
