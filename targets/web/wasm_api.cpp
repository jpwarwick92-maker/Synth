// Browser target: exposes the firmware to the page as a WebAssembly module.
//
// The module imports nothing. The page reaches the firmware only through
// the functions exported here, and through shared buffers (audio in and
// out, MIDI in and out, saved state, patch slots) whose addresses it asks for.
#include "../../firmware/firmware.h"

#define FW_EXPORT(name) __attribute__((export_name(#name))) name

namespace {
float gOutL[fw::kMaxBlock];
float gOutR[fw::kMaxBlock];
float gInL[fw::kMaxBlock];
float gInR[fw::kMaxBlock];
uint8_t gMidiBuf[256];
uint8_t gMidiOut[256];
// The page is handed the frame already expanded to the display's 16-bit
// format, as the real display would receive it line by line.
uint16_t gDisplay[fw::kDisplayW * fw::kDisplayH];
uint8_t gStore[fw::kStateSize];
uint32_t gStoreLen = 0;
uint32_t gSaveCount = 0;
// The patch slots, as a flash chip or memory card would hold them.
uint8_t gBank[fw::kPatchSlots][fw::kPatchSize];
uint32_t gBankLen[fw::kPatchSlots];
uint32_t gPatchSaveCount = 0;
}  // namespace

extern "C" {

// ---- storage: the page fills the buffer before fw_init, and reads it back
// whenever the save counter goes up ----
uint32_t fw_hal_storage_load(uint8_t* dst, uint32_t max) {
  uint32_t n = gStoreLen < max ? gStoreLen : max;
  for (uint32_t i = 0; i < n; ++i) dst[i] = gStore[i];
  return n;
}

void fw_hal_storage_save(const uint8_t* src, uint32_t len) {
  if (len > sizeof gStore) len = sizeof gStore;
  for (uint32_t i = 0; i < len; ++i) gStore[i] = src[i];
  gStoreLen = len;
  ++gSaveCount;
}

uint8_t* FW_EXPORT(fw_storage_ptr)() { return gStore; }
uint32_t FW_EXPORT(fw_storage_capacity)() { return sizeof gStore; }
uint32_t FW_EXPORT(fw_storage_len)() { return gStoreLen; }
void FW_EXPORT(fw_storage_set_len)(uint32_t n) { gStoreLen = n <= sizeof gStore ? n : 0; }
uint32_t FW_EXPORT(fw_storage_save_count)() { return gSaveCount; }

// ---- patch slots: filled by the page before fw_init, read back when the
// patch save counter goes up ----
uint32_t fw_hal_patch_load(uint32_t slot, uint8_t* dst, uint32_t max) {
  if (slot >= (uint32_t)fw::kPatchSlots) return 0;
  uint32_t n = gBankLen[slot] < max ? gBankLen[slot] : max;
  for (uint32_t i = 0; i < n; ++i) dst[i] = gBank[slot][i];
  return n;
}

void fw_hal_patch_save(uint32_t slot, const uint8_t* src, uint32_t len) {
  if (slot >= (uint32_t)fw::kPatchSlots || len > fw::kPatchSize) return;
  for (uint32_t i = 0; i < len; ++i) gBank[slot][i] = src[i];
  gBankLen[slot] = len;
  ++gPatchSaveCount;
}

int FW_EXPORT(fw_patch_slots)() { return fw::kPatchSlots; }
uint32_t FW_EXPORT(fw_patch_capacity)() { return fw::kPatchSize; }
uint8_t* FW_EXPORT(fw_patch_ptr)(uint32_t slot) { return gBank[slot < (uint32_t)fw::kPatchSlots ? slot : 0]; }
uint32_t FW_EXPORT(fw_patch_len)(uint32_t slot) { return slot < (uint32_t)fw::kPatchSlots ? gBankLen[slot] : 0; }
void FW_EXPORT(fw_patch_set_len)(uint32_t slot, uint32_t n) {
  if (slot < (uint32_t)fw::kPatchSlots) gBankLen[slot] = n <= fw::kPatchSize ? n : 0;
}
uint32_t FW_EXPORT(fw_patch_save_count)() { return gPatchSaveCount; }

// ---- lifecycle and audio ----
void FW_EXPORT(fw_init)(float sampleRate) {
  fw::saveWhilePlaying(true);  // the page's storage costs the audio nothing
  fw::init(sampleRate);
}
float* FW_EXPORT(fw_out_left_ptr)() { return gOutL; }
float* FW_EXPORT(fw_out_right_ptr)() { return gOutR; }
int FW_EXPORT(fw_max_block)() { return fw::kMaxBlock; }
void FW_EXPORT(fw_process)(int n) {
  if (n > fw::kMaxBlock) n = fw::kMaxBlock;
  fw::process(gOutL, gOutR, n);
}

// ---- audio in: write the input's samples to the two buffers, as the
// converter would deliver them, then call fw_process_in(n) in place of fw_process(n) ----
float* FW_EXPORT(fw_in_left_ptr)() { return gInL; }
float* FW_EXPORT(fw_in_right_ptr)() { return gInR; }
void FW_EXPORT(fw_process_in)(int n) {
  if (n > fw::kMaxBlock) n = fw::kMaxBlock;
  fw::process(gInL, gInR, gOutL, gOutR, n);
}
// What the firmware asks of the hardware in front of the converter, and the light by the jack.
int FW_EXPORT(fw_input_gain_db)() { return fw::inputGainDb(); }
int FW_EXPORT(fw_input_source)() { return fw::inputSource(); }
uint32_t FW_EXPORT(fw_input_light)() { return fw::inputLight(); }

// ---- MIDI in: write bytes to the buffer, then call fw_midi(count), or
// fw_midi_port(count, port) to say which input they arrived at (0 = the MIDI jack, 1 = USB) ----
uint8_t* FW_EXPORT(fw_midi_ptr)() { return gMidiBuf; }
int FW_EXPORT(fw_midi_capacity)() { return sizeof gMidiBuf; }
void FW_EXPORT(fw_midi_port)(int count, int port) {
  if (count > (int)sizeof gMidiBuf) count = sizeof gMidiBuf;
  for (int i = 0; i < count; ++i) fw::midiByte(gMidiBuf[i], port);
}
void FW_EXPORT(fw_midi)(int count) { fw_midi_port(count, fw::PORT_MIDI); }
// The channel the synth plays (0 = all), so that the page's stand-in keyboard can be on it.
int FW_EXPORT(fw_midi_channel)() { return fw::midiChannel(); }

// ---- MIDI out: fw_midi_out(port) copies the bytes waiting to leave by
// that output into the buffer and returns how many (0 = nothing more) ----
uint8_t* FW_EXPORT(fw_midi_out_ptr)() { return gMidiOut; }
int FW_EXPORT(fw_midi_out)(int port) { return (int)fw::midiOutRead(port, gMidiOut, sizeof gMidiOut); }

// ---- panel hardware ----
int FW_EXPORT(fw_control_count)() { return fw::kNumControls; }
const char* FW_EXPORT(fw_control_name)(int i) { return fw::controlInfo(i).name; }
int FW_EXPORT(fw_control_kind)(int i) { return fw::controlInfo(i).kind; }
int FW_EXPORT(fw_control_steps)(int i) { return fw::controlInfo(i).steps; }
void FW_EXPORT(fw_control_set)(int i, int raw) { fw::controlSet(i, raw); }
// A pot as a converter would read it, jitter and all (the page's "rough pots" stand-in for real hardware).
void FW_EXPORT(fw_pot_reading)(int i, int raw) { fw::potReading(i, raw); }
void FW_EXPORT(fw_encoder_turn)(int i, int delta) { fw::encoderTurn(i, delta); }
int FW_EXPORT(fw_control_get)(int i) { return fw::controlGet(i); }
// The light behind a control's label, 0xRRGGBB, as an LED driver would be sent it.
uint32_t FW_EXPORT(fw_led_rgb)(int i) { return fw::ledColour(i); }

// ---- spare-time work (preparing a changed drawing): call while it returns 1 ----
int FW_EXPORT(fw_work)() { return fw::work() ? 1 : 0; }

// ---- display ----
int FW_EXPORT(fw_tick)(int dtMs) {
  if (!fw::tick((uint32_t)dtMs)) return 0;
  for (int y = 0; y < fw::kDisplayH; ++y) fw::frameRow565(y, gDisplay + y * fw::kDisplayW);
  return 1;
}
const uint16_t* FW_EXPORT(fw_display_ptr)() { return gDisplay; }
int FW_EXPORT(fw_display_width)() { return fw::kDisplayW; }
int FW_EXPORT(fw_display_height)() { return fw::kDisplayH; }

}  // extern "C"
