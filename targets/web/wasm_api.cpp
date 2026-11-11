// Browser target: exposes the firmware to the page as a WebAssembly module.
//
// The module imports nothing. The page reaches the firmware only through
// the functions exported here, and through three shared buffers (audio out,
// MIDI in, saved state) whose addresses it asks for.
#include "../../firmware/firmware.h"

#define FW_EXPORT(name) __attribute__((export_name(#name))) name

namespace {
float gOut[fw::kMaxBlock];
uint8_t gMidiBuf[256];
uint8_t gStore[fw::kStateSize];
uint32_t gStoreLen = 0;
uint32_t gSaveCount = 0;
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

// ---- lifecycle and audio ----
void FW_EXPORT(fw_init)(float sampleRate) { fw::init(sampleRate); }
float* FW_EXPORT(fw_out_ptr)() { return gOut; }
int FW_EXPORT(fw_max_block)() { return fw::kMaxBlock; }
void FW_EXPORT(fw_process)(int n) {
  if (n > fw::kMaxBlock) n = fw::kMaxBlock;
  fw::process(gOut, n);
}

// ---- MIDI in: write bytes to the buffer, then call fw_midi(count) ----
uint8_t* FW_EXPORT(fw_midi_ptr)() { return gMidiBuf; }
int FW_EXPORT(fw_midi_capacity)() { return sizeof gMidiBuf; }
void FW_EXPORT(fw_midi)(int count) {
  if (count > (int)sizeof gMidiBuf) count = sizeof gMidiBuf;
  for (int i = 0; i < count; ++i) fw::midiByte(gMidiBuf[i]);
}

// ---- panel hardware ----
int FW_EXPORT(fw_control_count)() { return fw::kNumControls; }
const char* FW_EXPORT(fw_control_name)(int i) { return fw::controlInfo(i).name; }
int FW_EXPORT(fw_control_kind)(int i) { return fw::controlInfo(i).kind; }
int FW_EXPORT(fw_control_steps)(int i) { return fw::controlInfo(i).steps; }
void FW_EXPORT(fw_control_set)(int i, int raw) { fw::controlSet(i, raw); }
void FW_EXPORT(fw_encoder_turn)(int i, int delta) { fw::encoderTurn(i, delta); }
int FW_EXPORT(fw_control_get)(int i) { return fw::controlGet(i); }

// ---- display ----
int FW_EXPORT(fw_tick)(int dtMs) { return fw::tick((uint32_t)dtMs) ? 1 : 0; }
const uint16_t* FW_EXPORT(fw_display_ptr)() { return fw::frame(); }
int FW_EXPORT(fw_display_width)() { return fw::kDisplayW; }
int FW_EXPORT(fw_display_height)() { return fw::kDisplayH; }

}  // extern "C"
