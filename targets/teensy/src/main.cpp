// Teensy 4.1 target: the firmware core wrapped for real hardware.
//
// For now this is a bare board with no panel: notes arrive over USB MIDI,
// sound leaves over USB audio and I2S (for the audio board later), and the
// processor load is printed once a second over USB serial. The panel
// scanning and the display driver are added by the hardware build.

#include <Arduino.h>
#include <Audio.h>
#include <EEPROM.h>

#include "firmware.h"

static_assert(AUDIO_BLOCK_SAMPLES <= fw::kMaxBlock, "audio block larger than the firmware accepts");

static volatile bool gReady = false;

// The whole synth as one Teensy Audio Library object.
class AudioSynthFirmware : public AudioStream {
 public:
  AudioSynthFirmware() : AudioStream(0, nullptr) {}

  void update() override {
    if (!gReady) return;
    audio_block_t* left = allocate();
    if (!left) return;
    audio_block_t* right = allocate();
    if (!right) {
      release(left);
      return;
    }
    float l[AUDIO_BLOCK_SAMPLES], r[AUDIO_BLOCK_SAMPLES];
    fw::process(l, r, AUDIO_BLOCK_SAMPLES);
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
      left->data[i] = toInt16(l[i]);
      right->data[i] = toInt16(r[i]);
    }
    transmit(left, 0);
    transmit(right, 1);
    release(left);
    release(right);
  }

 private:
  static int16_t toInt16(float x) {
    float s = x * 32767.0f;
    if (s > 32767.0f) s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
  }
};

static AudioSynthFirmware synthOut;
static AudioOutputI2S i2sOut;  // also drives the audio clock
static AudioOutputUSB usbOut;
static AudioConnection patch1(synthOut, 0, i2sOut, 0);
static AudioConnection patch2(synthOut, 1, i2sOut, 1);
static AudioConnection patch3(synthOut, 0, usbOut, 0);
static AudioConnection patch4(synthOut, 1, usbOut, 1);

// ---- storage: the on-board EEPROM (4284 bytes on a Teensy 4.1) ----
// Saved state: two length bytes, then the blob, in the first 512 bytes.
// Patches: sixteen slots of 160 bytes after that, each one length byte and
// the patch. A memory card or the board's flash file store would hold far
// more; that choice belongs to the hardware build.
static const int kStateRegion = 512, kPatchRegion = 160;
static_assert(fw::kStateSize + 2 <= kStateRegion, "saved state no longer fits its EEPROM region");
static_assert(fw::kPatchSize + 1 <= kPatchRegion && fw::kPatchSize <= 255, "a patch no longer fits its EEPROM slot");
static_assert(kStateRegion + fw::kPatchSlots * kPatchRegion <= 4284, "patch slots do not fit the EEPROM");

extern "C" uint32_t fw_hal_storage_load(uint8_t* dst, uint32_t max) {
  uint32_t n = EEPROM.read(0) | ((uint32_t)EEPROM.read(1) << 8);
  if (n == 0 || n > max || n + 2 > (uint32_t)kStateRegion) return 0;
  for (uint32_t i = 0; i < n; ++i) dst[i] = EEPROM.read(2 + i);
  return n;
}

extern "C" void fw_hal_storage_save(const uint8_t* src, uint32_t len) {
  if (len + 2 > (uint32_t)kStateRegion) return;
  EEPROM.update(0, (uint8_t)(len & 0xFF));
  EEPROM.update(1, (uint8_t)(len >> 8));
  for (uint32_t i = 0; i < len; ++i) EEPROM.update(2 + i, src[i]);
}

extern "C" uint32_t fw_hal_patch_load(uint32_t slot, uint8_t* dst, uint32_t max) {
  if (slot >= (uint32_t)fw::kPatchSlots) return 0;
  int base = kStateRegion + (int)slot * kPatchRegion;
  uint32_t n = EEPROM.read(base);
  if (n == 0 || n > max || n + 1 > (uint32_t)kPatchRegion) return 0;
  for (uint32_t i = 0; i < n; ++i) dst[i] = EEPROM.read(base + 1 + i);
  return n;
}

extern "C" void fw_hal_patch_save(uint32_t slot, const uint8_t* src, uint32_t len) {
  if (slot >= (uint32_t)fw::kPatchSlots || len + 1 > (uint32_t)kPatchRegion) return;
  int base = kStateRegion + (int)slot * kPatchRegion;
  EEPROM.update(base, (uint8_t)len);
  for (uint32_t i = 0; i < len; ++i) EEPROM.update(base + 1 + i, src[i]);
}

void setup() {
  AudioMemory(16);
  Serial.begin(115200);
  fw::init(AUDIO_SAMPLE_RATE_EXACT);
  gReady = true;
}

void loop() {
  // USB MIDI in: hand the firmware the same bytes a MIDI cable would carry.
  while (usbMIDI.read()) {
    uint8_t type = usbMIDI.getType();
    if (type < 0x80 || type >= 0xF0) continue;
    fw::midiByte((uint8_t)(type | ((usbMIDI.getChannel() - 1) & 0x0F)));
    fw::midiByte(usbMIDI.getData1() & 0x7F);
    if (type != 0xC0 && type != 0xD0) fw::midiByte(usbMIDI.getData2() & 0x7F);
  }

  // Display and soft-control timing. The frame is drawn into fw::frame();
  // pushing it to the ST7789 is added with the display hardware.
  static elapsedMillis sinceTick;
  if (sinceTick >= 33) {
    uint32_t dt = sinceTick;
    sinceTick = 0;
    fw::tick(dt);
    // The label lights: fw::ledColour(i) for each panel control goes to the
    // LED chain here, once the lighting board exists.
  }

  // Processor load: the number the bench run is for.
  static elapsedMillis sinceReport;
  if (sinceReport >= 1000) {
    sinceReport = 0;
    Serial.printf("audio cpu %.1f%% (max %.1f%%)\n", (double)AudioProcessorUsage(), (double)AudioProcessorUsageMax());
    AudioProcessorUsageMaxReset();
  }
}
