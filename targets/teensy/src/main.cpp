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
    audio_block_t* block = allocate();
    if (!block) return;
    float buf[AUDIO_BLOCK_SAMPLES];
    fw::process(buf, AUDIO_BLOCK_SAMPLES);
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
      float s = buf[i] * 32767.0f;
      if (s > 32767.0f) s = 32767.0f;
      if (s < -32768.0f) s = -32768.0f;
      block->data[i] = (int16_t)s;
    }
    transmit(block, 0);
    release(block);
  }
};

static AudioSynthFirmware synthOut;
static AudioOutputI2S i2sOut;  // also drives the audio clock
static AudioOutputUSB usbOut;
static AudioConnection patch1(synthOut, 0, i2sOut, 0);
static AudioConnection patch2(synthOut, 0, i2sOut, 1);
static AudioConnection patch3(synthOut, 0, usbOut, 0);
static AudioConnection patch4(synthOut, 0, usbOut, 1);

// ---- storage: the saved state lives in the on-board EEPROM ----
// Layout: one length byte, then the blob.
extern "C" uint32_t fw_hal_storage_load(uint8_t* dst, uint32_t max) {
  uint32_t n = EEPROM.read(0);
  if (n == 0 || n > max) return 0;
  for (uint32_t i = 0; i < n; ++i) dst[i] = EEPROM.read(1 + i);
  return n;
}

extern "C" void fw_hal_storage_save(const uint8_t* src, uint32_t len) {
  if (len > 255) return;
  EEPROM.update(0, (uint8_t)len);
  for (uint32_t i = 0; i < len; ++i) EEPROM.update(1 + i, src[i]);
}

void setup() {
  AudioMemory(12);
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
  }

  // Processor load: the number the bench run is for.
  static elapsedMillis sinceReport;
  if (sinceReport >= 1000) {
    sinceReport = 0;
    Serial.printf("audio cpu %.1f%% (max %.1f%%)\n", (double)AudioProcessorUsage(), (double)AudioProcessorUsageMax());
    AudioProcessorUsageMaxReset();
  }
}
