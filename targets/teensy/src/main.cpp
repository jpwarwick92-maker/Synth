// Teensy 4.1 target: the firmware core wrapped for real hardware.
//
// For now this is a bare board with no panel: notes arrive over USB MIDI
// and the MIDI in jack (Serial1), sound leaves over USB audio and I2S (for
// the audio board later), the audio input arrives over I2S, and the
// processor load is printed once a second over USB serial. The panel
// scanning and the display driver are added by the hardware build.

#include <Arduino.h>
#include <Audio.h>
#include <EEPROM.h>

#include "firmware.h"

// Set to 1 once an audio chip with an input is fitted. Until then the input
// is left alone: an I2S input pin with nothing driving it reads as noise,
// which the level meter and the vocoder would take for a signal.
#ifndef SYNTH_HAS_AUDIO_INPUT
#define SYNTH_HAS_AUDIO_INPUT 0
#endif

static_assert(AUDIO_BLOCK_SAMPLES <= fw::kMaxBlock, "audio block larger than the firmware accepts");

static volatile bool gReady = false;

// The whole synth as one Teensy Audio Library object.
class AudioSynthFirmware : public AudioStream {
 public:
  AudioSynthFirmware() : AudioStream(2, inputs_) {}

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
    // The audio input, when the converter delivers one. The gain the
    // firmware asks for belongs in the preamp in front of the converter
    // (the audio chip's own gain setting, once that chip is chosen): applied
    // here, after the converter, it is only a stand-in and cannot undo an
    // input that has already overloaded.
    // Which input to listen to is the firmware's too: fw::inputSource() is
    // INSRC_LINE or INSRC_MIC, and the audio chip's input select follows it
    // (in loop(), where the chip is talked to).
    audio_block_t* inL = receiveReadOnly(0);
    audio_block_t* inR = receiveReadOnly(1);
    float il[AUDIO_BLOCK_SAMPLES], ir[AUDIO_BLOCK_SAMPLES];
    bool haveInput = SYNTH_HAS_AUDIO_INPUT && (inL || inR);
    if (haveInput) {
      static int lastDb = -1;
      static float gain = 1.0f;
      int db = fw::inputGainDb();
      if (db != lastDb) {
        lastDb = db;
        gain = powf(10.0f, (float)db / 20.0f);
      }
      const float k = gain / 32768.0f;
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        il[i] = inL ? (float)inL->data[i] * k : 0.0f;
        ir[i] = inR ? (float)inR->data[i] * k : il[i];
      }
    }
    if (inL) release(inL);
    if (inR) release(inR);
    // Time the block against the time there is for one. If the engine keeps
    // using nearly all of it, it sheds voices: otherwise an overload starves
    // loop(), MIDI stops being read, and the notes causing it never end.
    uint32_t start = ARM_DWT_CYCCNT;
    if (haveInput) fw::process(il, ir, l, r, AUDIO_BLOCK_SAMPLES);
    else fw::process(l, r, AUDIO_BLOCK_SAMPLES);
    static const float kBudget = (float)F_CPU_ACTUAL * (float)AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_EXACT;
    fw::reportLoad((float)(ARM_DWT_CYCCNT - start) / kBudget);
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
  audio_block_t* inputs_[2];
  static int16_t toInt16(float x) {
    float s = x * 32767.0f;
    if (s > 32767.0f) s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
  }
};

static AudioSynthFirmware synthOut;
// A plain I2S converter (PCM5102 and the like) needs nothing more than
// this. The PJRC audio shield would also need an AudioControlSGTL5000
// object and its enable() call in setup(), or it stays silent.
static AudioOutputI2S i2sOut;  // also drives the audio clock
static AudioOutputUSB usbOut;
// The audio input: line in or a microphone, whichever the audio chip is
// switched to (fw::inputSource() says which the firmware wants). A plain
// output-only converter has no input; the chip chosen for the hardware
// needs one, with a microphone preamp.
static AudioInputI2S i2sIn;
static AudioConnection patch5(i2sIn, 0, synthOut, 0);
static AudioConnection patch6(i2sIn, 1, synthOut, 1);
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

// Sends what the firmware has waiting for the USB MIDI output. The firmware
// hands over plain MIDI bytes; USB wants them a message at a time.
static void usbMidiSend(const uint8_t* bytes, uint32_t n) {
  static uint8_t status = 0, data[2], have = 0;
  for (uint32_t i = 0; i < n; ++i) {
    uint8_t b = bytes[i];
    if (b >= 0xF8) {
      usbMIDI.sendRealTime(b);
    } else if (b & 0x80) {
      status = b;
      have = 0;
    } else if (status) {
      data[have++] = b;
      uint8_t type = status & 0xF0;
      uint8_t need = (type == 0xC0 || type == 0xD0) ? 1 : 2;
      if (have < need) continue;
      have = 0;
      if (status == 0xF2) usbMIDI.sendSongPosition(data[0] | (data[1] << 7));
      else usbMIDI.send(type, data[0], need == 2 ? data[1] : 0, (status & 0x0F) + 1, 0);
    }
  }
}

void setup() {
  AudioMemory(24);
  Serial.begin(115200);
  Serial1.begin(31250);  // the MIDI in and out jacks: pins 0 (in) and 1 (out)
  fw::init(AUDIO_SAMPLE_RATE_EXACT);
  gReady = true;
}

void loop() {
  // USB MIDI in: hand the firmware the same bytes a MIDI cable would carry.
  while (usbMIDI.read()) {
    uint8_t type = usbMIDI.getType();
    if (type >= 0xF8) {  // clock, start, continue, stop
      fw::midiByte(type, fw::PORT_USB);
    } else if (type == 0xF2) {  // song position
      fw::midiByte(type, fw::PORT_USB);
      fw::midiByte(usbMIDI.getData1() & 0x7F, fw::PORT_USB);
      fw::midiByte(usbMIDI.getData2() & 0x7F, fw::PORT_USB);
    } else if (type >= 0x80 && type < 0xF0) {
      fw::midiByte((uint8_t)(type | ((usbMIDI.getChannel() - 1) & 0x0F)), fw::PORT_USB);
      fw::midiByte(usbMIDI.getData1() & 0x7F, fw::PORT_USB);
      if (type != 0xC0 && type != 0xD0) fw::midiByte(usbMIDI.getData2() & 0x7F, fw::PORT_USB);
    }
  }
  // The MIDI in jack: bytes go to the firmware exactly as they arrive.
  while (Serial1.available()) fw::midiByte((uint8_t)Serial1.read(), fw::PORT_MIDI);

  // MIDI out: thru, and clock when the firmware is sending it. The
  // firmware's own clock is made in the audio blocks, so each tick leaves
  // up to one block (2.9 ms) late; the tempo it carries is exact.
  uint8_t out[64];
  for (uint32_t n = fw::midiOutRead(fw::PORT_MIDI, out, sizeof out); n > 0; n = fw::midiOutRead(fw::PORT_MIDI, out, sizeof out))
    Serial1.write(out, n);
  for (uint32_t n = fw::midiOutRead(fw::PORT_USB, out, sizeof out); n > 0; n = fw::midiOutRead(fw::PORT_USB, out, sizeof out))
    usbMidiSend(out, n);

  // The panel, when it exists: each pot's converter reading goes to
  // fw::potReading(index, reading) on every scan, which does the smoothing.
  // Through a multiplexer, wait for the line to settle after switching
  // channel and throw away the first conversion, or each pot picks up a
  // little of the one read before it.

  // Display and soft-control timing. The frame is drawn into fw::frame();
  // pushing it to the ST7789 is added with the display hardware.
  static elapsedMillis sinceTick;
  if (sinceTick >= 33) {
    uint32_t dt = sinceTick;
    sinceTick = 0;
    fw::tick(dt);
    // The label lights: fw::ledColour(i) for each panel control goes to the
    // LED chain here, once the lighting board exists. The light by the
    // input jack is one more on the same chain: fw::inputLight().
  }

  // Processor load: the number the bench run is for.
  static elapsedMillis sinceReport;
  if (sinceReport >= 1000) {
    sinceReport = 0;
    // only when something is listening: an unread port fills up and a write then stalls loop()
    if (Serial.dtr()) Serial.printf("audio cpu %.1f%% (max %.1f%%)\n", (double)AudioProcessorUsage(), (double)AudioProcessorUsageMax());
    AudioProcessorUsageMaxReset();
  }
}
