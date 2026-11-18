#include "settings.h"

#include "firmware.h"

namespace fw {

namespace {

const char* const kClockLabels[CLOCK_COUNT] = {"INT", "MIDI", "USB"};
const char* const kOffOn[2] = {"OFF", "ON"};
const char* const kSourceLabels[INSRC_COUNT] = {"LINE", "MIC"};
const char* const kInputToLabels[INPUT_COUNT] = {"OFF", "MIX", "FX"};

// Read by the audio context too: single bytes, so a read is always a whole value.
volatile uint8_t gValue[SET_COUNT];

}  // namespace

const SettingDef kSettings[SET_COUNT] = {
    {"CLOCK", CLOCK_COUNT, CLOCK_INTERNAL, kClockLabels},
    {"CHANNEL", 17, 0, nullptr},
    {"THRU", 2, 0, kOffOn},
    {"CLOCK OUT", 2, 0, kOffOn},
    {"SOURCE", INSRC_COUNT, INSRC_LINE, kSourceLabels},
    {"GAIN", kInputGainMaxDb + 1, 0, nullptr},
    {"SEND TO", INPUT_COUNT, INPUT_OFF, kInputToLabels},
};

void settingsReset() {
  for (int i = 0; i < SET_COUNT; ++i) gValue[i] = kSettings[i].def;
}

int setting(int id) { return id >= 0 && id < SET_COUNT ? gValue[id] : 0; }

bool settingSet(int id, int value) {
  if (id < 0 || id >= SET_COUNT) return false;
  int hi = kSettings[id].count - 1;
  value = value < 0 ? 0 : value > hi ? hi : value;
  if (value == gValue[id]) return false;
  gValue[id] = (uint8_t)value;
  stateTouched();
  return true;
}

const char* settingText(int id, char* dst) {
  dst[0] = 0;
  if (id < 0 || id >= SET_COUNT) return dst;
  int v = gValue[id];
  if (kSettings[id].labels) return kSettings[id].labels[v];
  if (id == SET_CHANNEL && v == 0) return "ALL";
  char* p = dst;
  if (id == SET_INPUT_GAIN) *p++ = '+';
  if (v >= 10) *p++ = (char)('0' + v / 10);
  *p++ = (char)('0' + v % 10);
  if (id == SET_INPUT_GAIN) {
    *p++ = ' ';
    *p++ = 'D';
    *p++ = 'B';
  }
  *p = 0;
  return dst;
}

}  // namespace fw
