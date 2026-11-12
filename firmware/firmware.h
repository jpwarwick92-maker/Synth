// The firmware's top-level interface. A target (the browser page, the
// Teensy, the test runner) only ever calls these functions.
//
// Hardware model: every physical control on the panel has an index and a
// name. Pots and faders report a raw 12-bit position (0..4095), selectors
// report a stop index, buttons report down/up, encoders report clicks.
// What a control *does* is decided here, never by the hardware side.
#pragma once

#include "fw_config.h"
#include "params.h"

namespace fw {

enum ControlKind : uint8_t { CK_POT, CK_SELECTOR, CK_ENCODER, CK_BUTTON };

struct ControlInfo {
  const char* name;
  uint8_t kind;
  uint8_t steps;  // selectors only
};

constexpr int kNumEncoders = 4;
constexpr int kNumButtons = 9;  // five mode buttons, two oscillator engine buttons, Hold, Chord
constexpr int kFirstEncoder = kPanelCount;
constexpr int kFirstButton = kFirstEncoder + kNumEncoders;
constexpr int kNumControls = kFirstButton + kNumButtons;

// Start-up: builds tables, loads saved state, resets the engine.
void init(float sampleRate);

// Audio context: fills both channels with n samples (n <= kMaxBlock).
void process(float* outL, float* outR, int n);

// ---- control context ----
void midiByte(uint8_t b);

ControlInfo controlInfo(int index);
// The parameter a pot, fader, selector or switch sets, or -1 for encoders and buttons.
int controlParam(int index);
// Pot: raw position. Selector: stop index. Button or encoder push: 1 = down, 0 = up.
void controlSet(int index, int raw);
// Encoder turned by `delta` clicks (positive = clockwise).
void encoderTurn(int index, int delta);
// What the control shows: pot position, selector stop, button light (0/1),
// or the encoder's running click count.
int controlGet(int index);

// Call about 30 times a second with the elapsed time. Returns true when the
// display buffer has new content.
bool tick(uint32_t dtMs);
// The display frame: 320x240, 4 bits per pixel (a colour number), two
// pixels to a byte. frameRow565 expands one row to the display's native
// 16-bit format, so a driver can send it a line at a time.
const uint8_t* frame();
void frameRow565(int y, uint16_t* dst);

// ---- used inside the core ----
void onParamChanged(int id);

// Saved state: a small versioned blob of every parameter. A blob from an
// older build, with fewer parameters, still loads: the ones it lacks keep
// their defaults.
constexpr uint32_t kStateSize = 6 + P_COUNT * 2 + 2;
uint32_t stateWrite(uint8_t* dst);
bool stateRead(const uint8_t* src, uint32_t len);

}  // namespace fw

// ---- storage, provided by each target ----
// Copies up to `max` saved bytes into dst and returns how many (0 = none).
extern "C" uint32_t fw_hal_storage_load(uint8_t* dst, uint32_t max);
// Stores the bytes somewhere that survives a restart.
extern "C" void fw_hal_storage_save(const uint8_t* src, uint32_t len);
