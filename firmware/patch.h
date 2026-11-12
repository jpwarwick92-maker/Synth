// Patches, and how the panel takes over from one.
//
// Every main control on the panel is a real pot, fader or switch, so it
// cannot move by itself when a patch is loaded. Instead:
//
//  - Loading a patch sets every parameter from the stored values. A control
//    whose physical position differs is then "following": the sound uses
//    the stored value, not the control.
//  - A following pot or fader takes over ("goes live") only when it is moved
//    through the stored value, or lands within a small tolerance of it, so
//    the sound never jumps. A selector or switch goes live on any change.
//  - Each control has a light behind its label. After a load the lights
//    show which controls are off the patch and which way to move them.
//
// The firmware keeps the last physical position of every control, which is
// also what a control "shows" (controlGet). A MIDI controller message for a
// panel parameter is treated like a one-control load.
#pragma once

#include "fw_config.h"
#include "params.h"

namespace fw {

constexpr int kPatchSlots = 16;
constexpr int kPatchNameLen = 8;
// One stored patch: "PAT", version, parameter count, name, every value, checksum.
constexpr uint32_t kPatchSize = 5 + kPatchNameLen + P_COUNT * 2 + 2;

// What the label lights do for controls that are off the loaded patch.
enum GuideMode : uint8_t {
  GUIDE_OFF,   // nothing: every label is plain
  GUIDE_TINT,  // a steady tint on any control still following the patch
  GUIDE_ON,    // colour and blink show which way to move each control
  GUIDE_COUNT,
};

// Label light colours, 0xRRGGBB at the brightness the LED is driven.
constexpr uint32_t kLedIdle = 0x96876A;  // dim warm white
constexpr uint32_t kLedTint = 0x285A50;  // following the patch (GUIDE_TINT)
constexpr uint32_t kLedUp = 0xFF5A00;    // move up / clockwise: orange, quick blink
constexpr uint32_t kLedDown = 0x006EFF;  // move down / anticlockwise: blue, slow blink

// The part of the saved state this module owns.
constexpr uint32_t kPanelStateSize = 1 + kPanelCount * 5 + 3 + kPatchNameLen;

class PatchPanel {
 public:
  // Every control live at its parameter's value; no patch loaded.
  void init();
  // Reads the name of every stored patch.
  void scanSlots();

  // ---- the hardware side ----
  // A pot, fader, selector or switch reported a position.
  void moved(int index, int raw);
  int position(int index) const { return phys_[index]; }
  // The control is not live: the sound uses the stored value.
  bool following(int index) const { return (flags_[index] & kFollow) != 0; }
  // The control is not where the patch has it (outside the tolerance).
  bool offPatch(int index) const { return (flags_[index] & kOff) != 0; }
  // Which way the control has to move to reach the patch: +1 up, -1 down, 0.
  int direction(int index) const;
  int target(int index) const { return target_[index]; }
  int followCount() const;
  int offCount() const;
  // The colour of the light behind a control's label, now.
  uint32_t led(int index) const;

  // A panel parameter was set from somewhere other than its control (MIDI).
  void remoteSet(int id);

  // ---- patches ----
  bool slotUsed(int slot) const { return slot >= 0 && slot < kPatchSlots && used_[slot]; }
  const char* slotName(int slot) const { return slotName_[slot]; }
  bool load(int slot);
  void save(int slot);
  // Drops the patch: every control goes live where it physically is.
  void manual();
  int slot() const { return slot_; }  // -1 = none
  bool edited() const { return edited_; }
  void markEdited() { edited_ = true; }
  const char* name() const { return name_; }
  void setNameChar(int pos, char c);
  int guide() const { return guide_; }
  void setGuide(int g);

  void tick(uint32_t dtMs);

  // ---- saved state ----
  uint32_t stateWrite(uint8_t* dst) const;
  static bool stateValid(const uint8_t* src, uint32_t len);
  void stateRead(const uint8_t* src, uint32_t len);
  // After parameters were set wholesale: every control live at its value.
  void syncToParams();

 private:
  static constexpr uint8_t kFollow = 1, kOff = 2;
  void retarget(int index);

  uint16_t phys_[kPanelCount];
  uint16_t target_[kPanelCount];
  uint8_t flags_[kPanelCount];
  char slotName_[kPatchSlots][kPatchNameLen + 1];
  bool used_[kPatchSlots];
  char name_[kPatchNameLen + 1];
  int8_t slot_;
  bool edited_;
  uint8_t guide_;
  uint32_t blinkMs_;
};

extern PatchPanel gPatch;

// The characters a patch name can hold, in the order the encoder steps through them.
extern const char kNameChars[];
int nameCharIndex(char c);

}  // namespace fw

// ---- patch storage, provided by each target ----
// Copies the stored bytes of one slot into dst and returns how many (0 = empty).
extern "C" uint32_t fw_hal_patch_load(uint32_t slot, uint8_t* dst, uint32_t max);
extern "C" void fw_hal_patch_save(uint32_t slot, const uint8_t* src, uint32_t len);
