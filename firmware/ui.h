// The display and the soft controls: what is on the screen, what the mode
// buttons and the four encoders do. Draws into the display buffer (gfx.h).
#pragma once

#include "fw_config.h"

namespace fw {

enum ModeButton : int8_t { MB_NONE = -1, MB_FX1, MB_FX2, MB_MOD, MB_ENGINE, MB_PATCH, MB_COUNT };

constexpr uint32_t kPopupMs = 1500;  // how long a control's visual stays up

class Ui {
 public:
  void init();

  // A panel control or a MIDI controller moved this parameter: show its visual.
  void paramChanged(int id);

  // One of the five mode buttons was pressed. A mode can have several pages:
  // pressing its lit button steps through them, then returns home.
  void modeButton(int which);
  void encoderTurn(int enc, int delta);
  void encoderPush(int enc);

  // Advances time and redraws. Returns true if the display buffer changed.
  bool tick(uint32_t dtMs);

  int mode() const { return mode_; }
  int page() const { return page_; }
  int popupParam() const { return popupParam_; }

 private:
  void drawHome();
  void drawModePage();
  void drawParamPopup(int id);
  int pageParam(int enc) const;

  int8_t mode_;
  int8_t page_;
  int16_t popupParam_;   // -1 = none
  uint32_t popupMs_;
  uint8_t burst_[4];     // encoder clicks since the last tick, for speeding up fast turns
  bool dirty_;
};

extern Ui gUi;

}  // namespace fw
