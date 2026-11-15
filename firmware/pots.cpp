// Pots and faders as hardware really reports them.
//
// controlSet() expects a clean position. A real pot read through an
// analog-to-digital converter is not clean: the reading jitters by several
// counts even when nobody touches the knob (the Teensy 4's converter is
// good for about 10 of its 12 bits), and a multiplexer adds a little of
// its neighbours. Passed on raw, every twitch would count as a movement:
// it would bring up the control's picture, mark the patch as changed,
// restart the save timer, and stop a loaded patch's controls from ever
// resting. potReading() stands between the two. It smooths the readings,
// and only reports a new position when the smoothed value has plainly left
// the last one reported.
//
// What it cannot do is undo a reading that is steadily wrong. Bleed from a
// neighbouring pot through a multiplexer is of that kind: the scanning code
// and the circuit have to keep it below the dead band here (see kPotDeadBand).
#include "firmware.h"

#include "fw_math.h"

namespace fw {

namespace {

// A reported position stands until the smoothed reading is more than this
// far from it: about a quarter of a percent of the travel, 2.5 times the
// jitter expected. Steady errors from the circuit must stay inside it.
constexpr int kPotDeadBand = 10;
// Within this of either end counts as the end, so the full range is always reachable.
constexpr int kPotEndZone = 14;

struct Pot {
  int32_t smooth;  // smoothed reading, times 256
  int16_t out;     // last position reported
  bool primed;
};
Pot gPots[kPanelCount];

int absInt(int x) { return x < 0 ? -x : x; }

}  // namespace

void potsReset() {
  for (int i = 0; i < kPanelCount; ++i) gPots[i] = Pot{0, 0, false};
}

void potReading(int index, int raw) {
  if (index < 0 || index >= kPanelCount) return;
  if (controlInfo(index).kind != CK_POT) {  // a selector or switch has no jitter to smooth
    controlSet(index, raw);
    return;
  }
  raw = clampi(raw, 0, kPotMax);
  Pot& p = gPots[index];
  bool first = !p.primed;
  if (first) {
    // If the first reading is where the firmware last knew the knob to be,
    // give or take the jitter, the knob has not been touched while the power
    // was off: carry on from the known position. Otherwise it is simply
    // where the knob now is.
    p.primed = true;
    p.out = (int16_t)controlGet(index);
    p.smooth = (absInt(raw - p.out) <= 3 * kPotDeadBand ? p.out : raw) * 256;
  } else {
    // Quick to follow a knob that is plainly being turned, slow to follow
    // what is only jitter: the further the reading is from the smoothed
    // value, the larger the share of the difference taken.
    int diff = raw - p.smooth / 256;
    int far = absInt(diff);
    int share = far > 300 ? 192 : far > 96 ? 128 : far > 24 ? 64 : 16;  // of 256
    p.smooth += diff * share;
  }
  int v = (p.smooth + 128) / 256;

  // the ends, with their own dead band so a knob resting near one does not chatter
  int to = v;
  if (v <= kPotEndZone || (p.out == 0 && v <= kPotEndZone + kPotDeadBand)) to = 0;
  if (v >= kPotMax - kPotEndZone || (p.out == kPotMax && v >= kPotMax - kPotEndZone - kPotDeadBand)) to = kPotMax;

  bool moved = absInt(to - p.out) > kPotDeadBand || ((to == 0 || to == kPotMax) && to != p.out);
  if (moved) {
    p.out = (int16_t)to;
    controlSet(index, to);
  }
}

}  // namespace fw
