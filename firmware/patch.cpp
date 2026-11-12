#include "patch.h"

#include "firmware.h"
#include "fw_math.h"
#include "ui.h"

namespace fw {

PatchPanel gPatch;

const char kNameChars[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-";

int nameCharIndex(char c) {
  for (int i = 0; kNameChars[i]; ++i)
    if (kNameChars[i] == c) return i;
  return 0;
}

namespace {

constexpr uint8_t kPatchVersion = 1;
constexpr int kTolerance = 102;      // 2.5% of a pot's travel counts as "at the patch"
constexpr int kToleranceFine = 41;   // 1% for pitch
constexpr uint32_t kBlinkUpMs = 150, kBlinkDownMs = 450;  // half periods

int absi(int x) { return x < 0 ? -x : x; }

int toleranceOf(int id) { return id == P_OSC2_DETUNE ? kToleranceFine : kTolerance; }

// Master volume is not part of a patch, and Hold is a performance switch.
bool inPatch(int id) { return id != P_VOLUME && id != P_HOLD; }

void setName(char* dst, const char* src) {
  int i = 0;
  for (; i < kPatchNameLen && src[i]; ++i) dst[i] = kNameChars[nameCharIndex(src[i])];
  for (; i < kPatchNameLen; ++i) dst[i] = ' ';
  dst[kPatchNameLen] = 0;
}

uint16_t rolling(uint16_t sum, uint16_t v) { return (uint16_t)(sum * 31u + v + 1u); }

// Checks a stored patch and, if `vals` is given, unpacks it. Parameters are
// only ever added at the end, so an older patch is a prefix: the ones it
// lacks come back as their defaults.
bool patchRead(const uint8_t* src, uint32_t len, uint16_t* vals, char* name) {
  if (len < 5u + kPatchNameLen + 2u + 2u || len > kPatchSize) return false;
  if (src[0] != 'P' || src[1] != 'A' || src[2] != 'T' || src[3] != kPatchVersion) return false;
  int count = src[4];
  if (count < 1 || count > P_COUNT || len != 5u + kPatchNameLen + (uint32_t)count * 2u + 2u) return false;
  uint16_t sum = 0;
  const uint8_t* p = src + 5;
  for (int i = 0; i < kPatchNameLen; ++i) sum = rolling(sum, *p++);
  for (int i = 0; i < count; ++i) {
    sum = rolling(sum, (uint16_t)(p[0] | (p[1] << 8)));
    p += 2;
  }
  if ((uint16_t)(p[0] | (p[1] << 8)) != sum) return false;
  if (name) {
    char raw[kPatchNameLen + 1];
    for (int i = 0; i < kPatchNameLen; ++i) raw[i] = (char)src[5 + i];
    raw[kPatchNameLen] = 0;
    setName(name, raw);
  }
  if (vals) {
    p = src + 5 + kPatchNameLen;
    for (int i = 0; i < P_COUNT; ++i) {
      if (i < count) {
        vals[i] = (uint16_t)(p[0] | (p[1] << 8));
        p += 2;
      } else {
        vals[i] = kParams[i].def;
      }
    }
  }
  return true;
}

uint32_t scaleRgb(uint32_t rgb, int level255) {
  uint32_t r = ((rgb >> 16) & 255u) * (uint32_t)level255 / 255u;
  uint32_t g = ((rgb >> 8) & 255u) * (uint32_t)level255 / 255u;
  uint32_t b = (rgb & 255u) * (uint32_t)level255 / 255u;
  return (r << 16) | (g << 8) | b;
}

}  // namespace

void PatchPanel::init() {
  syncToParams();
  setName(name_, "INIT");
  slot_ = -1;
  edited_ = false;
  guide_ = GUIDE_ON;
  blinkMs_ = 0;
  for (int s = 0; s < kPatchSlots; ++s) {
    used_[s] = false;
    setName(slotName_[s], "");
  }
}

void PatchPanel::scanSlots() {
  uint8_t blob[kPatchSize];
  for (int s = 0; s < kPatchSlots; ++s) {
    uint32_t n = fw_hal_patch_load((uint32_t)s, blob, kPatchSize);
    used_[s] = n > 0 && patchRead(blob, n, nullptr, slotName_[s]);
    if (!used_[s]) setName(slotName_[s], "");
  }
}

void PatchPanel::syncToParams() {
  for (int i = 0; i < kPanelCount; ++i) {
    phys_[i] = target_[i] = gParams.val[kPanelParams[i]];
    flags_[i] = 0;
  }
}

// The parameter behind control `index` was just set from a patch or from
// MIDI: decide whether the control is still where the sound is.
void PatchPanel::retarget(int index) {
  int id = kPanelParams[index];
  if (!inPatch(id)) return;
  int v = gParams.val[id];
  target_[index] = (uint16_t)v;
  int gap = absi(v - (int)phys_[index]);
  if (gap == 0) flags_[index] = 0;
  else if (kParams[id].kind == PK_STEP) flags_[index] = kFollow | kOff;
  else flags_[index] = (uint8_t)(kFollow | (gap > toleranceOf(id) ? kOff : 0));
}

void PatchPanel::remoteSet(int id) {
  for (int i = 0; i < kPanelCount; ++i)
    if (kPanelParams[i] == id) {
      if (!inPatch(id)) phys_[i] = target_[i] = gParams.val[id];  // volume has no pickup: the knob is simply overruled
      else retarget(i);
      stateTouched();
      return;
    }
}

void PatchPanel::moved(int index, int raw) {
  if (index < 0 || index >= kPanelCount) return;
  int id = kPanelParams[index];
  const ParamDef& d = kParams[id];
  bool stepped = d.kind == PK_STEP;
  raw = clampi(raw, 0, stepped ? d.steps - 1 : kPotMax);
  int prev = phys_[index];
  if (raw == prev) return;

  if (!following(index)) {  // live: the control is the parameter
    phys_[index] = (uint16_t)raw;
    if (stepped && offPatch(index) && raw == target_[index]) flags_[index] = 0;  // back on the patch's stop
    if (gParams.set(id, raw)) onParamChanged(id);
    else stateTouched();
    return;
  }

  if (stepped) {  // a switch goes live on any change
    phys_[index] = (uint16_t)raw;
    flags_[index] = raw == target_[index] ? 0 : kOff;
    if (gParams.set(id, raw)) onParamChanged(id);
    else gUi.paramChanged(id);
    stateTouched();
    return;
  }

  // A following pot: it takes over when it passes through the stored value
  // or comes to rest close enough to it. (Smoothing a real pot's jitter is
  // the panel scanner's job, for live and following controls alike.)
  phys_[index] = (uint16_t)raw;
  int want = gParams.val[id];
  bool crossed = raw == want || (prev < want) != (raw < want);
  if (crossed || absi(raw - want) <= toleranceOf(id)) {
    flags_[index] = 0;
    target_[index] = (uint16_t)raw;
    if (gParams.set(id, raw)) onParamChanged(id);
    else gUi.paramChanged(id);
  } else {
    flags_[index] = kFollow | kOff;
    gUi.pickup(id);
  }
  stateTouched();
}

int PatchPanel::direction(int index) const {
  int t = following(index) ? (int)gParams.val[kPanelParams[index]] : (int)target_[index];
  int p = phys_[index];
  return t > p ? 1 : (t < p ? -1 : 0);
}

int PatchPanel::followCount() const {
  int n = 0;
  for (int i = 0; i < kPanelCount; ++i) n += following(i);
  return n;
}

int PatchPanel::offCount() const {
  int n = 0;
  for (int i = 0; i < kPanelCount; ++i) n += offPatch(i);
  return n;
}

uint32_t PatchPanel::led(int index) const {
  if (index < 0 || index >= kPanelCount) return 0;
  if (guide_ == GUIDE_OFF) return kLedIdle;
  if (guide_ == GUIDE_TINT) return following(index) ? kLedTint : kLedIdle;
  int dir = direction(index);
  if (!offPatch(index) || dir == 0) return kLedIdle;

  // Brighter the further there is to go; a pot's light dims as it closes in.
  int level = 255;
  int id = kPanelParams[index];
  if (kParams[id].kind != PK_STEP) {
    int gap = absi((int)gParams.val[id] - (int)phys_[index]);
    level = 115 + (gap > 1228 ? 140 : gap * 140 / 1228);
  }
  // Direction is in the colour and in the blink rate, so colour is never the only cue.
  uint32_t half = dir > 0 ? kBlinkUpMs : kBlinkDownMs;
  bool lit = ((blinkMs_ / half) & 1u) == 0;
  return scaleRgb(dir > 0 ? kLedUp : kLedDown, lit ? level : level / 4);
}

bool PatchPanel::load(int slot) {
  if (slot < 0 || slot >= kPatchSlots) return false;
  uint8_t blob[kPatchSize];
  uint16_t vals[P_COUNT];
  char nm[kPatchNameLen + 1];
  uint32_t n = fw_hal_patch_load((uint32_t)slot, blob, kPatchSize);
  if (n == 0 || !patchRead(blob, n, vals, nm)) return false;
  for (int i = 0; i < P_COUNT; ++i)
    if (inPatch(i)) gParams.set(i, vals[i]);
  for (int i = 0; i < kPanelCount; ++i) retarget(i);
  setName(name_, nm);
  slot_ = (int8_t)slot;
  edited_ = false;
  stateTouched();
  return true;
}

void PatchPanel::save(int slot) {
  if (slot < 0 || slot >= kPatchSlots) return;
  uint8_t blob[kPatchSize];
  blob[0] = 'P';
  blob[1] = 'A';
  blob[2] = 'T';
  blob[3] = kPatchVersion;
  blob[4] = (uint8_t)P_COUNT;
  uint16_t sum = 0;
  uint8_t* p = blob + 5;
  for (int i = 0; i < kPatchNameLen; ++i) {
    *p++ = (uint8_t)name_[i];
    sum = rolling(sum, (uint8_t)name_[i]);
  }
  for (int i = 0; i < P_COUNT; ++i) {
    uint16_t v = gParams.val[i];
    *p++ = (uint8_t)(v & 0xFF);
    *p++ = (uint8_t)(v >> 8);
    sum = rolling(sum, v);
  }
  *p++ = (uint8_t)(sum & 0xFF);
  *p++ = (uint8_t)(sum >> 8);
  fw_hal_patch_save((uint32_t)slot, blob, kPatchSize);

  used_[slot] = true;
  setName(slotName_[slot], name_);
  slot_ = (int8_t)slot;
  edited_ = false;
  // What is live now is what the patch holds: nothing live is off it any more.
  for (int i = 0; i < kPanelCount; ++i)
    if (!following(i)) {
      flags_[i] = 0;
      target_[i] = phys_[i];
    }
  stateTouched();
}

void PatchPanel::manual() {
  for (int i = 0; i < kPanelCount; ++i) {
    int id = kPanelParams[i];
    flags_[i] = 0;
    target_[i] = phys_[i];
    if (inPatch(id) && gParams.set(id, phys_[i])) edited_ = true;
  }
  stateTouched();
}

void PatchPanel::setNameChar(int pos, char c) {
  if (pos < 0 || pos >= kPatchNameLen) return;
  c = kNameChars[nameCharIndex(c)];
  if (name_[pos] == c) return;
  name_[pos] = c;
  edited_ = true;
  stateTouched();
}

void PatchPanel::setGuide(int g) {
  g = clampi(g, 0, GUIDE_COUNT - 1);
  if (g == guide_) return;
  guide_ = (uint8_t)g;
  stateTouched();
}

void PatchPanel::tick(uint32_t dtMs) { blinkMs_ = (blinkMs_ + dtMs) % 3600000u; }

uint32_t PatchPanel::stateWrite(uint8_t* dst) const {
  uint8_t* p = dst;
  *p++ = (uint8_t)kPanelCount;
  for (int i = 0; i < kPanelCount; ++i) {
    *p++ = (uint8_t)(phys_[i] & 0xFF);
    *p++ = (uint8_t)(phys_[i] >> 8);
    *p++ = (uint8_t)(target_[i] & 0xFF);
    *p++ = (uint8_t)(target_[i] >> 8);
    *p++ = flags_[i];
  }
  *p++ = (uint8_t)(slot_ + 1);
  *p++ = guide_;
  *p++ = edited_ ? 1 : 0;
  for (int i = 0; i < kPatchNameLen; ++i) *p++ = (uint8_t)name_[i];
  return (uint32_t)(p - dst);
}

bool PatchPanel::stateValid(const uint8_t* src, uint32_t len) {
  if (len < 1) return false;
  uint32_t n = src[0];
  return n >= 1 && n <= (uint32_t)kPanelCount && len == 1u + n * 5u + 3u + kPatchNameLen;
}

void PatchPanel::stateRead(const uint8_t* src, uint32_t len) {
  if (!stateValid(src, len)) return;
  int n = src[0];
  const uint8_t* p = src + 1;
  for (int i = 0; i < n; ++i, p += 5) {
    int id = kPanelParams[i];
    const ParamDef& d = kParams[id];
    int hi = d.kind == PK_STEP ? d.steps - 1 : kPotMax;
    phys_[i] = (uint16_t)clampi(p[0] | (p[1] << 8), 0, hi);
    target_[i] = (uint16_t)clampi(p[2] | (p[3] << 8), 0, hi);
    flags_[i] = inPatch(id) ? (uint8_t)(p[4] & (kFollow | kOff)) : 0;
    // A control saved as live must agree with its parameter.
    if (!following(i) && phys_[i] != gParams.val[id]) phys_[i] = gParams.val[id];
  }
  int s = (int)p[0] - 1;
  slot_ = (int8_t)(s >= 0 && s < kPatchSlots ? s : -1);
  guide_ = p[1] < GUIDE_COUNT ? p[1] : (uint8_t)GUIDE_ON;
  edited_ = p[2] != 0;
  char raw[kPatchNameLen + 1];
  for (int i = 0; i < kPatchNameLen; ++i) raw[i] = (char)p[3 + i];
  raw[kPatchNameLen] = 0;
  setName(name_, raw);
}

}  // namespace fw
