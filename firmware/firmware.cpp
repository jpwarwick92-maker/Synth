#include "firmware.h"

#include "gfx.h"
#include "midi.h"
#include "synth.h"
#include "tables.h"
#include "ui.h"

namespace fw {

namespace {

const char* const kEncoderNames[kNumEncoders] = {"enc1", "enc2", "enc3", "enc4"};
const char* const kButtonNames[kNumButtons] = {
    "btn_fx1", "btn_fx2", "btn_mod", "btn_arp", "btn_engine", "btn_patch", "btn_osc1_engine", "btn_osc2_engine",
};

constexpr uint32_t kSaveDelayMs = 1000;  // save once the controls have been still this long
constexpr uint8_t kStateVersion = 1;

uint16_t gEncoderPos[kNumEncoders];
uint32_t gSaveMs;

}  // namespace

uint32_t stateWrite(uint8_t* dst) {
  dst[0] = 'S';
  dst[1] = 'Y';
  dst[2] = 'N';
  dst[3] = 'T';
  dst[4] = kStateVersion;
  dst[5] = (uint8_t)P_COUNT;
  uint16_t sum = 0;
  uint8_t* p = dst + 6;
  for (int i = 0; i < P_COUNT; ++i) {
    uint16_t v = gParams.val[i];
    *p++ = (uint8_t)(v & 0xFF);
    *p++ = (uint8_t)(v >> 8);
    sum = (uint16_t)(sum * 31u + v + 1u);
  }
  *p++ = (uint8_t)(sum & 0xFF);
  *p++ = (uint8_t)(sum >> 8);
  return kStateSize;
}

bool stateRead(const uint8_t* src, uint32_t len) {
  if (len != kStateSize) return false;
  if (src[0] != 'S' || src[1] != 'Y' || src[2] != 'N' || src[3] != 'T') return false;
  if (src[4] != kStateVersion || src[5] != (uint8_t)P_COUNT) return false;
  uint16_t sum = 0;
  const uint8_t* p = src + 6;
  for (int i = 0; i < P_COUNT; ++i) {
    uint16_t v = (uint16_t)(p[0] | (p[1] << 8));
    p += 2;
    sum = (uint16_t)(sum * 31u + v + 1u);
  }
  if ((uint16_t)(p[0] | (p[1] << 8)) != sum) return false;
  p = src + 6;
  for (int i = 0; i < P_COUNT; ++i) {
    gParams.set(i, p[0] | (p[1] << 8));
    p += 2;
  }
  return true;
}

void init(float sampleRate) {
  tablesInit();
  gParams.reset();
  uint8_t blob[kStateSize];
  uint32_t n = fw_hal_storage_load(blob, kStateSize);
  if (n > 0) stateRead(blob, n);
  gSynth.init(sampleRate);
  gMidi.init();
  gUi.init();
  for (int i = 0; i < kNumEncoders; ++i) gEncoderPos[i] = 0;
  gSaveMs = 0;
}

void process(float* out, int n) { gSynth.process(out, n); }

void midiByte(uint8_t b) { gMidi.byte(b); }

void onParamChanged(int id) {
  gUi.paramChanged(id);
  gSaveMs = kSaveDelayMs;
}

ControlInfo controlInfo(int index) {
  if (index < 0 || index >= kNumControls) return ControlInfo{"", CK_BUTTON, 0};
  if (index < kFirstEncoder) {
    const ParamDef& d = kParams[index];
    if (d.kind == PK_STEP) return ControlInfo{d.key, CK_SELECTOR, d.steps};
    return ControlInfo{d.key, CK_POT, 0};
  }
  if (index < kFirstButton) return ControlInfo{kEncoderNames[index - kFirstEncoder], CK_ENCODER, 0};
  return ControlInfo{kButtonNames[index - kFirstButton], CK_BUTTON, 0};
}

void controlSet(int index, int raw) {
  if (index < 0 || index >= kNumControls) return;
  if (index < kFirstEncoder) {  // pot or selector: the control is the parameter
    if (gParams.set(index, raw)) onParamChanged(index);
    return;
  }
  if (raw == 0) return;  // buttons and encoder pushes act when pressed
  if (index < kFirstButton) {
    gUi.encoderPush(index - kFirstEncoder);
    return;
  }
  int b = index - kFirstButton;
  if (b < MB_COUNT) gUi.modeButton(b);
  else gUi.oscEngineButton(b - MB_COUNT);
}

void encoderTurn(int index, int delta) {
  int e = index - kFirstEncoder;
  if (e < 0 || e >= kNumEncoders || delta == 0) return;
  gEncoderPos[e] = (uint16_t)(gEncoderPos[e] + delta);
  gUi.encoderTurn(e, delta);
}

int controlGet(int index) {
  if (index < 0 || index >= kNumControls) return 0;
  if (index < kFirstEncoder) return gParams.val[index];
  if (index < kFirstButton) return gEncoderPos[index - kFirstEncoder];
  int b = index - kFirstButton;
  return b < MB_COUNT && gUi.mode() == b ? 1 : 0;
}

bool tick(uint32_t dtMs) {
  gMidi.tick(dtMs);
  if (gSaveMs > 0) {
    if (gSaveMs > dtMs) {
      gSaveMs -= dtMs;
    } else {
      gSaveMs = 0;
      uint8_t blob[kStateSize];
      fw_hal_storage_save(blob, stateWrite(blob));
    }
  }
  return gUi.tick(dtMs);
}

const uint16_t* frame() { return gFrame; }

}  // namespace fw
