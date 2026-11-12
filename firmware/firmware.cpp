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
    "btn_fx1", "btn_fx2", "btn_mod", "btn_engine", "btn_patch", "btn_osc1_engine", "btn_osc2_engine", "btn_hold",
};
// after the mode buttons: oscillator 1's engine button, oscillator 2's, then Hold
constexpr int kButtonOsc1Engine = MB_COUNT, kButtonHold = MB_COUNT + 2;

constexpr uint32_t kSaveDelayMs = 1000;  // save once the controls have been still this long
// Version 2: the LFO Shape control became a five-stop selector. Version 1
// stored it as a pot position; that is converted on loading.
// Version 3: arp Tempo and Gate became panel pots (they were stepped encoder values).
constexpr uint8_t kStateVersion = 3;

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
  if (len < 8 || len > kStateSize) return false;
  if (src[0] != 'S' || src[1] != 'Y' || src[2] != 'N' || src[3] != 'T') return false;
  uint8_t version = src[4];
  if (version < 1 || version > kStateVersion) return false;
  int count = src[5];  // parameters are only ever added at the end, so an older blob is a prefix
  if (count < 1 || count > P_COUNT || len != 6u + (uint32_t)count * 2u + 2u) return false;
  uint16_t sum = 0;
  const uint8_t* p = src + 6;
  for (int i = 0; i < count; ++i) {
    uint16_t v = (uint16_t)(p[0] | (p[1] << 8));
    p += 2;
    sum = (uint16_t)(sum * 31u + v + 1u);
  }
  if ((uint16_t)(p[0] | (p[1] << 8)) != sum) return false;
  p = src + 6;
  for (int i = 0; i < count; ++i) {
    int v = p[0] | (p[1] << 8);
    if (version == 1 && i == P_LFO_SHAPE) v = mapLfoShape((float)v * (1.0f / (float)kPotMax));
    if (version < 3 && i == P_ARP_TEMPO) v = v * kPotMax / 200;  // was 0..200 for 40..240 BPM
    if (version < 3 && i == P_ARP_GATE) v = v * kPotMax / 19;    // was 0..19 for 5..100%
    gParams.set(i, v);
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

void process(float* outL, float* outR, int n) { gSynth.process(outL, outR, n); }

void midiByte(uint8_t b) { gMidi.byte(b); }

void onParamChanged(int id) {
  gUi.paramChanged(id);
  gSaveMs = kSaveDelayMs;
}

int controlParam(int index) { return index >= 0 && index < kFirstEncoder ? kPanelParams[index] : -1; }

ControlInfo controlInfo(int index) {
  if (index < 0 || index >= kNumControls) return ControlInfo{"", CK_BUTTON, 0};
  if (index < kFirstEncoder) {
    const ParamDef& d = kParams[kPanelParams[index]];
    if (d.kind == PK_STEP) return ControlInfo{d.key, CK_SELECTOR, d.steps};
    return ControlInfo{d.key, CK_POT, 0};
  }
  if (index < kFirstButton) return ControlInfo{kEncoderNames[index - kFirstEncoder], CK_ENCODER, 0};
  return ControlInfo{kButtonNames[index - kFirstButton], CK_BUTTON, 0};
}

void controlSet(int index, int raw) {
  if (index < 0 || index >= kNumControls) return;
  if (index < kFirstEncoder) {  // pot, selector or switch: the control is the parameter
    int id = kPanelParams[index];
    if (gParams.set(id, raw)) onParamChanged(id);
    return;
  }
  if (raw == 0) return;  // buttons and encoder pushes act when pressed
  if (index < kFirstButton) {
    gUi.encoderPush(index - kFirstEncoder);
    return;
  }
  int b = index - kFirstButton;
  if (b < MB_COUNT) {
    gUi.modeButton(b);
  } else if (b == kButtonHold) {  // Hold latches; press again to let go
    gParams.set(P_HOLD, gParams.step(P_HOLD) ? 0 : 1);
    onParamChanged(P_HOLD);
  } else {  // an oscillator's engine button steps through the engines
    int id = b == kButtonOsc1Engine ? P_OSC1_ENGINE : P_OSC2_ENGINE;
    gParams.set(id, (gParams.step(id) + 1) % ENG_COUNT);
    onParamChanged(id);
  }
}

void encoderTurn(int index, int delta) {
  int e = index - kFirstEncoder;
  if (e < 0 || e >= kNumEncoders || delta == 0) return;
  gEncoderPos[e] = (uint16_t)(gEncoderPos[e] + delta);
  gUi.encoderTurn(e, delta);
}

int controlGet(int index) {
  if (index < 0 || index >= kNumControls) return 0;
  if (index < kFirstEncoder) return gParams.val[kPanelParams[index]];
  if (index < kFirstButton) return gEncoderPos[index - kFirstEncoder];
  int b = index - kFirstButton;
  if (b < MB_COUNT) return gUi.mode() == b ? 1 : 0;
  if (b == kButtonHold) return gParams.step(P_HOLD);
  // an engine button is lit while its oscillator is on anything but the plain VA engine
  return gParams.step(b == kButtonOsc1Engine ? P_OSC1_ENGINE : P_OSC2_ENGINE) != ENG_VA ? 1 : 0;
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

const uint8_t* frame() { return gFrame; }

void frameRow565(int y, uint16_t* dst) { gfxRow565(y, dst); }

}  // namespace fw
