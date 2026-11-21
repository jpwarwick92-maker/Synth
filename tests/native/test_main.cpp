// Native tests for the firmware core.
//
// These build the same source files that go into the browser and Teensy
// builds, drive them only through firmware.h, and check the results by
// rendering audio and display frames with no browser involved.
//
//   build/tests <outdir>     run every test; writes display frames to <outdir>
//   build/tests --controls   print docs/controls.md

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <string>
#include <chrono>
#include <limits>
#include <vector>

#include "firmware.h"
#include "fw_math.h"
#include "gfx.h"
#include "params.h"
#include "synth.h"
#include "tables.h"
#include "ui.h"
#include "vocoder.h"

// ---------------------------------------------------------------- harness

static int gFails = 0;
static int gChecks = 0;

#define CHECK(cond, ...)                                  \
  do {                                                    \
    ++gChecks;                                            \
    if (!(cond)) {                                        \
      ++gFails;                                           \
      std::printf("    FAIL %s:%d  %s  -- ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

static void section(const char* name) { std::printf("%s\n", name); }

// ---- allocation counting (R4) ----
static bool gCounting = false;
static long gAllocs = 0;

extern "C" {
void* __real_malloc(size_t);
void* __real_calloc(size_t, size_t);
void* __real_realloc(void*, size_t);
void* __wrap_malloc(size_t n) {
  if (gCounting) ++gAllocs;
  return __real_malloc(n);
}
void* __wrap_calloc(size_t a, size_t b) {
  if (gCounting) ++gAllocs;
  return __real_calloc(a, b);
}
void* __wrap_realloc(void* p, size_t n) {
  if (gCounting) ++gAllocs;
  return __real_realloc(p, n);
}
}
void* operator new(size_t n) {
  if (gCounting) ++gAllocs;
  void* p = __real_malloc(n ? n : 1);
  if (!p) std::abort();
  return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }

// ---- storage stand-in ----
static uint8_t gStore[1024];
static uint32_t gStoreLen = 0;
static int gSaveCount = 0;
// patch slots, as a target's flash or card would hold them
static uint8_t gBank[fw::kPatchSlots][fw::kPatchSize];
static uint32_t gBankLen[fw::kPatchSlots];
static int gPatchSaves = 0;

extern "C" uint32_t fw_hal_patch_load(uint32_t slot, uint8_t* dst, uint32_t max) {
  if (slot >= (uint32_t)fw::kPatchSlots) return 0;
  uint32_t n = gBankLen[slot] < max ? gBankLen[slot] : max;
  std::memcpy(dst, gBank[slot], n);
  return n;
}
extern "C" void fw_hal_patch_save(uint32_t slot, const uint8_t* src, uint32_t len) {
  if (slot >= (uint32_t)fw::kPatchSlots || len > fw::kPatchSize) return;
  std::memcpy(gBank[slot], src, len);
  gBankLen[slot] = len;
  ++gPatchSaves;
}

extern "C" uint32_t fw_hal_storage_load(uint8_t* dst, uint32_t max) {
  uint32_t n = gStoreLen < max ? gStoreLen : max;
  std::memcpy(dst, gStore, n);
  return n;
}
extern "C" void fw_hal_storage_save(const uint8_t* src, uint32_t len) {
  std::memcpy(gStore, src, len);
  gStoreLen = len;
  ++gSaveCount;
}

// ---- driving the firmware ----
static double gSr = 48000.0;

static void boot(double sr, bool keepStorage = false) {
  if (!keepStorage) {
    gStoreLen = 0;
    std::memset(gBankLen, 0, sizeof gBankLen);
  }
  gSr = sr;
  fw::init((float)sr);
}

static int ctl(const char* name) {
  for (int i = 0; i < fw::kNumControls; ++i)
    if (!std::strcmp(fw::controlInfo(i).name, name)) return i;
  std::printf("    no control named %s\n", name);
  std::exit(2);
}

static void setPot(const char* name, double n) { fw::controlSet(ctl(name), (int)std::lround(n * 4095.0)); }
static void setSel(const char* name, int idx) { fw::controlSet(ctl(name), idx); }
static void press(const char* name) {
  fw::controlSet(ctl(name), 1);
  fw::controlSet(ctl(name), 0);
}
static void midi(std::initializer_list<int> bytes) {
  for (int b : bytes) fw::midiByte((uint8_t)b);
}

// Renders into `out` (left channel). The right channel of the same span is
// kept in gRight for the tests that look at stereo.
static std::vector<float> gRight;

static void render(std::vector<float>& out, double sec) {
  long n = std::lround(sec * gSr);
  float l[fw::kMaxBlock], r[fw::kMaxBlock];
  while (n > 0) {
    int len = n > fw::kMaxBlock ? fw::kMaxBlock : (int)n;
    fw::process(l, r, len);
    out.insert(out.end(), l, l + len);
    gRight.insert(gRight.end(), r, r + len);
    n -= len;
  }
}

static std::vector<float> renderNew(double sec) {
  std::vector<float> v;
  gRight.clear();
  render(v, sec);
  return v;
}

// Sets any parameter through its MIDI controller number, as a knob box would.
static void setCC(int param, int value7) { midi({0xB0, fw::kParams[param].cc, value7}); }

static double peakOf(const std::vector<float>& x, size_t from = 0, size_t to = (size_t)-1) {
  double p = 0;
  if (to > x.size()) to = x.size();
  for (size_t i = from; i < to; ++i) p = std::max(p, (double)std::fabs(x[i]));
  return p;
}

static double rmsDiff(const std::vector<float>& a, const std::vector<float>& b) {
  size_t n = std::min(a.size(), b.size());
  double s = 0;
  for (size_t i = 0; i < n; ++i) s += (double)(a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(s / (double)n);
}

// A plain, clean tone: oscillator 1 only, filter wide open, flat envelope.
static void cleanTone(int wave) {
  setSel("osc1_wave", wave);
  setSel("osc1_octave", 2);
  setPot("mix_osc1", 1.0);
  setPot("mix_osc2", 0.0);
  setPot("mix_sub", 0.0);
  setPot("mix_noise", 0.0);
  setPot("filter_cutoff", 1.0);
  setPot("filter_resonance", 0.0);
  setPot("filter_env_amount", 0.5);
  setPot("filter_keytrack", 0.0);
  setPot("filter_drive", 0.0);
  setSel("filter_type", 0);
  midi({0xB0, 113, 0});  // 12 dB slope
  setPot("voice_glide", 0.0);
  setSel("voice_mode", 0);
  setPot("aenv_attack", 0.0);
  setPot("aenv_sustain", 1.0);
  setPot("aenv_release", 0.2);
  setPot("aenv_velocity", 0.0);
  setPot("lfo_depth", 0.0);
  setPot("volume", 0.75);
  renderNew(0.05);  // let the control smoothing settle
}

// Frequency from rising zero crossings (use on sine waves).
static double measureFreq(const std::vector<float>& x, size_t from, size_t to) {
  double first = -1, last = -1;
  long count = 0;
  for (size_t i = from + 1; i < to; ++i) {
    if (x[i - 1] < 0 && x[i] >= 0) {
      double t = (double)(i - 1) + (double)(-x[i - 1]) / (double)(x[i] - x[i - 1]);
      if (first < 0) first = t;
      last = t;
      ++count;
    }
  }
  if (count < 2) return 0;
  return (double)(count - 1) / (last - first) * gSr;
}

// ---- FFT ----
static void fft(std::vector<std::complex<double>>& a) {
  size_t n = a.size();
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(a[i], a[j]);
  }
  for (size_t len = 2; len <= n; len <<= 1) {
    double ang = -2 * M_PI / (double)len;
    std::complex<double> wl(std::cos(ang), std::sin(ang));
    for (size_t i = 0; i < n; i += len) {
      std::complex<double> w(1);
      for (size_t k = 0; k < len / 2; ++k) {
        auto u = a[i + k], v = a[i + k + len / 2] * w;
        a[i + k] = u + v;
        a[i + k + len / 2] = u - v;
        w *= wl;
      }
    }
  }
}

static std::vector<double> spectrum(const std::vector<float>& x, size_t from, size_t n) {
  std::vector<std::complex<double>> a(n);
  for (size_t i = 0; i < n; ++i) {
    double t = 2 * M_PI * (double)i / (double)(n - 1);
    double w = 0.35875 - 0.48829 * std::cos(t) + 0.14128 * std::cos(2 * t) - 0.01168 * std::cos(3 * t);
    a[i] = (double)x[from + i] * w;  // Blackman-Harris: side lobes below -92 dB
  }
  fft(a);
  std::vector<double> m(n / 2);
  for (size_t i = 0; i < n / 2; ++i) m[i] = std::abs(a[i]);
  return m;
}

struct AliasResult {
  double f0;
  double worstDb;      // loudest content between the harmonics, relative to the fundamental
  double worstHz;
  double h2Db, h3Db;   // second and third harmonic, relative to the fundamental
  int harmonics;
};

// With knownHz = 0 the fundamental is found from the spectrum (the lowest
// strong peak). Engines whose fundamental can be weak or missing (FM) pass
// the pitch they were asked for; levels are then relative to the loudest partial.
static AliasResult aliasCheck(const std::vector<float>& x, double knownHz = 0) {
  const size_t n = 32768;
  size_t from = (size_t)(0.4 * gSr);
  std::vector<double> m = spectrum(x, from, n);
  double binHz = gSr / (double)n;

  size_t pk = 4;
  for (size_t i = 4; i < m.size(); ++i)
    if (m[i] > m[pk]) pk = i;
  // the fundamental is the lowest strong peak, not necessarily the largest bin
  size_t fund = pk;
  for (size_t i = 4; i < pk; ++i)
    if (m[i] > 0.25 * m[pk] && m[i] >= m[i - 1] && m[i] >= m[i + 1]) {
      fund = i;
      break;
    }
  double a = m[fund - 1], b = m[fund], c = m[fund + 1];
  double f0bin = (double)fund + 0.5 * (a - c) / (a - 2 * b + c);
  if (knownHz > 0) {
    f0bin = knownHz / binHz;
    fund = pk;
  }

  AliasResult r{};
  r.f0 = f0bin * binHz;
  r.worstDb = -300;
  r.harmonics = (int)((double)m.size() / f0bin);
  const double guard = 8;  // bins either side of a harmonic that belong to it
  for (size_t i = 8; i < m.size(); ++i) {
    double k = std::round((double)i / f0bin);
    if (k >= 1 && std::fabs((double)i - k * f0bin) <= guard) continue;
    double db = 20 * std::log10(m[i] / m[fund] + 1e-300);
    if (db > r.worstDb) {
      r.worstDb = db;
      r.worstHz = (double)i * binHz;
    }
  }
  auto harm = [&](int k) {
    size_t i = (size_t)std::lround(k * f0bin);
    double v = 0;
    for (size_t j = i - 2; j <= i + 2 && j < m.size(); ++j) v = std::max(v, m[j]);
    return 20 * std::log10(v / m[fund] + 1e-300);
  };
  r.h2Db = harm(2);
  r.h3Db = harm(3);
  return r;
}

// ---- display frames ----
static std::string gOutDir;

static void saveFrame(const char* name) {
  if (gOutDir.empty()) return;
  std::string path = gOutDir + "/" + name + ".ppm";
  FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return;
  std::fprintf(f, "P6\n%d %d\n255\n", fw::kDisplayW, fw::kDisplayH);
  uint16_t row[fw::kDisplayW];
  for (int y = 0; y < fw::kDisplayH; ++y) {
    fw::frameRow565(y, row);  // the same expansion a display driver uses
    for (int x = 0; x < fw::kDisplayW; ++x) {
      uint16_t p = row[x];
      unsigned char rgb[3] = {(unsigned char)((p >> 11) << 3), (unsigned char)(((p >> 5) & 63) << 2),
                              (unsigned char)((p & 31) << 3)};
      std::fwrite(rgb, 1, 3, f);
    }
  }
  std::fclose(f);
}

// Writes a render as a sound file (one channel, 16 bits) beside the display
// frames, so a result can be listened to as well as measured.
static void saveSound(const char* name, const std::vector<float>& x) {
  if (gOutDir.empty()) return;
  FILE* f = std::fopen((gOutDir + "/" + name + ".wav").c_str(), "wb");
  if (!f) return;
  uint32_t rate = (uint32_t)gSr, bytes = (uint32_t)x.size() * 2;
  auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
  auto u16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
  std::fwrite("RIFF", 1, 4, f);
  u32(36 + bytes);
  std::fwrite("WAVEfmt ", 1, 8, f);
  u32(16);
  u16(1);
  u16(1);
  u32(rate);
  u32(rate * 2);
  u16(2);
  u16(16);
  std::fwrite("data", 1, 4, f);
  u32(bytes);
  for (float v : x) u16((uint16_t)(int16_t)std::lround(std::max(-1.0f, std::min(1.0f, v)) * 32767.0f));
  std::fclose(f);
}

static long countColour(fw::Colour c, int y0 = 0, int y1 = fw::kDisplayH) {
  long n = 0;
  for (int y = y0; y < y1; ++y)
    for (int x = 0; x < fw::kDisplayW; ++x)
      if (fw::gfxGet(x, y) == c) ++n;
  return n;
}

static std::vector<uint8_t> grabFrame() { return std::vector<uint8_t>(fw::frame(), fw::frame() + fw::kFrameBytes); }

// ------------------------------------------------------------------ tests

static void testMath() {
  section("maths helpers match the C library");
  double worst2 = 0, worstL = 0, worstS = 0, worstT = 0;
  for (double x = -20; x <= 20; x += 0.0137) worst2 = std::max(worst2, std::fabs(fw::exp2f_((float)x) / std::exp2(x) - 1));
  for (double x = 0.001; x <= 30000; x *= 1.013) worstL = std::max(worstL, std::fabs(fw::log2f_((float)x) - std::log2(x)));
  for (double x = -30; x <= 30; x += 0.0071) worstS = std::max(worstS, std::fabs(fw::sinf_((float)x) - std::sin(x)));
  for (double x = 0.0005; x <= 1.42; x += 0.0031) worstT = std::max(worstT, std::fabs(fw::tanf_((float)x) / std::tan(x) - 1));
  CHECK(worst2 < 2e-6, "exp2 relative error %g", worst2);
  CHECK(worstL < 2e-5, "log2 error %g", worstL);
  CHECK(worstS < 5e-6, "sin error %g", worstS);
  CHECK(worstT < 2e-5, "tan relative error %g", worstT);
}

// R2: tuning and envelope times must not depend on the sample rate.
static void testSampleRateIndependence() {
  section("R2  tuning and envelope times are the same at 44.1 kHz and 48 kHz");
  double freq[2], attack[2];
  const double rates[2] = {44100.0, 48000.0};
  for (int r = 0; r < 2; ++r) {
    boot(rates[r]);
    cleanTone(fw::WAVE_SIN);
    midi({0x90, 69, 100});
    std::vector<float> x = renderNew(1.5);
    freq[r] = measureFreq(x, (size_t)(0.3 * gSr), x.size());
    midi({0x80, 69, 0});
    CHECK(std::fabs(freq[r] / 440.0 - 1) < 0.001, "A4 at %.0f Hz measured %.3f Hz", rates[r], freq[r]);

    // attack time: a 1760 Hz sine with a ~400 ms attack, timed to 90% of full level
    boot(rates[r]);
    cleanTone(fw::WAVE_SIN);
    setPot("aenv_attack", 0.65);
    renderNew(0.05);
    midi({0x90, 93, 100});
    x = renderNew(1.5);
    double full = peakOf(x, (size_t)(1.2 * gSr));
    size_t win = (size_t)(gSr / 1760.0) + 1;
    attack[r] = -1;
    for (size_t i = 0; i + win < x.size(); ++i) {
      if (peakOf(x, i, i + win) >= 0.9 * full) {
        attack[r] = (double)i / gSr;
        break;
      }
    }
    CHECK(attack[r] > 0.2 && attack[r] < 0.5, "attack reached 90%% after %.4f s at %.0f Hz", attack[r], rates[r]);
  }
  CHECK(std::fabs(freq[0] / freq[1] - 1) < 0.001, "pitch differs between rates: %.4f vs %.4f Hz", freq[0], freq[1]);
  CHECK(std::fabs(attack[0] / attack[1] - 1) < 0.02, "attack differs between rates: %.4f vs %.4f s", attack[0], attack[1]);
  std::printf("    A4: %.3f Hz @44.1k, %.3f Hz @48k.  attack: %.1f ms @44.1k, %.1f ms @48k\n", freq[0], freq[1],
              attack[0] * 1000, attack[1] * 1000);
}

// R5: high notes must not alias.
static void testAliasing() {
  section("R5  high saw and square notes: nothing louder than -60 dB between the harmonics");
  // First prove the measurement can fail: a textbook saw with no
  // band-limiting aliases badly at this pitch, and the check must see it.
  gSr = 48000.0;
  std::vector<float> naive(60000);
  double ph = 0;
  for (float& s : naive) {
    s = (float)(0.3 * (2 * ph - 1));
    ph += 2093.0 / gSr;
    if (ph >= 1) ph -= 1;
  }
  AliasResult bad = aliasCheck(naive);
  CHECK(bad.worstDb > -45.0, "the check missed the aliasing in a naive saw (%.1f dB)", bad.worstDb);
  std::printf("    naive saw, for comparison: worst %.1f dB\n", bad.worstDb);

  const double rates[2] = {48000.0, 44100.0};
  const int notes[3] = {96, 101, 108};  // C7 2093 Hz, F7 2794 Hz, C8 4186 Hz
  const int waves[2] = {fw::WAVE_SAW, fw::WAVE_SQR};
  const char* waveNames[2] = {"saw", "square"};
  for (double sr : rates)
    for (int w = 0; w < 2; ++w)
      for (int note : notes) {
        boot(sr);
        cleanTone(waves[w]);
        midi({0x90, note, 127});
        std::vector<float> x = renderNew(1.3);
        AliasResult a = aliasCheck(x);
        double expect = 440.0 * std::pow(2.0, (note - 69) / 12.0);
        CHECK(std::fabs(a.f0 / expect - 1) < 0.002, "%s note %d: fundamental %.1f Hz, expected %.1f", waveNames[w], note,
              a.f0, expect);
        CHECK(a.worstDb <= -60.0, "%s note %d at %.0f Hz: %.1f dB at %.0f Hz", waveNames[w], note, sr, a.worstDb,
              a.worstHz);
        // and it really is a saw or a square, not a sine
        if (waves[w] == fw::WAVE_SAW && a.harmonics >= 2)
          CHECK(a.h2Db > -9.0 && a.h2Db < -4.0, "saw note %d: 2nd harmonic %.1f dB, expected about -6", note, a.h2Db);
        if (waves[w] == fw::WAVE_SQR && a.harmonics >= 3) {
          CHECK(a.h3Db > -13.0 && a.h3Db < -7.5, "square note %d: 3rd harmonic %.1f dB, expected about -9.5", note,
                a.h3Db);
          CHECK(a.h2Db < -60.0, "square note %d: 2nd harmonic %.1f dB, expected none", note, a.h2Db);
        }
        std::printf("    %-6s note %3d @%.1fk: worst %.1f dB (%d harmonics below Nyquist)\n", waveNames[w], note,
                    sr / 1000, a.worstDb, a.harmonics);
      }
}

// R4: nothing in the firmware allocates memory.
static void testNoAllocation() {
  section("R4  no memory allocation, at start-up or while playing");
  float buf[fw::kMaxBlock], buf2[fw::kMaxBlock];
  gStoreLen = 0;
  gAllocs = 0;
  gCounting = true;
  fw::init(48000.0f);
  for (int round = 0; round < 40; ++round) {
    for (int n = 0; n < 10; ++n) {
      fw::midiByte(0x90);
      fw::midiByte((uint8_t)(40 + n * 3 + round % 5));
      fw::midiByte(100);
    }
    for (int c = 0; c < fw::kFirstEncoder; ++c) fw::controlSet(c, (round * 97 + c * 311) % 4096);
    fw::controlSet(fw::kFirstButton + round % fw::kNumButtons, 1);
    fw::encoderTurn(fw::kFirstEncoder + round % 4, round % 3 - 1);
    fw::controlSet(fw::kFirstEncoder + round % 4, 1);  // encoder pushes: on the Patch page these load and save
    fw::gPatch.save(round % fw::kPatchSlots);
    fw::midiByte(0xC0);                                // program change: loads a patch
    fw::midiByte((uint8_t)((round + 3) % fw::kPatchSlots));
    fw::midiByte(0xE0);
    fw::midiByte(0);
    fw::midiByte((uint8_t)(round * 3 % 128));
    for (int cc = 0; cc < 120; ++cc) {  // every controller number, so the soft parameters move too
      if (cc == 120 || cc == 123) continue;
      fw::midiByte(0xB0);
      fw::midiByte((uint8_t)cc);
      fw::midiByte((uint8_t)((round * 37 + cc * 11) % 128));
    }
    for (int b = 0; b < 30; ++b) fw::process(buf, buf2, fw::kMaxBlock);
    fw::tick(33);
    fw::tick(1200);  // long enough to trigger a state save
    for (int n = 0; n < 10; ++n) {
      fw::midiByte(0x80);
      fw::midiByte((uint8_t)(40 + n * 3 + round % 5));
      fw::midiByte(0);
    }
  }
  gCounting = false;
  CHECK(gAllocs == 0, "%ld allocations", gAllocs);
}

static void testVoices() {
  section("AC3  eight voices, the ninth note takes over the oldest");
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  for (int i = 0; i < 8; ++i) midi({0x90, 48 + i * 2, 100});
  renderNew(0.2);
  CHECK(fw::gSynth.activeVoices() == 8, "voices after 8 notes: %d", fw::gSynth.activeVoices());
  midi({0x90, 80, 100});
  renderNew(0.2);
  CHECK(fw::gSynth.activeVoices() == 8, "voices after 9 notes: %d", fw::gSynth.activeVoices());
  // Release notes 2..9 but never note 1. If note 1 had kept its voice it
  // would still be sounding; it was taken over, so everything dies away.
  for (int i = 1; i < 8; ++i) midi({0x80, 48 + i * 2, 0});
  midi({0x80, 80, 0});
  std::vector<float> tail = renderNew(1.5);
  CHECK(fw::gSynth.activeVoices() == 0, "voices still active: %d", fw::gSynth.activeVoices());
  CHECK(peakOf(tail, tail.size() - 4800) < 1e-4, "not silent: %g", peakOf(tail, tail.size() - 4800));

  section("AC3  velocity: harder is louder, by the amount on the Velocity knob");
  double level[2][2];
  for (int amt = 0; amt < 2; ++amt)
    for (int v = 0; v < 2; ++v) {
      boot(48000);
      cleanTone(fw::WAVE_SIN);
      setPot("aenv_velocity", amt ? 1.0 : 0.0);
      renderNew(0.05);
      midi({0x90, 69, v ? 127 : 40});
      level[amt][v] = peakOf(renderNew(0.5), 12000);
    }
  CHECK(std::fabs(level[0][0] / level[0][1] - 1) < 0.01, "velocity amount 0: soft %.4f vs hard %.4f", level[0][0],
        level[0][1]);
  CHECK(level[1][0] < 0.3 * level[1][1], "velocity amount full: soft %.4f vs hard %.4f", level[1][0], level[1][1]);

  section("AC3  level: a full chord stays clear of clipping, a single note is not too quiet");
  boot(48000);
  for (int i = 0; i < 8; ++i) midi({0x90, 36 + i * 5, 127});
  std::vector<float> chord = renderNew(1.0);
  bool finite = true;
  for (float s : chord) finite = finite && std::isfinite(s);
  CHECK(finite, "output has NaN or infinity");
  CHECK(peakOf(chord) < 0.99, "chord peak %.3f", peakOf(chord));
  boot(48000);
  midi({0x90, 60, 127});
  double one = peakOf(renderNew(0.5));
  CHECK(one > 0.1, "single note peak %.3f", one);
  std::printf("    default patch: single note peak %.3f, 8-note chord peak %.3f\n", one, peakOf(chord));
}

static void testPerformanceControls() {
  section("AC8  pitch bend, mod wheel and sustain pedal");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  midi({0x90, 69, 100});
  midi({0xE0, 0x7F, 0x7F});  // full bend up; default range is 2 semitones
  std::vector<float> x = renderNew(1.0);
  double f = measureFreq(x, 9600, x.size());
  CHECK(std::fabs(f / 493.883 - 1) < 0.002, "full bend up from A4: %.2f Hz, expected 493.88", f);
  midi({0xE0, 0x00, 0x40});
  x = renderNew(1.0);
  f = measureFreq(x, 9600, x.size());
  CHECK(std::fabs(f / 440.0 - 1) < 0.001, "bend centred: %.2f Hz", f);

  // mod wheel adds LFO depth (LFO depth knob at zero, destination pitch)
  std::vector<float> wheel[2];
  for (int m = 0; m < 2; ++m) {
    boot(48000);
    cleanTone(fw::WAVE_SIN);
    setSel("lfo_destination", fw::LFO_PITCH);
    midi({0xB0, 1, m ? 127 : 0});
    midi({0x90, 69, 100});
    wheel[m] = renderNew(0.5);
  }
  CHECK(rmsDiff(wheel[0], wheel[1]) > 0.01, "mod wheel made no difference (%.5f)", rmsDiff(wheel[0], wheel[1]));
  double fPlain = measureFreq(wheel[0], 4800, wheel[0].size());
  CHECK(std::fabs(fPlain / 440.0 - 1) < 0.001, "wheel at zero should leave the pitch alone: %.2f Hz", fPlain);

  // sustain pedal holds a released note
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  midi({0x90, 69, 100});
  midi({0xB0, 64, 127});
  renderNew(0.1);
  midi({0x80, 69, 0});
  x = renderNew(1.0);
  CHECK(peakOf(x, x.size() - 4800) > 0.05, "pedal did not hold the note: %.4f", peakOf(x, x.size() - 4800));
  midi({0xB0, 64, 0});
  x = renderNew(1.0);
  CHECK(peakOf(x, x.size() - 4800) < 1e-4, "note did not stop when the pedal came up");

  section("AC10  panic silences everything at once");
  boot(48000);
  for (int i = 0; i < 6; ++i) midi({0x90, 50 + i * 3, 110});
  midi({0xB0, 64, 127});
  renderNew(0.3);
  midi({0xB0, 120, 0});
  x = renderNew(0.05);
  CHECK(peakOf(x) < 1e-6, "sound after panic: %g", peakOf(x));
  CHECK(fw::gSynth.activeVoices() == 0, "voices after panic: %d", fw::gSynth.activeVoices());
}

// AC4: every panel control changes the sound.
static std::vector<float> phrase() {
  std::vector<float> x;
  midi({0x90, 55, 70});
  render(x, 0.12);
  midi({0x90, 62, 110});
  render(x, 0.25);
  midi({0x80, 55, 0});
  midi({0x80, 62, 0});
  render(x, 0.45);
  return x;
}

static void phraseBaseline() {
  // a patch in which every control has something to do
  setSel("osc1_wave", fw::WAVE_SQR);
  setSel("osc2_wave", fw::WAVE_SAW);
  setPot("osc1_macro", 0.4);
  setPot("osc2_detune", 0.7);
  setPot("mix_osc1", 0.5);
  setPot("mix_osc2", 0.5);
  setPot("mix_sub", 0.5);
  setPot("mix_noise", 0.3);
  setPot("filter_cutoff", 0.45);
  setPot("filter_resonance", 0.4);
  setPot("filter_env_amount", 0.8);
  setPot("filter_keytrack", 0.3);
  setPot("filter_drive", 0.3);
  setPot("voice_glide", 0.4);
  setPot("fenv_attack", 0.3);
  setPot("fenv_decay", 0.45);
  setPot("fenv_sustain", 0.4);
  setPot("fenv_release", 0.45);
  setPot("aenv_attack", 0.3);
  setPot("aenv_decay", 0.45);
  setPot("aenv_sustain", 0.5);
  setPot("aenv_release", 0.45);
  setPot("aenv_velocity", 0.5);
  setPot("lfo_rate", 0.8);
  setPot("lfo_depth", 0.5);
  setSel("lfo_shape", fw::LFOS_SINE);
  setSel("lfo_destination", fw::LFO_CUTOFF);
  setPot("volume", 0.6);
}

static void testEveryControl() {
  section("AC4  each of the 44 panel controls changes the sound");
  boot(48000);
  phraseBaseline();
  renderNew(0.05);
  std::vector<float> basePlain = phrase();
  boot(48000);
  phraseBaseline();
  setSel("arp_mode", fw::ARP_UP);
  renderNew(0.05);
  std::vector<float> baseArp = phrase();
  boot(48000);
  phraseBaseline();
  setPot("chorus", 0.7);
  renderNew(0.05);
  std::vector<float> baseChorus = phrase();
  int silent = 0;
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    boot(48000);
    phraseBaseline();
    int id = fw::controlParam(c);
    // Octaves, Tempo and Gate belong to the arpeggiator: they are tried with it running
    bool arpOnly = id == fw::P_ARP_OCTAVES || id == fw::P_ARP_TEMPO || id == fw::P_ARP_GATE;
    if (arpOnly) setSel("arp_mode", fw::ARP_UP);
    bool chorusOnly = id == fw::P_CHORUS_TYPE;  // Chorus or Ensemble: tried with the chorus turned up
    if (chorusOnly) setPot("chorus", 0.7);
    const std::vector<float>& base = arpOnly ? baseArp : chorusOnly ? baseChorus : basePlain;
    fw::ControlInfo info = fw::controlInfo(c);
    int cur = fw::controlGet(c);
    if (info.kind == fw::CK_SELECTOR) fw::controlSet(c, (cur + 1) % info.steps);
    else fw::controlSet(c, cur < 2048 ? 3700 : 400);
    renderNew(0.05);
    std::vector<float> x = phrase();
    double d = rmsDiff(base, x);
    if (d < 1e-4) {
      ++silent;
      CHECK(false, "control %s made no audible difference (%.6f)", info.name, d);
    }
  }
  CHECK(silent == 0, "%d controls did nothing", silent);
}

static void testMidi() {
  section("AC9  a controller number sets its parameter; the control stays put and follows until it is moved through the value");
  boot(48000);
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    fw::ControlInfo info = fw::controlInfo(c);
    int id = fw::controlParam(c);
    int cc = fw::kParams[id].cc;
    int was = fw::controlGet(c);
    midi({0xB0, cc, 0});
    midi({0xB0, cc, 100});
    int expect = info.kind == fw::CK_SELECTOR ? 100 * info.steps / 128 : (100 * 4095 + 63) / 127;
    CHECK(fw::gParams.val[id] == expect, "%s via CC %d: got %d, expected %d", info.name, cc, (int)fw::gParams.val[id], expect);
    CHECK(fw::gUi.popupParam() == id, "%s via CC did not bring up its visual", info.name);
    CHECK(fw::controlGet(c) == was, "%s: a real control cannot move by itself (was %d, now %d)", info.name, was, fw::controlGet(c));
    CHECK(fw::gPatch.following(c) == (was != expect), "%s should be following the incoming value", info.name);
    // dragged to the same place, the control takes over with the same value
    fw::controlSet(c, expect);
    CHECK(fw::gParams.val[id] == expect && !fw::gPatch.following(c), "%s: dragging to the value should take over there",
          info.name);
    fw::controlSet(c, info.kind == fw::CK_SELECTOR ? 0 : 500);
    CHECK(fw::gParams.val[id] == (info.kind == fw::CK_SELECTOR ? 0 : 500), "%s: once live the control sets the value", info.name);
  }
  // no two parameters share a number, and none collides with the reserved ones
  for (int a = 0; a < fw::P_COUNT; ++a) {
    int cc = fw::kParams[a].cc;
    CHECK(cc != 1 && cc != 64 && cc != 120 && cc != 123 && cc > 0 && cc < 120, "%s has reserved CC %d",
          fw::kParams[a].key, cc);
    for (int b = a + 1; b < fw::P_COUNT; ++b)
      CHECK(cc != fw::kParams[b].cc, "%s and %s share CC %d", fw::kParams[a].key, fw::kParams[b].key, cc);
  }

  section("MIDI parsing: running status, note-on with velocity 0, stray bytes");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  midi({0x90, 60, 100, 64, 100, 67, 100});  // three notes on one status byte
  midi({0xF8});                             // clock byte in between
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() == 3, "running status: %d voices", fw::gSynth.activeVoices());
  midi({0x90, 60, 0, 64, 0, 67, 0});        // note-on velocity 0 = note-off
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 0, "velocity-0 note-offs: %d voices left", fw::gSynth.activeVoices());
  midi({0xF0, 0x7D, 0x3C, 0x7F, 0x3C, 0x7F, 0xF7});  // sysex contents must not play as running-status notes
  midi({0xC0, 5});                             // program change to an empty slot: nothing happens
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() == 0, "sysex or program change started a voice");
}

static void testUi() {
  section("AC5  home screen: patch name, live scope, status strip");
  boot(48000);
  fw::tick(33);
  CHECK(countColour(fw::kAccent, 0, 30) > 100, "no patch name in the title row");
  long quietScope = countColour(fw::kLine, 40, 208);
  for (int i = 0; i < 3; ++i) midi({0x90, 50 + i * 7, 110});
  renderNew(0.3);
  fw::tick(33);
  saveFrame("home");
  CHECK(countColour(fw::kLine, 40, 208) > quietScope + 200, "scope shows no waveform while notes sound");
  CHECK(countColour(fw::kLine, 222, 236) == 3 * 64, "status strip should show 3 voices lit");
  CHECK(countColour(fw::kAccent, 222, 236) >= 60, "MIDI activity light is not lit");
  midi({0xB0, 120, 0});

  section("AC6  moving a control shows its visual, home returns after 1.5 s");
  boot(48000);
  fw::tick(33);
  std::vector<uint8_t> home = grabFrame();
  static const char* const kShots[] = {"osc1_wave",     "osc1_macro",      "osc2_detune",  "mix_sub",
                                       "filter_cutoff", "filter_resonance", "filter_type",  "fenv_decay",
                                       "aenv_release",  "aenv_velocity",   "lfo_shape",    "lfo_destination",
                                       "voice_glide",   "voice_mode",      "volume"};
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    fw::ControlInfo info = fw::controlInfo(c);
    int cur = fw::controlGet(c);
    fw::controlSet(c, info.kind == fw::CK_SELECTOR ? (cur + 1) % info.steps : (cur < 2048 ? 3000 : 1000));
    CHECK(fw::tick(33), "%s: display did not redraw", info.name);
    CHECK(fw::gUi.popupParam() == fw::controlParam(c), "%s: visual not shown", info.name);
    CHECK(grabFrame() != home, "%s: display still shows the home screen", info.name);
    CHECK(countColour(fw::kAccent, 0, 44) > 60 && countColour(fw::kInk, 0, 44) > 60, "%s: no name or value in the header",
          info.name);
    for (const char* shot : kShots)
      if (!std::strcmp(shot, info.name)) saveFrame((std::string("popup_") + info.name).c_str());
  }
  fw::tick(1400);
  CHECK(fw::gUi.popupParam() >= 0, "visual vanished before 1.5 s");
  fw::tick(200);
  CHECK(fw::gUi.popupParam() < 0, "visual still up after 1.6 s");
  CHECK(countColour(fw::kAccent, 0, 30) > 100, "home screen did not come back");

  section("AC7  Engine page and the oscillator engine buttons");
  boot(48000);
  int enc1 = ctl("enc1"), enc4 = ctl("enc4");
  int before = fw::gParams.step(fw::P_UNI_DETUNE);
  fw::encoderTurn(enc1, 3);
  CHECK(fw::gParams.step(fw::P_UNI_DETUNE) == before, "encoder changed a value while on the home screen");

  press("btn_engine");
  fw::tick(33);
  saveFrame("mode_engine");
  CHECK(fw::gUi.mode() == fw::MB_ENGINE, "Engine button did not open the Engine page");
  CHECK(fw::controlGet(ctl("btn_engine")) == 1, "Engine button light is off");
  std::vector<uint8_t> enginePage = grabFrame();

  fw::encoderTurn(enc1, 3);
  CHECK(fw::gParams.step(fw::P_UNI_DETUNE) == before + 3, "enc1 should raise unison detune by 3");
  fw::encoderTurn(ctl("enc2"), 1);
  CHECK(fw::gParams.step(fw::P_SUB_DETUNE) + fw::kParams[fw::P_SUB_DETUNE].intMin == 1, "enc2 should detune the sub by one cent");
  fw::encoderTurn(ctl("enc3"), 5);
  CHECK(fw::gParams.step(fw::P_BEND_RANGE) + fw::kParams[fw::P_BEND_RANGE].intMin == 7, "enc3 should set bend range to 7");
  fw::encoderTurn(enc4, -1);
  CHECK(fw::gParams.step(fw::P_FILT_SLOPE) == 0, "enc4 should switch the filter to 12 dB");
  fw::encoderTurn(enc4, -5);
  CHECK(fw::gParams.step(fw::P_FILT_SLOPE) == 0, "turning past the end should stop at the end");
  fw::tick(33);
  CHECK(grabFrame() != enginePage, "Engine page did not show the new values");
  CHECK(fw::gUi.popupParam() < 0, "Engine page values should update in place, not pop up");

  fw::controlSet(enc1, 1);  // push = back to default
  fw::controlSet(enc1, 0);
  CHECK(fw::gParams.step(fw::P_UNI_DETUNE) == fw::kParams[fw::P_UNI_DETUNE].def, "pushing enc1 should reset unison detune");

  // the Engine mode has a second page: its lit button steps to it, then home
  press("btn_engine");
  fw::tick(33);
  saveFrame("mode_engine_2");
  CHECK(fw::gUi.mode() == fw::MB_ENGINE && fw::gUi.page() == 1, "second press should show Engine page 2");
  CHECK(fw::controlGet(ctl("btn_engine")) == 1, "Engine light should stay on for page 2");
  press("btn_engine");
  CHECK(fw::gUi.mode() == fw::MB_NONE, "third press should return to the home screen");
}

static void testSoftPages() {
  section("soft pages: FX 1, FX 2 and Mod each put four values on the encoders");
  struct PageCase {
    const char* button;
    const char* shot;
    int params[4];
  };
  static const PageCase kCases[] = {
      {"btn_fx1", "mode_fx1", {fw::P_DLY_TIME, fw::P_DLY_TONE, fw::P_DLY_FDBK, fw::P_DLY_MIX}},
      {"btn_fx2", "mode_fx2", {fw::P_COMPRESS, fw::P_RVB_SIZE, fw::P_RVB_TONE, fw::P_RVB_MIX}},
      {"btn_mod", "mode_mod", {fw::P_LFO_FADE, fw::P_VEL_CUTOFF, fw::P_FENV_PITCH, fw::P_DUCK}},
  };
  static const char* const kEncoders[4] = {"enc1", "enc2", "enc3", "enc4"};
  boot(48000);
  for (const PageCase& c : kCases) {
    press(c.button);
    fw::tick(33);
    CHECK(fw::controlGet(ctl(c.button)) == 1, "%s: light is off", c.button);
    CHECK(countColour(fw::kInk, 44, 70) > 100 && countColour(fw::kInk, 178, 204) > 100, "%s: values missing", c.button);
    for (int e = 0; e < 4; ++e) {
      int id = c.params[e];
      int v0 = fw::gParams.step(id);
      fw::encoderTurn(ctl(kEncoders[e]), 1);
      CHECK(fw::gParams.step(id) == v0 + 1, "%s %s: one click should move %s by one step", c.button, kEncoders[e], fw::kParams[id].key);
      CHECK(fw::gUi.popupParam() < 0, "%s: a value on the open page should update in place", c.button);
      fw::tick(33);
      fw::controlSet(ctl(kEncoders[e]), 1);  // push = default
      fw::controlSet(ctl(kEncoders[e]), 0);
      CHECK(fw::gParams.step(id) == fw::kParams[id].def, "%s %s: push should restore the default", c.button, kEncoders[e]);
    }
    fw::tick(33);
    saveFrame(c.shot);
    press(c.button);
    if (!std::strcmp(c.button, "btn_fx2")) {  // FX 2 has a second page, the vocoder
      CHECK(fw::gUi.mode() == fw::MB_FX2 && fw::gUi.page() == 1, "btn_fx2: a second press should show the Vocoder page");
      fw::tick(33);
      saveFrame("mode_vocoder_off");
      press(c.button);
    }
    CHECK(fw::gUi.mode() == fw::MB_NONE, "%s: pressing the lit button should return home", c.button);
  }

  section("soft pages: a fast spin of an encoder covers a long range quickly");
  press("btn_fx1");
  fw::tick(33);
  int enc = ctl("enc4");
  for (int i = 0; i < 8; ++i) {  // eight slow clicks, a display tick apart
    fw::encoderTurn(enc, 1);
    fw::tick(33);
  }
  CHECK(fw::gParams.step(fw::P_DLY_MIX) == 8, "slow clicks should move one step each (got %d)", fw::gParams.step(fw::P_DLY_MIX));
  for (int i = 0; i < 8; ++i) fw::encoderTurn(enc, 1);  // eight clicks inside one tick
  CHECK(fw::gParams.step(fw::P_DLY_MIX) > 8 + 20, "a fast spin should move in bigger steps (got %d)", fw::gParams.step(fw::P_DLY_MIX));
  press("btn_fx1");

  section("soft pages: turning the encoders on the Patch pages changes no sound parameter");
  press("btn_patch");
  fw::tick(33);
  uint16_t before[fw::P_COUNT];
  for (int i = 0; i < fw::P_COUNT; ++i) before[i] = fw::gParams.val[i];
  for (int e = 0; e < 4; ++e) fw::encoderTurn(ctl(kEncoders[e]), 2);
  press("btn_patch");
  for (int e = 0; e < 4; ++e) fw::encoderTurn(ctl(kEncoders[e]), 2);
  int changed = 0;
  for (int i = 0; i < fw::P_COUNT; ++i) changed += before[i] != fw::gParams.val[i];
  CHECK(changed == 0, "browsing slots or editing the name changed %d parameters", changed);
  press("btn_patch");
  CHECK(fw::gUi.mode() == fw::MB_PATCH && fw::gUi.page() == 2, "third press should show the Setup page");
  press("btn_patch");
  CHECK(fw::gUi.mode() == fw::MB_PATCH && fw::gUi.page() == 3, "fourth press should show the MIDI page");
  press("btn_patch");
  CHECK(fw::gUi.mode() == fw::MB_PATCH && fw::gUi.page() == 4, "fifth press should show the Input page");
  press("btn_patch");
  CHECK(fw::gUi.mode() == fw::MB_NONE, "the Patch button should return home after its fifth page");

  section("oscillator engine buttons step through VA, FM, wavetable and pluck");
  boot(48000);
  static const char* const kNames[4] = {"VA", "FM", "WAVETABLE", "PLUCK"};
  for (int i = 1; i <= 4; ++i) {
    press("btn_osc1_engine");
    fw::tick(33);
    int e = fw::gParams.step(fw::P_OSC1_ENGINE);
    CHECK(e == i % 4, "press %d: engine %s, expected %s", i, kNames[e], kNames[i % 4]);
    CHECK(fw::gUi.popupParam() == fw::P_OSC1_ENGINE, "the display should show the engine");
    CHECK(fw::controlGet(ctl("btn_osc1_engine")) == (e != fw::ENG_VA), "the button is lit for any engine but VA");
    if (i == 1) saveFrame("popup_osc_engine");
  }
  CHECK(fw::gParams.step(fw::P_OSC2_ENGINE) == fw::ENG_VA, "oscillator 2 should not have changed");

  section("Engine page 2 follows each oscillator's engine");
  fw::tick(1600);
  press("btn_engine");
  press("btn_engine");
  int enc1 = ctl("enc1"), enc2 = ctl("enc2"), enc3 = ctl("enc3");
  fw::encoderTurn(enc1, 3);
  CHECK(fw::gParams.step(fw::P_OSC1_COLOR) == 0, "VA has no second timbre control: encoder 1 should do nothing");
  fw::encoderTurn(enc2, 3);
  fw::tick(33);
  fw::encoderTurn(enc3, 3);
  CHECK(fw::gParams.step(fw::P_OSC2_MACRO) > 0, "encoder 3 should set oscillator 2's Macro");
  press("btn_osc1_engine");  // -> FM, which does have one (feedback)
  fw::tick(1600);
  fw::encoderTurn(enc1, 3);
  CHECK(fw::gParams.step(fw::P_OSC1_COLOR) > 0, "FM: encoder 1 should set feedback");
  press("btn_osc2_engine");
  press("btn_osc2_engine");
  press("btn_osc2_engine");  // osc 2 -> pluck
  fw::tick(1600);
  fw::tick(33);
  saveFrame("mode_engine_2_fm_pluck");

  section("pop-ups name the Wave selector and Macro by engine");
  setSel("osc1_wave", 2);
  fw::tick(33);
  saveFrame("popup_fm_ratio");
  setPot("osc1_macro", 0.7);
  fw::tick(33);
  saveFrame("popup_fm_amount");
  setSel("osc2_wave", 3);
  fw::tick(33);
  saveFrame("popup_pluck_exciter");
  midi({0xB0, fw::kParams[fw::P_OSC1_ENGINE].cc, 80});  // wavetable
  setPot("osc1_macro", 0.5);
  fw::tick(33);
  saveFrame("popup_wavetable_position");

  section("Hold button latches, lights, and shows on the display");
  boot(48000);
  fw::tick(33);
  long accentBefore = countColour(fw::kAccent, 222, 236);
  press("btn_hold");
  CHECK(fw::gParams.step(fw::P_HOLD) == 1 && fw::controlGet(ctl("btn_hold")) == 1, "Hold should be on and lit");
  fw::tick(1600);
  fw::tick(33);
  CHECK(countColour(fw::kAccent, 222, 236) > accentBefore + 60, "status strip should show HOLD");
  saveFrame("home_hold");
  press("btn_hold");
  CHECK(fw::gParams.step(fw::P_HOLD) == 0 && fw::controlGet(ctl("btn_hold")) == 0, "Hold should be off again");
  setCC(fw::P_HOLD, 127);
  CHECK(fw::controlGet(ctl("btn_hold")) == 1, "controller 69 should switch Hold on");
}

// ---- helpers for the note-level tests ----

// Sets a stepped parameter to an exact step through its controller number.
static void setStep(int param, int step) {
  int steps = fw::kParams[param].steps;
  for (int v = 0; v < 128; ++v)
    if (v * steps / 128 == step) {
      setCC(param, v);
      return;
    }
  std::printf("    cannot reach step %d of %s by controller\n", step, fw::kParams[param].key);
  std::exit(2);
}

static double rmsOf(const std::vector<float>& x, double fromSec, double toSec) {
  size_t a = (size_t)(fromSec * gSr), b = std::min(x.size(), (size_t)(toSec * gSr));
  double s = 0;
  for (size_t i = a; i < b; ++i) s += (double)x[i] * x[i];
  return b > a ? std::sqrt(s / (double)(b - a)) : 0;
}

// The MIDI note sounding in each arpeggiator step (0 = silence).
static std::vector<int> stepNotes(const std::vector<float>& x, double stepSec, int count) {
  std::vector<int> notes;
  for (int i = 0; i < count; ++i) {
    double t0 = i * stepSec;
    if (rmsOf(x, t0 + 0.1 * stepSec, t0 + 0.4 * stepSec) < 0.005) {
      notes.push_back(0);
      continue;
    }
    double f = measureFreq(x, (size_t)((t0 + 0.08 * stepSec) * gSr), (size_t)((t0 + 0.45 * stepSec) * gSr));
    notes.push_back(f > 0 ? (int)std::lround(69 + 12 * std::log2(f / 440.0)) : 0);
  }
  return notes;
}

static std::string show(const std::vector<int>& v) {
  std::string s;
  for (int n : v) s += std::to_string(n) + " ";
  return s;
}

static void arpTone() {
  cleanTone(fw::WAVE_SIN);
  setPot("aenv_release", 0.1);
  renderNew(0.05);
}

static void testArpAndHold() {
  const double step = 0.125;  // sixteenth notes at the default 120 BPM
  section("arp: patterns play the held notes in the right order, eight to the second");
  struct Case {
    int mode;
    int octaves;
    std::vector<int> keys;
    std::vector<int> expect;
  };
  const Case cases[] = {
      {fw::ARP_UP, 1, {64, 60, 67}, {60, 64, 67, 60, 64, 67, 60, 64}},
      {fw::ARP_DOWN, 1, {64, 60, 67}, {67, 64, 60, 67, 64, 60, 67, 64}},
      {fw::ARP_UPDOWN, 1, {64, 60, 67}, {60, 64, 67, 64, 60, 64, 67, 64}},
      {fw::ARP_PLAYED, 1, {64, 60, 67}, {64, 60, 67, 64, 60, 67, 64, 60}},
      {fw::ARP_UP, 2, {60, 64, 67}, {60, 64, 67, 72, 76, 79, 60, 64}},
      {fw::ARP_DOWN, 2, {60, 64}, {76, 72, 64, 60, 76, 72, 64, 60}},
  };
  for (const Case& c : cases) {
    boot(48000);
    arpTone();
    setStep(fw::P_ARP_MODE, c.mode);
    setStep(fw::P_ARP_OCTAVES, c.octaves - 1);
    renderNew(0.05);
    for (int k : c.keys) midi({0x90, k, 100});
    std::vector<float> x = renderNew(1.0);
    std::vector<int> got = stepNotes(x, step, 8);
    CHECK(got == c.expect, "%s x%d: got %s", fw::kParams[fw::P_ARP_MODE].labels[c.mode], c.octaves, show(got).c_str());
    for (int k : c.keys) midi({0x80, k, 0});
    x = renderNew(0.6);
    CHECK(rmsOf(x, 0.3, 0.6) < 1e-4, "%s: should stop when the keys are let go", fw::kParams[fw::P_ARP_MODE].labels[c.mode]);
  }

  boot(48000);
  arpTone();
  setStep(fw::P_ARP_MODE, fw::ARP_RANDOM);
  renderNew(0.05);
  for (int k : {60, 64, 67, 71}) midi({0x90, k, 100});
  std::vector<int> rnd = stepNotes(renderNew(3.0), step, 24);
  bool inSet = true, varied = false;
  for (int n : rnd) inSet = inSet && (n == 60 || n == 64 || n == 67 || n == 71);
  for (size_t i = 1; i < rnd.size(); ++i) varied = varied || rnd[i] != rnd[0];
  CHECK(inSet && varied, "random: got %s", show(rnd).c_str());

  section("arp: Gate sets how much of each step sounds");
  for (int gate : {9, 19}) {  // 50% and 100%
    boot(48000);
    arpTone();
    setStep(fw::P_ARP_MODE, fw::ARP_UP);
    setPot("arp_gate", gate == 9 ? 0.474 : 1.0);
    renderNew(0.05);
    for (int k : {60, 64, 67}) midi({0x90, k, 100});
    std::vector<float> x = renderNew(1.0);
    double early = rmsOf(x, 2 * step + 0.1 * step, 2 * step + 0.4 * step);
    double late = rmsOf(x, 2 * step + 0.7 * step, 2 * step + 0.95 * step);
    if (gate == 9) CHECK(early > 0.05 && late < 0.1 * early, "50%%: first half %.4f, second half %.4f", early, late);
    else CHECK(late > 0.5 * early, "100%%: first half %.4f, second half %.4f", early, late);
  }

  section("Hold with the arp: it keeps running hands-free, and a new chord replaces the old");
  boot(48000);
  arpTone();
  setStep(fw::P_ARP_MODE, fw::ARP_UP);
  press("btn_hold");
  renderNew(0.05);
  for (int k : {60, 64, 67}) midi({0x90, k, 100});
  renderNew(0.25);
  for (int k : {60, 64, 67}) midi({0x80, k, 0});
  std::vector<int> held = stepNotes(renderNew(1.0), step, 8);
  int sounding = 0;
  for (int n : held) sounding += n != 0;
  CHECK(sounding == 8, "arp should keep playing after the keys are let go: %s", show(held).c_str());
  midi({0x90, 72, 100});
  midi({0x90, 76, 100});
  std::vector<int> fresh = stepNotes(renderNew(0.5), step, 4);
  CHECK((fresh == std::vector<int>{72, 76, 72, 76}), "a new chord should replace the held one: %s", show(fresh).c_str());
  midi({0x80, 72, 0});
  midi({0x80, 76, 0});
  renderNew(0.5);
  press("btn_hold");  // Hold off: nothing has a finger on it, so it stops
  std::vector<float> x = renderNew(0.6);
  CHECK(rmsOf(x, 0.3, 0.6) < 1e-4, "switching Hold off should stop the arp");

  section("Hold without the arp: a chord rings on, the next chord replaces it");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  press("btn_hold");
  midi({0x90, 60, 100});
  midi({0x90, 64, 100});
  renderNew(0.2);
  midi({0x80, 60, 0});
  midi({0x80, 64, 0});
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 2, "held chord: %d voices sounding, expected 2", fw::gSynth.activeVoices());
  midi({0x90, 72, 100});
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 1, "after a new note: %d voices, expected only the new one", fw::gSynth.activeVoices());
  x = renderNew(0.3);
  double f = measureFreq(x, 0, x.size());
  CHECK(std::fabs(f / 523.25 - 1) < 0.002, "the note still sounding should be the new one (%.1f Hz)", f);
  midi({0x80, 72, 0});
  renderNew(0.5);
  CHECK(fw::gSynth.activeVoices() == 1, "the new note should hold too");
  press("btn_hold");
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 0, "Hold off should let everything go (%d voices)", fw::gSynth.activeVoices());

  section("a key still held when Hold goes off keeps sounding");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  press("btn_hold");
  midi({0x90, 60, 100});
  midi({0x90, 64, 100});
  renderNew(0.1);
  midi({0x80, 64, 0});
  press("btn_hold");
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 1, "%d voices, expected the one key still down", fw::gSynth.activeVoices());

  section("arp: the sustain pedal acts as Hold while the arp runs; switching the arp off hands the notes back");
  boot(48000);
  arpTone();
  setStep(fw::P_ARP_MODE, fw::ARP_UP);
  renderNew(0.05);
  midi({0xB0, 64, 127});
  for (int k : {60, 64, 67}) midi({0x90, k, 100});
  renderNew(0.25);  // two whole steps, so the next render starts on a step
  for (int k : {60, 64, 67}) midi({0x80, k, 0});
  held = stepNotes(renderNew(0.5), step, 4);
  sounding = 0;
  for (int n : held) sounding += n != 0;
  CHECK(sounding == 4, "pedal down: arp should continue (%s)", show(held).c_str());
  midi({0xB0, 64, 0});
  x = renderNew(0.6);
  CHECK(rmsOf(x, 0.3, 0.6) < 1e-4, "pedal up: arp should stop");

  for (int k : {60, 64, 67}) midi({0x90, k, 100});
  renderNew(0.3);
  setStep(fw::P_ARP_MODE, fw::ARP_OFF);
  renderNew(0.5);
  CHECK(fw::gSynth.activeVoices() == 3, "arp off with keys held: %d voices, expected the chord of 3", fw::gSynth.activeVoices());
  setStep(fw::P_ARP_MODE, fw::ARP_UP);
  renderNew(0.5);
  CHECK(fw::gSynth.activeVoices() <= 1, "arp on again: %d voices, expected one note at a time", fw::gSynth.activeVoices());
  midi({0xB0, 120, 0});
  x = renderNew(0.3);
  CHECK(peakOf(x) < 1e-6, "panic should stop the arp too");
}

static void testEffects() {
  section("effects: with everything off, left and right are the same dry signal");
  boot(48000);
  for (int k : {48, 55, 60, 64}) midi({0x90, k, 100});
  std::vector<float> l = renderNew(0.5);
  CHECK(rmsDiff(l, gRight) == 0, "left and right differ with no effect on (%.8f)", rmsDiff(l, gRight));
  std::vector<float> dry = l;

  section("effects: chorus widens the sound");
  boot(48000);
  setCC(fw::P_CHORUS, 110);
  for (int k : {48, 55, 60, 64}) midi({0x90, k, 100});
  l = renderNew(0.5);
  CHECK(rmsDiff(l, gRight) > 0.005, "left and right should differ (%.5f)", rmsDiff(l, gRight));
  CHECK(rmsDiff(l, dry) > 0.005, "chorus made no difference to the sound");

  section("effects: delay repeats at the set time, each repeat quieter, at either sample rate");
  const double echo = 0.02 * std::pow(2.0, 0.55 * 5.2288);  // the default Time setting
  for (double sr : {48000.0, 44100.0}) {
    boot(sr);
    cleanTone(fw::WAVE_SAW);
    setPot("aenv_release", 0.1);
    setCC(fw::P_DLY_MIX, 127);
    setCC(fw::P_DLY_FDBK, 64);
    renderNew(0.6);  // the delay time settles
    midi({0x90, 60, 110});
    std::vector<float> x = renderNew(0.04);
    midi({0x80, 60, 0});
    render(x, 0.8);
    double gap1 = rmsOf(x, echo - 0.05, echo - 0.01), rep1 = rmsOf(x, echo + 0.005, echo + 0.04);
    double gap2 = rmsOf(x, 2 * echo - 0.05, 2 * echo - 0.01), rep2 = rmsOf(x, 2 * echo + 0.005, 2 * echo + 0.04);
    CHECK(rep1 > 5 * gap1 && rep1 > 0.01, "@%.0f first repeat at %.0f ms: %.4f against %.4f just before", sr, echo * 1000, rep1, gap1);
    CHECK(rep2 > 3 * gap2, "@%.0f second repeat: %.4f against %.4f just before", sr, rep2, gap2);
    CHECK(rep2 < 0.8 * rep1, "@%.0f repeats should get quieter: %.4f then %.4f", sr, rep1, rep2);
  }

  section("effects: reverb leaves a tail that dies away, longer with a bigger Size");
  double tail[2];
  for (int size = 0; size < 2; ++size) {
    boot(48000);
    cleanTone(fw::WAVE_SAW);
    setPot("aenv_release", 0.1);
    setCC(fw::P_RVB_MIX, 127);
    setCC(fw::P_RVB_SIZE, size ? 110 : 20);
    renderNew(0.1);
    midi({0x90, 60, 110});
    std::vector<float> x = renderNew(0.05);
    midi({0x80, 60, 0});
    render(x, 4.0);
    tail[size] = rmsOf(x, 1.0, 1.5);
    bool finite = true;
    for (float v : x) finite = finite && std::isfinite(v);
    CHECK(finite, "reverb output has NaN or infinity");
    CHECK(rmsOf(x, 0.2, 0.4) > 10 * rmsOf(x, 3.6, 4.0) || size, "small room should have died away by 4 s");
    if (size) {
      CHECK(rmsOf(x, 0.5, 1.0) > rmsOf(x, 3.0, 3.5), "the tail should fall, not grow");
      std::vector<float> lr(x.begin(), x.end());
      CHECK(rmsDiff(lr, gRight) > 1e-4, "the tail should be different left and right");
    }
  }
  CHECK(tail[1] > 5 * tail[0], "tail a second later: small %.5f, large %.5f", tail[0], tail[1]);

  section("effects: all on with a full chord stays finite and clear of hard clipping");
  boot(48000);
  setCC(fw::P_CHORUS, 127);
  setCC(fw::P_DLY_MIX, 127);
  setCC(fw::P_DLY_FDBK, 127);
  setCC(fw::P_RVB_MIX, 127);
  setCC(fw::P_RVB_SIZE, 127);
  for (int i = 0; i < 8; ++i) midi({0x90, 36 + i * 5, 127});
  l = renderNew(6.0);
  bool finite = true;
  for (float v : l) finite = finite && std::isfinite(v);
  CHECK(finite && peakOf(l) < 0.99, "peak %.3f", peakOf(l));
  std::printf("    worst case, all effects full with eight voices: peak %.3f\n", peakOf(l));
  saveSound("all_effects_eight_voices", l);
  boot(48000);  // the same with the compressor at full as well: its make-up gain must not push the chord into the limiter
  setCC(fw::P_CHORUS, 127);
  setCC(fw::P_DLY_MIX, 127);
  setCC(fw::P_DLY_FDBK, 127);
  setCC(fw::P_RVB_MIX, 127);
  setCC(fw::P_RVB_SIZE, 127);
  setStep(fw::P_COMPRESS, 100);
  for (int i = 0; i < 8; ++i) midi({0x90, 36 + i * 5, 127});
  l = renderNew(6.0);
  finite = true;
  for (float v : l) finite = finite && std::isfinite(v);
  CHECK(finite && peakOf(l) < 0.98, "with the compressor at full too: peak %.3f", peakOf(l));
  std::printf("    and with the compressor at full as well: peak %.3f\n", peakOf(l));
}

// Spread of the gaps between upward zero crossings: how much the pitch wobbles.
static double pitchSpread(const std::vector<float>& x, double fromSec, double toSec) {
  double last = -1, lo = 1e9, hi = 0;
  for (size_t i = (size_t)(fromSec * gSr) + 1; i < (size_t)(toSec * gSr) && i < x.size(); ++i)
    if (x[i - 1] < 0 && x[i] >= 0) {
      double t = (double)(i - 1) + (double)(-x[i - 1]) / (double)(x[i] - x[i - 1]);
      if (last >= 0) {
        lo = std::min(lo, t - last);
        hi = std::max(hi, t - last);
      }
      last = t;
    }
  return hi > 0 ? (hi - lo) / lo : 0;
}

static void testMod() {
  section("Mod: Env>Pitch bends the pitch up by the set amount, following the filter envelope");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  setPot("fenv_attack", 0.0);
  setPot("fenv_decay", 0.8);  // about 1.6 s
  setPot("fenv_sustain", 0.0);
  setStep(fw::P_FENV_PITCH, 12);
  renderNew(0.05);
  midi({0x90, 69, 100});
  std::vector<float> x = renderNew(4.0);
  double early = measureFreq(x, (size_t)(0.002 * gSr), (size_t)(0.02 * gSr));
  double late = measureFreq(x, (size_t)(3.5 * gSr), x.size());
  CHECK(early > 440 * 1.6 && early < 440 * 2.05, "start of the note: %.1f Hz, expected nearly an octave up", early);
  CHECK(std::fabs(late / 440.0 - 1) < 0.01, "once the envelope has fallen: %.1f Hz", late);

  section("Mod: Vel>Cutoff makes harder notes brighter");
  double bright[2][2];
  for (int amt = 0; amt < 2; ++amt)
    for (int hard = 0; hard < 2; ++hard) {
      boot(48000);
      cleanTone(fw::WAVE_SAW);
      setPot("filter_cutoff", 0.3);
      setCC(fw::P_VEL_CUTOFF, amt ? 127 : 0);
      renderNew(0.05);
      midi({0x90, 45, hard ? 127 : 30});
      bright[amt][hard] = rmsOf(renderNew(0.6), 0.2, 0.6);
    }
  CHECK(std::fabs(bright[0][1] / bright[0][0] - 1) < 0.02, "with the amount at zero velocity should not matter (%.4f, %.4f)", bright[0][0], bright[0][1]);
  CHECK(bright[1][1] > 1.5 * bright[1][0], "with it up, hard %.4f should carry more than soft %.4f", bright[1][1], bright[1][0]);

  section("Mod: LFO Fade brings the wobble in gradually after each note starts");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  setSel("lfo_destination", fw::LFO_PITCH);
  setPot("lfo_depth", 0.5);
  setPot("lfo_rate", 0.8);
  setCC(fw::P_LFO_FADE, 83);  // about one second
  renderNew(0.05);
  midi({0x90, 69, 100});
  x = renderNew(3.0);
  double start = pitchSpread(x, 0.0, 0.2), settled = pitchSpread(x, 2.0, 3.0);
  CHECK(settled > 0.05 && start < 0.35 * settled, "pitch wobble at the start %.4f, once faded in %.4f", start, settled);

  section("Mod: LFO Sync restarts the LFO with a note played from silence");
  for (int sync = 0; sync < 2; ++sync) {
    std::vector<float> take[2];
    for (int t = 0; t < 2; ++t) {
      boot(48000);
      cleanTone(fw::WAVE_SAW);
      setSel("lfo_destination", fw::LFO_CUTOFF);
      setPot("lfo_depth", 0.7);
      setPot("lfo_rate", 0.75);
      setPot("filter_cutoff", 0.5);
      setStep(fw::P_LFO_SYNC, sync);
      renderNew(t ? 0.18 : 0.1);  // wait a different time before playing (about half an LFO cycle apart)
      midi({0x90, 57, 100});
      take[t] = renderNew(0.5);
    }
    double d = rmsDiff(take[0], take[1]);
    if (sync) CHECK(d < 1e-6, "KEY: the note should sound the same whenever it is played (%.6f)", d);
    else CHECK(d > 0.003, "FREE: the LFO should be somewhere different each time (%.6f)", d);
  }
}

static void engineTone(int osc1Engine, int wave) {
  cleanTone(wave);
  setStep(fw::P_OSC1_ENGINE, osc1Engine);
  renderNew(0.05);
}

static void testEngines() {
  static const char* const kNames[4] = {"VA", "FM", "wavetable", "pluck"};

  section("engines: every engine and every Wave position gives its own sound at a sensible level");
  std::vector<std::vector<float>> sounds;
  for (int e = 0; e < 4; ++e)
    for (int w = 0; w < 5; ++w) {
      boot(48000);
      engineTone(e, w);
      setPot("osc1_macro", 0.6);
      renderNew(0.05);
      midi({0x90, 57, 110});
      std::vector<float> x = renderNew(0.4);
      bool finite = true;
      for (float v : x) finite = finite && std::isfinite(v);
      double pk = peakOf(x);
      CHECK(finite && pk > 0.03 && pk < 0.6, "%s position %d: peak %.3f", kNames[e], w, pk);
      for (size_t k = 0; k < sounds.size(); ++k)
        CHECK(rmsDiff(x, sounds[k]) > 0.003, "%s position %d sounds the same as case %zu", kNames[e], w, k);
      sounds.push_back(x);
    }

  section("engines: Macro and Color do something where the engine has them, on either oscillator");
  for (int osc = 0; osc < 2; ++osc)
    for (int e = 0; e < 4; ++e) {
      bool hasColor = fw::kEngineColorName[e][0] != 0;
      std::vector<float> take[3];
      for (int t = 0; t < 3; ++t) {  // 0 = base, 1 = Macro moved, 2 = Color moved
        boot(48000);
        cleanTone(fw::WAVE_SQR);
        if (osc == 1) {
          setPot("mix_osc1", 0.0);
          setPot("mix_osc2", 1.0);
          setSel("osc2_wave", fw::WAVE_SQR);
          setPot("osc2_detune", 0.5);
        }
        setStep(osc ? fw::P_OSC2_ENGINE : fw::P_OSC1_ENGINE, e);
        double macro = t == 1 ? 0.8 : 0.3;
        if (osc == 0) setPot("osc1_macro", macro);
        else setCC(fw::P_OSC2_MACRO, (int)(macro * 127));
        setCC(osc ? fw::P_OSC2_COLOR : fw::P_OSC1_COLOR, t == 2 ? 110 : 20);
        renderNew(0.05);
        midi({0x90, 57, 110});
        take[t] = renderNew(0.5);
      }
      CHECK(rmsDiff(take[0], take[1]) > 0.003, "osc %d %s: Macro made no difference", osc + 1, kNames[e]);
      if (hasColor) CHECK(rmsDiff(take[0], take[2]) > 0.002, "osc %d %s: Color made no difference", osc + 1, kNames[e]);
      else CHECK(rmsDiff(take[0], take[2]) == 0, "osc %d %s: has no Color, yet it changed the sound", osc + 1, kNames[e]);
    }

  section("engines: all are in tune");
  struct Tune {
    int engine, wave;
    double macro;
    const char* what;
  };
  const Tune tunes[] = {
      {fw::ENG_FM, 0, 0.0, "FM, no modulation"},      {fw::ENG_FM, 1, 0.5, "FM 2:1"},
      {fw::ENG_WAVETABLE, 0, 0.0, "wavetable 1"},     {fw::ENG_WAVETABLE, 2, 0.7, "wavetable 3"},
      {fw::ENG_PLUCK, 3, 0.0, "pluck, bright"},       {fw::ENG_PLUCK, 3, 0.6, "pluck, damped"},
      {fw::ENG_PLUCK, 0, 0.3, "pluck, noise exciter"},
      {fw::ENG_FM, 4, 0.5, "FM 4:1"},                 {fw::ENG_WAVETABLE, 4, 0.5, "wavetable 5"},
      {fw::ENG_PLUCK, 4, 0.3, "pluck, pick exciter"},
  };
  for (double sr : {48000.0, 44100.0})
    for (const Tune& t : tunes)
      for (int note : {45, 69}) {
        boot(sr);
        engineTone(t.engine, t.wave);
        setPot("osc1_macro", t.macro);
        setCC(fw::P_OSC1_COLOR, 127);  // long ring for the pluck, so there is something to measure
        renderNew(0.05);
        midi({0x90, note, 110});
        std::vector<float> x = renderNew(1.3);
        double want = 440.0 * std::pow(2.0, (note - 69) / 12.0);
        double got = aliasCheck(x).f0;
        CHECK(std::fabs(got / want - 1) < 0.004, "%s, note %d @%.0f: %.2f Hz, expected %.2f", t.what, note, sr, got, want);
      }

  section("engines: high notes stay clean (nothing louder than -60 dB between the harmonics)");
  for (double sr : {48000.0, 44100.0}) {
    double worstFm = -300, worstWt = -300;
    for (int note : {84, 96, 103}) {
      for (int w : {0, 1, 2, 4})  // the whole-number ratios; 3.5:1 is deliberately not harmonic
        for (double macro : {0.5, 1.0, 0.75}) {
          boot(sr);
          engineTone(fw::ENG_FM, w);
          setPot("osc1_macro", macro);
          setCC(fw::P_OSC1_COLOR, macro == 0.75 ? 0 : 100);  // with and without feedback
          renderNew(0.05);
          midi({0x90, note, 127});
          AliasResult a = aliasCheck(renderNew(1.3), 440.0 * std::pow(2.0, (note - 69) / 12.0));
          worstFm = std::max(worstFm, a.worstDb);
          CHECK(a.worstDb <= -60.0, "FM ratio %d, amount %.0f%%, note %d @%.0f: %.1f dB at %.0f Hz", w, macro * 100, note, sr, a.worstDb, a.worstHz);
        }
      for (int w = 0; w < 5; ++w)
        for (double macro : {0.0, 0.45, 1.0}) {
          boot(sr);
          engineTone(fw::ENG_WAVETABLE, w);
          setPot("osc1_macro", macro);
          renderNew(0.05);
          midi({0x90, note, 127});
          AliasResult a = aliasCheck(renderNew(1.3), 440.0 * std::pow(2.0, (note - 69) / 12.0));
          worstWt = std::max(worstWt, a.worstDb);
          CHECK(a.worstDb <= -60.0, "wavetable %d, position %.0f%%, note %d @%.0f: %.1f dB at %.0f Hz", w + 1, macro * 100, note, sr, a.worstDb, a.worstHz);
        }
    }
    std::printf("    @%.1fk worst case: FM %.1f dB, wavetable %.1f dB\n", sr / 1000, worstFm, worstWt);
  }

  section("engines: a plucked string dies away on its own, slower with more Decay");
  double left[2];
  for (int c = 0; c < 2; ++c) {
    boot(48000);
    engineTone(fw::ENG_PLUCK, 3);
    setCC(fw::P_OSC1_COLOR, c ? 127 : 0);
    renderNew(0.05);
    midi({0x90, 57, 110});  // key held throughout
    std::vector<float> x = renderNew(3.0);
    left[c] = rmsOf(x, 2.5, 3.0) / rmsOf(x, 0.05, 0.25);
  }
  CHECK(left[0] < 0.2, "short decay: %.3f of the starting level left after 2.5 s", left[0]);
  CHECK(left[1] > 3 * left[0], "long decay %.3f should ring well past short decay %.3f", left[1], left[0]);
}

// AC4, extended: every soft parameter changes the sound too.
static void softBaseline() {
  phraseBaseline();
  setSel("voice_mode", fw::MODE_UNI);       // so unison detune matters
  setSel("lfo_destination", fw::LFO_PITCH);
  setStep(fw::P_OSC1_ENGINE, fw::ENG_FM);   // FM has a Color
  setStep(fw::P_OSC2_ENGINE, fw::ENG_PLUCK);
  setCC(fw::P_OSC2_MACRO, 40);
  setCC(fw::P_OSC1_COLOR, 40);
  setCC(fw::P_OSC2_COLOR, 40);
  setCC(fw::P_DLY_MIX, 70);
  setCC(fw::P_CHORUS, 70);
  setCC(fw::P_RVB_MIX, 70);
  setCC(fw::P_LFO_FADE, 60);
  setCC(fw::P_VEL_CUTOFF, 60);
  setStep(fw::P_FENV_PITCH, 5);
  setStep(fw::P_ARP_MODE, fw::ARP_UP);
  midi({0xE0, 0x00, 0x60});                 // some pitch bend, so bend range matters
}

static void testEverySoftParameter() {
  section("AC4  each of the soft parameters changes the sound");
  boot(48000);
  softBaseline();
  renderNew(0.4);
  std::vector<float> base = phrase();
  render(base, 0.4);
  for (int id = 0; id < fw::P_COUNT; ++id) {
    if (fw::isPanelParam(id)) continue;  // those are covered control by control above
    // the vocoder's other three do nothing without a voice at the input: they are tested with one, further down
    if (id == fw::P_VOC_FORMANT || id == fw::P_VOC_CONSONANTS || id == fw::P_VOC_SMOOTH) continue;
    boot(48000);
    softBaseline();
    int steps = fw::kParams[id].steps;
    // (the arp pattern goes from up to down: with only two notes held, several of the others coincide)
    int moved = id == fw::P_ARP_MODE ? fw::ARP_DOWN : (fw::gParams.step(id) + std::max(1, steps / 2)) % steps;
    fw::gParams.set(id, moved);
    renderNew(0.4);
    std::vector<float> x = phrase();
    render(x, 0.4);
    double d = rmsDiff(base, x);
    CHECK(d > 1e-4, "%s made no audible difference (%.6f)", fw::kParams[id].key, d);
  }
}

// Share of the sound's energy above `hz`, in dB.
static double shareAbove(const std::vector<float>& x, double hz) {
  const size_t n = 32768;
  std::vector<double> m = spectrum(x, (size_t)(0.25 * gSr), n);
  double all = 0, hi = 0;
  for (size_t i = 2; i < m.size(); ++i) {
    double p = m[i] * m[i];
    all += p;
    if ((double)i * gSr / (double)n > hz) hi += p;
  }
  return 10 * std::log10(hi / all + 1e-30);
}

static long countDiff(const std::vector<uint8_t>& a, int y0, int y1) {
  long n = 0;
  for (int y = y0; y < y1; ++y)
    for (int x = 0; x < fw::kDisplayW; ++x) {
      uint8_t b = a[(size_t)(y * fw::kDisplayW + x) >> 1];
      n += ((x & 1) ? (b & 0x0F) : (b >> 4)) != fw::gfxGet(x, y);
    }
  return n;
}

// Hands-on finding, 2026-11-11: Env amount, Keytrack and Drive all showed
// the same unchanging filter curve, and Drive was hard to hear.
static void testFilterControls() {
  section("filter: Env amount sweeps the cutoff, from the default patch");
  auto chord = [] {
    for (int k : {48, 55, 60, 63}) midi({0x90, k, 100});
    return renderNew(1.2);
  };
  double env[3];
  const double envSet[3] = {0.0, 0.5, 1.0};
  for (int i = 0; i < 3; ++i) {
    boot(48000);
    setPot("filter_env_amount", envSet[i]);
    renderNew(0.1);
    env[i] = shareAbove(chord(), 3000);
  }
  CHECK(env[2] > env[1] + 15 && env[0] < env[1] - 10, "energy above 3 kHz: fully down %.1f dB, centre %.1f dB, fully up %.1f dB", env[0], env[1], env[2]);

  section("filter: Keytrack opens the filter for high keys and closes it for low ones, pivoting on middle C");
  double kt[3][2];
  const int ktNotes[3] = {36, 60, 84};
  for (int n = 0; n < 3; ++n)
    for (int v = 0; v < 2; ++v) {
      boot(48000);
      setPot("filter_keytrack", v ? 1.0 : 0.0);
      renderNew(0.1);
      midi({0x90, ktNotes[n], 100});
      kt[n][v] = shareAbove(renderNew(1.2), 3000);
    }
  CHECK(kt[2][1] > kt[2][0] + 10, "C6: none %.1f dB, full %.1f dB above 3 kHz", kt[2][0], kt[2][1]);
  CHECK(kt[0][1] < kt[0][0] - 10, "C2: none %.1f dB, full %.1f dB above 3 kHz", kt[0][0], kt[0][1]);
  CHECK(std::fabs(kt[1][1] - kt[1][0]) < 1.5, "middle C should not change: %.1f dB against %.1f dB", kt[1][0], kt[1][1]);

  section("filter: Drive adds harmonics above the cutoff without a jump in loudness");
  double hi[3], level[3];
  const double driveSet[3] = {0.0, 0.5, 1.0};
  for (int i = 0; i < 3; ++i) {
    boot(48000);
    setPot("filter_drive", driveSet[i]);
    renderNew(0.1);
    std::vector<float> x = chord();
    hi[i] = shareAbove(x, 3000);
    level[i] = 20 * std::log10(rmsOf(x, 0.25, 1.2));
  }
  CHECK(hi[1] > hi[0] + 4 && hi[2] > hi[1] + 4, "energy above 3 kHz: off %.1f dB, half %.1f dB, full %.1f dB", hi[0], hi[1], hi[2]);
  CHECK(std::fabs(level[1] - level[0]) < 3 && std::fabs(level[2] - level[0]) < 3, "level: off %.1f dB, half %.1f dB, full %.1f dB", level[0], level[1], level[2]);
  std::printf("    drive off / half / full: above 3 kHz %.1f / %.1f / %.1f dB, level %.1f / %.1f / %.1f dB\n", hi[0], hi[1], hi[2], level[0], level[1], level[2]);

  section("filter: each of the three has its own picture, and the picture follows the knob");
  struct Shot {
    const char* control;
    double a, b;
    const char* name;
  };
  const Shot shots[] = {
      {"filter_env_amount", 0.5, 0.9, "popup_filter_env_amount"},
      {"filter_keytrack", 0.0, 1.0, "popup_filter_keytrack"},
      {"filter_drive", 0.25, 0.8, "popup_filter_drive"},
  };
  boot(48000);
  setPot("filter_cutoff", 0.55);
  fw::tick(33);
  std::vector<uint8_t> plain = grabFrame();  // the ordinary curve, as Cutoff shows it
  for (const Shot& s : shots) {
    setPot(s.control, s.a);
    fw::tick(33);
    std::vector<uint8_t> first = grabFrame();
    saveFrame((std::string(s.name) + "_a").c_str());
    setPot(s.control, s.b);
    fw::tick(33);
    saveFrame((std::string(s.name) + "_b").c_str());
    CHECK(countDiff(first, 50, 216) > 300, "%s: the picture barely changed between two settings (%ld pixels)", s.control, countDiff(first, 50, 216));
    CHECK(countDiff(plain, 50, 216) > 300, "%s: the picture is the plain filter curve", s.control);
  }
  // Env amount: an amber curve for the envelope's peak, none when there is no sweep
  setPot("filter_env_amount", 0.6);
  setPot("filter_env_amount", 0.9);
  fw::tick(33);
  CHECK(countColour(fw::kAccent, 50, 216) > 300, "Env amount up: no curve for the envelope's peak");
  setPot("filter_env_amount", 0.5);
  fw::tick(33);
  CHECK(countColour(fw::kAccent, 50, 216) == 0, "Env amount at centre: there should be no sweep drawn");
  // with the cutoff wide open there is nowhere to sweep to, and the display says so
  setPot("filter_cutoff", 1.0);
  setPot("filter_env_amount", 0.9);
  fw::tick(33);
  saveFrame("popup_filter_env_amount_open");
  CHECK(countColour(fw::kGrey, 54, 72) > 200, "cutoff fully open: the display should say the sweep has nowhere to go");
  setPot("filter_cutoff", 0.55);
  // Keytrack: the low and high keys' curves separate from the middle one
  setPot("filter_keytrack", 0.0);
  fw::tick(33);
  long together = countColour(fw::kLineDim, 74, 216);
  setPot("filter_keytrack", 1.0);
  fw::tick(33);
  CHECK(together == 0 && countColour(fw::kLineDim, 74, 216) > 300, "Keytrack: dim curves %ld at none, %ld at full", together, countColour(fw::kLineDim, 74, 216));
  // Drive: the output wave pulls away from the input wave
  setPot("filter_drive", 0.3);
  setPot("filter_drive", 0.0);
  fw::tick(33);
  long hidden = countColour(fw::kGrey, 60, 200);
  setPot("filter_drive", 1.0);
  fw::tick(33);
  CHECK(hidden < 40 && countColour(fw::kGrey, 60, 200) > 200, "Drive: input wave showing %ld px at none, %ld at full", hidden, countColour(fw::kGrey, 60, 200));
}

// Loudest content below `hz`, relative to the loudest content anywhere, in dB.
// A clean high note has nothing below its fundamental; false tones show up there.
static double worstBelow(const std::vector<float>& x, double hz) {
  const size_t n = 32768;
  std::vector<double> m = spectrum(x, (size_t)(0.4 * gSr), n);
  double top = 0, low = 0;
  for (size_t i = 8; i < m.size(); ++i) {
    top = std::max(top, m[i]);
    if ((double)i * gSr / (double)n < hz) low = std::max(low, m[i]);
  }
  return 20 * std::log10(low / top + 1e-300);
}

static void testPanelRegistry() {
  section("panel: every control maps to its own parameter, and old control numbers are unchanged");
  CHECK(fw::kFirstEncoder == 44 && fw::kNumControls == 57, "%d panel controls, %d controls in all", fw::kFirstEncoder, fw::kNumControls);
  for (int c = 0; c < 32; ++c) CHECK(fw::controlParam(c) == c, "control %d moved to parameter %d", c, fw::controlParam(c));
  for (int a = 0; a < fw::kNumControls; ++a)
    for (int b = a + 1; b < fw::kNumControls; ++b) {
      CHECK(std::strcmp(fw::controlInfo(a).name, fw::controlInfo(b).name) != 0, "two controls are both called %s", fw::controlInfo(a).name);
      if (a < fw::kFirstEncoder && b < fw::kFirstEncoder)
        CHECK(fw::controlParam(a) != fw::controlParam(b), "controls %d and %d set the same parameter", a, b);
    }
  static const char* const kExpected[] = {"arp_mode", "arp_octaves", "arp_tempo", "arp_gate", "voice_drift", "voice_spread", "osc_xmod", "osc_sync"};
  for (const char* name : kExpected) CHECK(ctl(name) >= 32 && ctl(name) < fw::kFirstEncoder, "%s should be a panel control", name);
  CHECK(fw::controlInfo(ctl("arp_mode")).kind == fw::CK_SELECTOR && fw::controlInfo(ctl("arp_mode")).steps == 6, "arp Mode: a six-stop selector");
  CHECK(fw::controlInfo(ctl("arp_octaves")).kind == fw::CK_SELECTOR && fw::controlInfo(ctl("arp_octaves")).steps == 4, "arp Octaves: a four-stop selector");
  CHECK(fw::controlInfo(ctl("arp_tempo")).kind == fw::CK_POT && fw::controlInfo(ctl("arp_gate")).kind == fw::CK_POT, "arp Tempo and Gate: knobs");
  CHECK(fw::controlInfo(ctl("osc_sync")).kind == fw::CK_SELECTOR && fw::controlInfo(ctl("osc_sync")).steps == 2, "Sync: a two-position switch");
  CHECK(fw::controlInfo(ctl("osc1_wave")).steps == 5 && fw::controlInfo(ctl("osc2_wave")).steps == 5, "Wave selectors: five stops");
}

static void testDriftAndSpread() {
  section("Drift: at zero every voice is exactly in tune; turned up, pitch wanders slowly and voices disagree");
  double wander[2], apart[2];
  for (int d = 0; d < 2; ++d) {
    boot(48000);
    cleanTone(fw::WAVE_SIN);
    setPot("voice_drift", d ? 1.0 : 0.0);
    renderNew(0.05);
    midi({0x90, 69, 100});
    std::vector<float> x = renderNew(6.0);
    // pitch measured second by second
    double lo = 1e9, hi = 0;
    for (int s = 0; s < 6; ++s) {
      double f = measureFreq(x, (size_t)(s * gSr), (size_t)((s + 1) * gSr));
      lo = std::min(lo, f);
      hi = std::max(hi, f);
    }
    wander[d] = 1200 * std::log2(hi / lo);  // cents
    midi({0x80, 69, 0});
    renderNew(1.0);
    // the same note on two different voices, one after the other: do they agree?
    midi({0x90, 60, 100});
    renderNew(0.2);
    midi({0x90, 69, 100});
    x = renderNew(1.0);
    midi({0x80, 60, 0});
    renderNew(1.0);
    x = renderNew(1.0);
    double f2 = measureFreq(x, 0, x.size());
    apart[d] = std::fabs(1200 * std::log2(f2 / 440.0));
  }
  CHECK(wander[0] < 0.05 && apart[0] < 0.05, "drift off: wanders %.3f cents, voice off by %.3f cents", wander[0], apart[0]);
  CHECK(wander[1] > 1.0 && wander[1] < 40.0, "drift full: pitch wandered %.2f cents over six seconds", wander[1]);
  std::printf("    drift full: one voice wandered %.1f cents in 6 s; a second voice sat %.1f cents from true\n", wander[1], apart[1]);

  section("Spread: voices fan out between the speakers; a mono voice stays in the middle");
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  midi({0x90, 57, 100});
  std::vector<float> l = renderNew(0.5);
  double centre = rmsOf(l, 0.1, 0.5);
  CHECK(rmsDiff(l, gRight) == 0, "spread off: left and right should be identical");
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  setPot("voice_spread", 1.0);
  renderNew(0.05);
  midi({0x90, 57, 100});
  l = renderNew(0.5);
  std::vector<float> r = gRight;
  CHECK(std::fabs(rmsOf(l, 0.1, 0.5) / centre - 1) < 0.02 && rmsOf(r, 0.1, 0.5) < 1e-5, "full spread, first voice: left %.4f (was %.4f), right %.6f", rmsOf(l, 0.1, 0.5), centre, rmsOf(r, 0.1, 0.5));
  midi({0x90, 64, 100});
  l = renderNew(0.5);
  r = gRight;
  CHECK(rmsOf(r, 0.1, 0.5) > 0.5 * centre, "full spread, second voice should be on the right (%.4f)", rmsOf(r, 0.1, 0.5));
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  setPot("voice_spread", 1.0);
  setSel("voice_mode", fw::MODE_MONO);
  renderNew(0.05);
  midi({0x90, 57, 100});
  l = renderNew(0.5);
  CHECK(rmsDiff(l, gRight) == 0, "mono mode: the one voice should stay in the middle");
  fw::tick(33);
  setPot("voice_spread", 0.7);
  fw::tick(33);
  saveFrame("popup_voice_spread");
}

static void testSupersawSyncXmod() {
  section("Supersaw: a fifth Wave stop; Macro spreads its seven saws; clean on high notes");
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  midi({0x90, 57, 110});
  std::vector<float> saw = renderNew(1.5);
  double flutter[2];
  for (int m = 0; m < 2; ++m) {
    boot(48000);
    cleanTone(fw::WAVE_SUPER);
    setPot("osc1_macro", m ? 0.8 : 0.0);
    fw::tick(33);
    if (m) saveFrame("popup_supersaw_detune");
    renderNew(0.05);
    midi({0x90, 57, 110});
    std::vector<float> x = renderNew(1.5);
    CHECK(rmsDiff(x, saw) > 0.01, "supersaw should not sound like a single saw");
    double pk = peakOf(x);
    CHECK(pk > 0.05 && pk < 0.6, "supersaw peak %.3f", pk);
    // how much the loudness heaves as the saws beat against each other
    double lo = 1e9, hi = 0;
    for (int k = 4; k < 28; ++k) {
      double e = rmsOf(x, k * 0.05, k * 0.05 + 0.05);
      lo = std::min(lo, e);
      hi = std::max(hi, e);
    }
    flutter[m] = hi / lo;
    // (with the saws close together; spread wide, the lowest of the seven is the lowest thing sounding)
    if (!m) CHECK(std::fabs(aliasCheck(x).f0 / 220.0 - 1) < 0.01, "supersaw should be centred on the note (%.1f Hz)", aliasCheck(x).f0);
  }
  CHECK(flutter[0] > 1.02, "even at minimum the saws should beat a little (%.3f)", flutter[0]);
  double worst = -300;
  for (double sr : {48000.0, 44100.0})
    for (int note : {96, 108})
      for (double macro : {0.0, 1.0}) {
        boot(sr);
        cleanTone(fw::WAVE_SUPER);
        setPot("osc1_macro", macro);
        renderNew(0.05);
        midi({0x90, note, 127});
        double f0 = 440.0 * std::pow(2.0, (note - 69) / 12.0);
        double below = worstBelow(renderNew(1.3), f0 * 0.85);
        worst = std::max(worst, below);
        CHECK(below <= -60.0, "supersaw note %d, detune %.0f%% @%.0f: %.1f dB below the note", note, macro * 100, sr, below);
      }
  std::printf("    supersaw on high notes: worst %.1f dB below the fundamental\n", worst);

  section("Sync: oscillator 2 restarts with oscillator 1, so the pitch stays oscillator 1's");
  double worstSync = -300;
  for (int w : {fw::WAVE_SAW, fw::WAVE_SQR}) {
    std::vector<float> take[2];
    for (int on = 0; on < 2; ++on) {
      boot(48000);
      cleanTone(fw::WAVE_SAW);
      setPot("mix_osc1", 0.0);       // listen to oscillator 2 alone
      setPot("mix_osc2", 1.0);
      setSel("osc2_wave", w);
      setSel("osc2_octave", 3);      // an octave up...
      setPot("osc2_detune", 0.9);    // ...and well out of tune: unsynced, this is another note entirely
      setSel("osc_sync", on);
      renderNew(0.05);
      midi({0x90, 57, 110});
      take[on] = renderNew(1.3);
    }
    CHECK(rmsDiff(take[0], take[1]) > 0.01, "sync made no difference");
    AliasResult off = aliasCheck(take[0]), on = aliasCheck(take[1], 220.0);
    CHECK(off.f0 > 400.0, "unsynced, oscillator 2 should sound an octave up and sharp (%.1f Hz)", off.f0);
    CHECK(on.worstDb <= -40.0, "synced: %.1f dB between the harmonics of oscillator 1's note, at %.0f Hz", on.worstDb, on.worstHz);
    worstSync = std::max(worstSync, on.worstDb);
  }
  std::printf("    sync, A3 with oscillator 2 an octave up: worst %.1f dB between harmonics\n", worstSync);

  // the sweep that sync is known for: Env>Pitch moves oscillator 2 only
  double f1[2];
  for (int on = 0; on < 2; ++on) {
    boot(48000);
    cleanTone(fw::WAVE_SIN);  // oscillator 1 alone, a sine: easy to measure
    setPot("fenv_attack", 0.0);
    setPot("fenv_decay", 0.8);
    setPot("fenv_sustain", 1.0);
    setStep(fw::P_FENV_PITCH, 12);
    setSel("osc_sync", on);
    renderNew(0.05);
    midi({0x90, 69, 100});
    std::vector<float> x = renderNew(0.6);
    f1[on] = measureFreq(x, 9600, x.size());
  }
  CHECK(f1[0] > 800 && std::fabs(f1[1] / 440.0 - 1) < 0.002, "Env>Pitch should move oscillator 1 only with Sync off: off %.1f Hz, on %.1f Hz", f1[0], f1[1]);

  section("X-Mod: oscillator 2 bends oscillator 1 even when oscillator 2 itself is turned down");
  std::vector<float> xm[2];
  for (int on = 0; on < 2; ++on) {
    boot(48000);
    cleanTone(fw::WAVE_SIN);  // oscillator 2's fader is at zero
    setSel("osc2_octave", 3);
    setPot("osc_xmod", on ? 0.8 : 0.0);
    renderNew(0.05);
    midi({0x90, 57, 110});
    xm[on] = renderNew(0.8);
    bool finite = true;
    for (float v : xm[on]) finite = finite && std::isfinite(v);
    CHECK(finite && peakOf(xm[on]) < 0.6, "x-mod output peak %.3f", peakOf(xm[on]));
  }
  CHECK(rmsDiff(xm[0], xm[1]) > 0.01, "x-mod made no difference (%.5f)", rmsDiff(xm[0], xm[1]));
  CHECK(shareAbove(xm[1], 1000) > shareAbove(xm[0], 1000) + 20, "x-mod should add partials above the note: %.1f dB against %.1f dB", shareAbove(xm[1], 1000), shareAbove(xm[0], 1000));
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  setPot("osc_xmod", 1.0);
  renderNew(0.05);
  midi({0x90, 115, 127});  // very high: the modulation is held back so nothing folds over
  double high = worstBelow(renderNew(1.3), 440.0 * std::pow(2.0, (115 - 69) / 12.0) * 0.9);
  CHECK(high <= -60.0, "x-mod on a very high note: %.1f dB below the note", high);

  section("arp: Tempo is a panel knob now, from 40 to 240 BPM");
  boot(48000);
  arpTone();
  setSel("arp_mode", fw::ARP_UP);
  setPot("arp_tempo", 1.0);  // 240 BPM: sixteen notes a second
  renderNew(0.05);
  for (int k : {60, 64, 67}) midi({0x90, k, 100});
  std::vector<int> fast = stepNotes(renderNew(0.5), 0.0625, 8);
  CHECK((fast == std::vector<int>{60, 64, 67, 60, 64, 67, 60, 64}), "240 BPM: got %s", show(fast).c_str());
  setPot("arp_tempo", 0.0);
  fw::tick(33);
  saveFrame("popup_arp_tempo");
  CHECK(fw::gUi.popupParam() == fw::P_ARP_TEMPO, "moving Tempo should show its value");
}

static void testVersion2State() {
  section("AC11  state saved before the arp moved to the panel still loads, tempo and gate included");
  const int oldCount = 58;
  uint8_t blob[6 + oldCount * 2 + 2] = {'S', 'Y', 'N', 'T', 2, (uint8_t)oldCount};
  uint16_t sum = 0;
  for (int i = 0; i < oldCount; ++i) {
    uint16_t v = fw::kParams[i].def;
    if (i == fw::P_ARP_TEMPO) v = 160;  // old form: steps above 40 BPM, so 200 BPM
    if (i == fw::P_ARP_GATE) v = 19;    // old form: the last of 20 steps, 100%
    if (i == fw::P_CUTOFF) v = 1234;
    blob[6 + i * 2] = (uint8_t)(v & 0xFF);
    blob[7 + i * 2] = (uint8_t)(v >> 8);
    sum = (uint16_t)(sum * 31u + v + 1u);
  }
  blob[6 + oldCount * 2] = (uint8_t)(sum & 0xFF);
  blob[7 + oldCount * 2] = (uint8_t)(sum >> 8);
  std::memcpy(gStore, blob, sizeof blob);
  gStoreLen = sizeof blob;
  boot(48000, true);
  CHECK(fw::gParams.val[fw::P_CUTOFF] == 1234, "an unchanged parameter should load as it was");
  CHECK(fw::mapTempoBpm(fw::gParams.norm(fw::P_ARP_TEMPO)) == 200.0f, "tempo: %.0f BPM, expected 200", fw::mapTempoBpm(fw::gParams.norm(fw::P_ARP_TEMPO)));
  CHECK(fw::mapArpGate(fw::gParams.norm(fw::P_ARP_GATE)) > 0.999f, "gate: %.3f, expected 1", fw::mapArpGate(fw::gParams.norm(fw::P_ARP_GATE)));
  CHECK(fw::gParams.val[fw::P_DRIFT] == 0 && fw::gParams.step(fw::P_SYNC) == 0, "controls that did not exist yet should be at their defaults");
}

// Frequency of the strongest partial.
static double peakHz(const std::vector<float>& x) {
  const size_t n = 32768;
  std::vector<double> m = spectrum(x, (size_t)(0.3 * gSr), n);
  size_t pk = 8;
  for (size_t i = 8; i < m.size(); ++i)
    if (m[i] > m[pk]) pk = i;
  double a = m[pk - 1], b = m[pk], c = m[pk + 1];
  return ((double)pk + 0.5 * (a - c) / (a - 2 * b + c)) * gSr / (double)n;
}

static void testLadderFilter() {
  section("ladder filter: a third slope on the Engine page, with its own sound");
  CHECK(fw::kParams[fw::P_FILT_SLOPE].steps == 3, "filter slope should have three settings");
  std::vector<float> take[3];
  for (int slope = 0; slope < 3; ++slope) {
    boot(48000);
    setPot("filter_cutoff", 0.5);
    setPot("filter_resonance", 0.7);
    setStep(fw::P_FILT_SLOPE, slope);
    renderNew(0.1);
    for (int k : {48, 55, 60}) midi({0x90, k, 100});
    take[slope] = renderNew(1.0);
  }
  CHECK(rmsDiff(take[2], take[1]) > 0.005 && rmsDiff(take[2], take[0]) > 0.005, "ladder should not sound like the 12 or 24 dB filter");

  section("ladder filter: the resonant peak sits at the cutoff, at either sample rate");
  for (double sr : {48000.0, 44100.0})
    for (double cut : {0.45, 0.6, 0.75}) {
      boot(sr);
      cleanTone(fw::WAVE_SAW);
      setPot("mix_osc1", 0.0);
      setPot("mix_noise", 0.15);         // quiet noise in: the filter's own peak is what comes out
      setPot("filter_cutoff", cut);
      setPot("filter_resonance", 0.97);
      setStep(fw::P_FILT_SLOPE, fw::SLOPE_LADDER);
      renderNew(0.1);
      midi({0x90, 60, 100});
      double want = 20.0 * std::pow(2.0, cut * 10.0);
      // (averaged over several stretches: one stretch of noise has random peaks of its own)
      double got = 0;
      for (int rep = 0; rep < 6; ++rep) got += peakHz(renderNew(1.2)) / 6.0;
      CHECK(std::fabs(got / want - 1) < 0.05, "cutoff %.0f Hz @%.0f: peak at %.0f Hz", want, sr, got);
    }

  section("ladder filter: falls away at 24 dB per octave, and stays in bounds whatever is thrown at it");
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  setPot("mix_osc1", 0.0);
  setPot("mix_noise", 1.0);
  setPot("filter_cutoff", 0.5);          // 640 Hz
  setStep(fw::P_FILT_SLOPE, fw::SLOPE_LADDER);
  renderNew(0.1);
  midi({0x90, 60, 100});
  std::vector<float> x = renderNew(1.5);
  std::vector<double> m = spectrum(x, (size_t)(0.3 * gSr), 32768);
  auto band = [&](double lo, double hi) {
    double e = 0;
    int n = 0;
    for (size_t i = 0; i < m.size(); ++i) {
      double f = (double)i * gSr / 32768.0;
      if (f >= lo && f < hi) {
        e += m[i] * m[i];
        ++n;
      }
    }
    return 10 * std::log10(e / n + 1e-300);
  };
  double pass = band(100, 300), twoUp = band(2400, 2700), threeUp = band(4800, 5400);
  CHECK(pass - twoUp > 38 && pass - twoUp < 56, "two octaves above the cutoff: %.1f dB down, expected about 46", pass - twoUp);
  CHECK(twoUp - threeUp > 18 && twoUp - threeUp < 30, "the next octave: a further %.1f dB, expected about 24", twoUp - threeUp);

  boot(48000);
  setStep(fw::P_FILT_SLOPE, fw::SLOPE_LADDER);
  setPot("filter_resonance", 1.0);
  setPot("filter_drive", 1.0);
  setPot("mix_noise", 1.0);
  setPot("mix_sub", 1.0);
  for (int i = 0; i < 8; ++i) midi({0x90, 30 + i * 9, 127});
  bool finite = true;
  double pk = 0;
  for (int sweep = 0; sweep <= 20; ++sweep) {
    setPot("filter_cutoff", sweep / 20.0);
    std::vector<float> y = renderNew(0.2);
    for (float v : y) finite = finite && std::isfinite(v);
    pk = std::max(pk, peakOf(y));
  }
  CHECK(finite && pk < 0.99, "full resonance, full drive, cutoff swept end to end: peak %.3f", pk);

  boot(48000);
  setStep(fw::P_FILT_SLOPE, fw::SLOPE_LADDER);
  setPot("filter_resonance", 0.8);
  fw::tick(33);
  saveFrame("popup_filter_ladder");
}

// Pitches of the strongest partials in a sound, as MIDI note numbers.
static std::vector<int> soundingNotes(const std::vector<float>& x, int howMany) {
  const size_t n = 32768;
  std::vector<double> m = spectrum(x, (size_t)(0.15 * gSr), n);
  std::vector<int> notes;
  for (int k = 0; k < howMany; ++k) {
    size_t pk = 8;
    for (size_t i = 8; i < m.size() - 1; ++i)
      if (m[i] > m[pk]) pk = i;
    notes.push_back((int)std::lround(69 + 12 * std::log2((double)pk * gSr / (double)n / 440.0)));
    for (size_t i = pk > 30 ? pk - 30 : 0; i < pk + 30 && i < m.size(); ++i) m[i] = 0;
  }
  std::sort(notes.begin(), notes.end());
  return notes;
}

static void testChordMemory() {
  section("chord memory: learn a held chord, then play it from one key");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  press("btn_chord");
  renderNew(0.05);
  CHECK(fw::controlGet(ctl("btn_chord")) == 0, "with nothing held there is nothing to learn: the light should stay off");
  for (int k : {60, 64, 67}) midi({0x90, k, 100});
  renderNew(0.1);
  press("btn_chord");
  renderNew(0.05);
  CHECK(fw::controlGet(ctl("btn_chord")) == 1, "Chord should light once it has learned a chord");
  CHECK(fw::gSynth.activeVoices() == 3, "the keys being held should carry on unchanged (%d voices)", fw::gSynth.activeVoices());
  for (int k : {60, 64, 67}) midi({0x80, k, 0});
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 0, "letting the chord go should silence it");

  midi({0x90, 62, 100});  // one key: D
  std::vector<float> x = renderNew(0.8);
  CHECK(fw::gSynth.activeVoices() == 3, "one key should sound three voices (%d)", fw::gSynth.activeVoices());
  std::vector<int> got = soundingNotes(x, 3);
  CHECK((got == std::vector<int>{62, 66, 69}), "D should play D, F sharp, A: got %s", show(got).c_str());
  fw::tick(1600);  // let the last control's visual clear
  fw::tick(33);
  saveFrame("home_chord");
  CHECK(countColour(fw::kAccent, 222, 236) > 100, "the display should show that chord memory is on");

  // two keys whose chords share notes: letting one go must not cut the other's notes
  midi({0x90, 66, 100});  // F sharp major: F#, A#, C#; shares F# with the D chord
  renderNew(0.3);
  CHECK(fw::gSynth.activeVoices() == 5, "D and F sharp chords together: %d voices, expected 5 (they share one note)", fw::gSynth.activeVoices());
  midi({0x80, 62, 0});
  x = renderNew(1.2);
  CHECK(fw::gSynth.activeVoices() == 3, "after releasing D: %d voices, expected the three of F sharp major", fw::gSynth.activeVoices());
  got = soundingNotes(renderNew(0.8), 3);
  CHECK((got == std::vector<int>{66, 70, 73}), "the F sharp chord should be intact: got %s", show(got).c_str());
  midi({0x80, 66, 0});
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 0, "all keys up: silence");

  section("chord memory: switching it off returns keys to single notes, even one being held");
  midi({0x90, 62, 100});
  renderNew(0.3);
  press("btn_chord");
  renderNew(1.0);
  CHECK(fw::controlGet(ctl("btn_chord")) == 0, "second press should switch Chord off");
  CHECK(fw::gSynth.activeVoices() == 1, "the held key should drop back to its own note (%d voices)", fw::gSynth.activeVoices());
  x = renderNew(0.5);
  CHECK(std::fabs(measureFreq(x, 0, x.size()) / 293.66 - 1) < 0.003, "and that note should be the key itself");
  midi({0x80, 62, 0});
  midi({0x90, 65, 100});
  renderNew(0.3);
  CHECK(fw::gSynth.activeVoices() <= 2, "with Chord off a key is one note again");
  midi({0x80, 65, 0});

  section("chord memory with the arp: one key arpeggiates the remembered chord");
  boot(48000);
  arpTone();
  for (int k : {60, 63, 67, 70}) midi({0x90, k, 100});  // C minor seventh
  renderNew(0.1);
  press("btn_chord");
  renderNew(0.05);
  for (int k : {60, 63, 67, 70}) midi({0x80, k, 0});
  renderNew(0.5);
  setSel("arp_mode", fw::ARP_UP);
  renderNew(0.05);
  midi({0x90, 57, 100});  // one key: A
  std::vector<int> seq = stepNotes(renderNew(1.0), 0.125, 8);
  CHECK((seq == std::vector<int>{57, 60, 64, 67, 57, 60, 64, 67}), "one key should arpeggiate A minor seventh: got %s", show(seq).c_str());
  midi({0x80, 57, 0});
  x = renderNew(0.6);
  CHECK(rmsOf(x, 0.3, 0.6) < 1e-4, "and stop when the key is let go");

  section("chord memory with Hold: the held chord is replaced by the next key's chord");
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  for (int k : {60, 64, 67}) midi({0x90, k, 100});
  renderNew(0.1);
  press("btn_chord");
  renderNew(0.05);
  for (int k : {60, 64, 67}) midi({0x80, k, 0});
  renderNew(1.0);
  press("btn_hold");
  midi({0x90, 62, 100});
  renderNew(0.2);
  midi({0x80, 62, 0});
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 3, "Hold should keep the chord sounding (%d voices)", fw::gSynth.activeVoices());
  midi({0x90, 65, 100});
  renderNew(0.2);
  midi({0x80, 65, 0});
  renderNew(1.2);
  got = soundingNotes(renderNew(0.8), 3);
  CHECK(fw::gSynth.activeVoices() == 3 && (got == std::vector<int>{65, 69, 72}), "the next key's chord should replace it: %d voices, %s", fw::gSynth.activeVoices(), show(got).c_str());
}

static void testLfoShapes() {
  section("LFO Shape is a five-stop selector, and each stop is a different shape");
  int i = ctl("lfo_shape");
  CHECK(fw::controlInfo(i).kind == fw::CK_SELECTOR && fw::controlInfo(i).steps == 5, "Shape should be a selector with 5 stops");
  std::vector<std::vector<float>> takes;
  for (int shape = 0; shape < 5; ++shape) {
    boot(48000);
    cleanTone(fw::WAVE_SAW);
    setPot("filter_cutoff", 0.5);
    setSel("lfo_destination", fw::LFO_CUTOFF);
    setPot("lfo_depth", 0.8);
    setPot("lfo_rate", 0.8);
    setSel("lfo_shape", shape);
    fw::tick(33);
    if (shape == fw::LFOS_SQUARE) saveFrame("popup_lfo_shape_selector");
    renderNew(0.05);
    midi({0x90, 57, 100});
    std::vector<float> x = renderNew(1.0);
    for (size_t k = 0; k < takes.size(); ++k)
      CHECK(rmsDiff(x, takes[k]) > 0.003, "%s sounds the same as %s", fw::kLfoShapeNames[shape], fw::kLfoShapeNames[k]);
    takes.push_back(x);
  }
}

static void testOlderSavedState() {
  section("AC11  state saved by the first build (36 parameters) still loads");
  const int oldCount = 36;
  uint8_t blob[6 + oldCount * 2 + 2] = {'S', 'Y', 'N', 'T', 1, (uint8_t)oldCount};
  uint16_t sum = 0;
  for (int i = 0; i < oldCount; ++i) {
    const fw::ParamDef& d = fw::kParams[i];
    uint16_t v = d.kind == fw::PK_STEP ? (uint16_t)(d.steps - 1) : (uint16_t)(900 + i * 7);
    if (i == fw::P_LFO_SHAPE) v = 2600;  // the first build stored Shape as a knob position: this one is in the SQUARE zone
    if (i == fw::P_SUB_OCTAVE) v = 1;    // and the sub as 0 = one octave down, 1 = two
    blob[6 + i * 2] = (uint8_t)(v & 0xFF);
    blob[7 + i * 2] = (uint8_t)(v >> 8);
    sum = (uint16_t)(sum * 31u + v + 1u);
  }
  blob[6 + oldCount * 2] = (uint8_t)(sum & 0xFF);
  blob[7 + oldCount * 2] = (uint8_t)(sum >> 8);
  std::memcpy(gStore, blob, sizeof blob);
  gStoreLen = sizeof blob;
  boot(48000, true);
  int wrong = 0;
  for (int i = 0; i < oldCount; ++i) {
    const fw::ParamDef& d = fw::kParams[i];
    uint16_t v = d.kind == fw::PK_STEP ? (uint16_t)(d.steps - 1) : (uint16_t)(900 + i * 7);
    if (i == fw::P_LFO_SHAPE || i == fw::P_SUB_OCTAVE) continue;
    wrong += fw::gParams.val[i] != v;
  }
  CHECK(wrong == 0, "%d of the old parameters came back different", wrong);
  CHECK(fw::gParams.step(fw::P_SUB_OCTAVE) == fw::SUB_2_OCT_DOWN, "an old 'two octaves down' sub should still be two octaves down (got %d)",
        fw::gParams.step(fw::P_SUB_OCTAVE));
  CHECK(fw::gParams.step(fw::P_LFO_SHAPE) == fw::LFOS_SQUARE, "an old Shape knob position should become the matching stop (got %d)",
        fw::gParams.step(fw::P_LFO_SHAPE));
  wrong = 0;
  for (int i = oldCount; i < fw::P_COUNT; ++i) wrong += fw::gParams.val[i] != fw::kParams[i].def;
  CHECK(wrong == 0, "%d of the newer parameters are not at their defaults", wrong);
}

// Hands-on finding, 2026-11-11: text at the font's natural size is 1 mm tall
// on the real display and was unreadable. Nothing may be drawn smaller than
// kMinTextScale, whatever a screen asks for.
static void testTextSize() {
  section("display text is never smaller than 2.1 mm (scale 2)");
  boot(48000);
  fw::gfxClear(fw::kBlack);
  fw::gfxText(10, 10, "H", fw::kInk, 1);
  long asked1 = countColour(fw::kInk);
  fw::gfxClear(fw::kBlack);
  fw::gfxText(10, 10, "H", fw::kInk, 2);
  long asked2 = countColour(fw::kInk);
  CHECK(asked1 == asked2 && asked2 == 17 * 4, "scale 1 drew %ld pixels, scale 2 drew %ld (an H is 17 dots)", asked1, asked2);
  CHECK(fw::gfxTextWidth("AB", 1) == fw::gfxTextWidth("AB", 2), "text width must follow the same rule");

  // plot lines are two pixels thick in both directions, so a vertical edge
  // is as visible as a horizontal one
  fw::gfxClear(fw::kBlack);
  fw::gfxPlotLine(50, 20, 50, 60, fw::kLine);
  long vertical = countColour(fw::kLine);
  fw::gfxClear(fw::kBlack);
  fw::gfxPlotLine(20, 50, 60, 50, fw::kLine);
  long horizontal = countColour(fw::kLine);
  CHECK(vertical == horizontal && vertical == 2 * 42, "vertical %ld px, horizontal %ld px", vertical, horizontal);
}

// The display buffer is 4 bits per pixel. These guard the bit-packing: a
// slip there would smear neighbouring pixels or lose odd columns.
static void testPackedFrame() {
  section("display buffer: 4 bits per pixel, every pixel independent");
  boot(48000);
  CHECK(fw::kFrameBytes == 38400, "frame is %d bytes, expected 38400", fw::kFrameBytes);
  fw::gfxClear(fw::kGrid);
  CHECK(countColour(fw::kGrid) == 320L * 240, "clear did not fill every pixel");
  // single pixels on even and odd columns must not disturb their neighbours
  fw::gfxPixel(10, 5, fw::kAccent);
  fw::gfxPixel(13, 5, fw::kLine);
  CHECK(fw::gfxGet(10, 5) == fw::kAccent && fw::gfxGet(11, 5) == fw::kGrid && fw::gfxGet(9, 5) == fw::kGrid, "even column");
  CHECK(fw::gfxGet(13, 5) == fw::kLine && fw::gfxGet(12, 5) == fw::kGrid && fw::gfxGet(14, 5) == fw::kGrid, "odd column");
  // fills with every mix of odd and even edges cover exactly their rectangle
  int wrong = 0;
  for (int x0 = 20; x0 < 24; ++x0)
    for (int w = 1; w <= 6; ++w) {
      fw::gfxClear(fw::kBlack);
      fw::gfxFill(x0, 30, w, 3, fw::kInk);
      for (int y = 28; y < 36; ++y)
        for (int x = 14; x < 36; ++x) {
          bool inside = x >= x0 && x < x0 + w && y >= 30 && y < 33;
          wrong += (fw::gfxGet(x, y) == fw::kInk) != inside;
        }
    }
  CHECK(wrong == 0, "%d pixels wrong across the odd/even fill cases", wrong);
  // drawing off every edge must not wrap round or write outside the buffer
  fw::gfxClear(fw::kBlack);
  fw::gfxFill(-5, -5, 10, 10, fw::kLine);
  fw::gfxFill(315, 235, 10, 10, fw::kLine);
  fw::gfxPixel(320, 0, fw::kAccent);
  fw::gfxPixel(-1, 0, fw::kAccent);
  CHECK(countColour(fw::kLine) == 50 && countColour(fw::kAccent) == 0, "clipping at the edges: %ld line-colour pixels, %ld accent", countColour(fw::kLine), countColour(fw::kAccent));
  // a row expands to the display's own colours
  uint16_t row[fw::kDisplayW];
  fw::gfxClear(fw::kBlack);
  fw::gfxPixel(7, 100, fw::kAccent);
  fw::frameRow565(100, row);
  CHECK(row[7] == fw::gfxPalette565()[fw::kAccent] && row[6] == fw::gfxPalette565()[fw::kBlack] && row[8] == 0, "row expansion");

  section("oscillator tables: stored as 16-bit, none clipped");
  int peakSaw = 0, peakTri = 0, peakWt = 0;
  for (int l = 0; l < fw::kSawLevels; ++l)
    for (int i = 0; i <= fw::kTabLen; ++i) peakSaw = std::max(peakSaw, std::abs((int)fw::gSaw[l][i]));
  for (int l = 0; l < fw::kTriLevels; ++l)
    for (int i = 0; i <= fw::kTabLen; ++i) peakTri = std::max(peakTri, std::abs((int)fw::gTri[l][i]));
  for (int t = 0; t < fw::kWtTables; ++t)
    for (int f = 0; f < fw::kWtFrames; ++f)
      for (int l = 0; l < fw::kWtLevels; ++l)
        for (int i = 0; i <= fw::kWtLen; ++i) peakWt = std::max(peakWt, std::abs((int)fw::gWt[t][f][l][i]));
  CHECK(peakSaw > 27000 && peakSaw < 32700, "saw tables peak at %d of 32767", peakSaw);
  CHECK(peakTri > 26000 && peakTri < 32700, "triangle tables peak at %d of 32767", peakTri);
  CHECK(peakWt > 31000 && peakWt < 32700, "wavetables peak at %d of 32767", peakWt);
}

static void testState() {
  section("AC11  every control and encoder value survives a restart");
  boot(48000);
  gSaveCount = 0;
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    fw::ControlInfo info = fw::controlInfo(c);
    fw::controlSet(c, info.kind == fw::CK_SELECTOR ? info.steps - 1 : 137 + c * 101);
  }
  press("btn_engine");
  fw::encoderTurn(ctl("enc1"), 9);
  fw::encoderTurn(ctl("enc3"), 4);
  fw::encoderTurn(ctl("enc4"), -1);
  uint16_t want[fw::P_COUNT];
  for (int i = 0; i < fw::P_COUNT; ++i) want[i] = fw::gParams.val[i];
  fw::tick(500);
  CHECK(gSaveCount == 0, "saved before the controls had been still for a second");
  fw::tick(600);
  CHECK(gSaveCount == 1, "expected one save, got %d", gSaveCount);

  boot(48000, true);
  int wrong = 0;
  for (int i = 0; i < fw::P_COUNT; ++i) wrong += fw::gParams.val[i] != want[i];
  CHECK(wrong == 0, "%d parameters came back different", wrong);

  gStore[10] ^= 0x55;  // a damaged save must be ignored, not half-applied
  boot(48000, true);
  wrong = 0;
  for (int i = 0; i < fw::P_COUNT; ++i) wrong += fw::gParams.val[i] != fw::kParams[i].def;
  CHECK(wrong == 0, "damaged save: %d parameters are not at their defaults", wrong);
}

// ---------------------------------------------------------------- patches

static int ledR(int c) { return (int)((fw::ledColour(c) >> 16) & 255); }
static int ledG(int c) { return (int)((fw::ledColour(c) >> 8) & 255); }
static int ledB(int c) { return (int)(fw::ledColour(c) & 255); }

// Stores the sound as it stands in slot `slot` (0-based) through the Patch page.
static void savePatchTo(int slot) {
  if (fw::gUi.mode() != fw::MB_PATCH || fw::gUi.page() != 0) {
    while (fw::gUi.mode() != fw::MB_NONE) press("btn_patch");
    press("btn_patch");
  }
  fw::encoderTurn(ctl("enc1"), -fw::kPatchSlots);
  fw::encoderTurn(ctl("enc1"), slot);
  press("enc2");
  if (fw::gPatch.slot() != slot) press("enc2");  // a used slot asks twice
}

static void testPatches() {
  section("patches: saving stores every value; loading brings them back while the controls stay put");
  boot(48000);
  gPatchSaves = 0;
  int cut = ctl("filter_cutoff"), res = ctl("filter_resonance"), wave = ctl("osc1_wave"), dest = ctl("lfo_destination");
  fw::tick(33);
  std::vector<uint8_t> homeInit = grabFrame();
  setPot("filter_cutoff", 0.30);
  setPot("filter_resonance", 0.70);
  setSel("osc1_wave", fw::WAVE_SIN);
  setSel("lfo_destination", fw::LFO_AMP);
  setStep(fw::P_DLY_MIX, 40);
  setStep(fw::P_OSC1_ENGINE, fw::ENG_FM);
  uint16_t stored[fw::P_COUNT];
  for (int i = 0; i < fw::P_COUNT; ++i) stored[i] = fw::gParams.val[i];
  CHECK(fw::gPatch.slot() < 0 && fw::gPatch.edited(), "before any save: no slot, and the sound counts as changed");

  press("btn_patch");
  CHECK(fw::gUi.mode() == fw::MB_PATCH && fw::controlGet(ctl("btn_patch")) == 1, "Patch button should open the Patch page and light");
  fw::encoderTurn(ctl("enc1"), 4);  // slot 5
  fw::tick(33);
  saveFrame("mode_patch_empty");
  press("enc2");
  CHECK(gPatchSaves == 1 && fw::gPatch.slotUsed(4), "an empty slot saves on one push (%d saves)", gPatchSaves);
  CHECK(fw::gPatch.slot() == 4 && !fw::gPatch.edited(), "after a save the sound is that slot's, unchanged");
  CHECK(gBankLen[4] == fw::kPatchSize, "stored patch is %u bytes, expected %u", gBankLen[4], fw::kPatchSize);

  // change the sound: three controls and a soft value
  setPot("filter_cutoff", 0.80);
  setPot("filter_resonance", 0.10);
  setSel("osc1_wave", fw::WAVE_SAW);
  setStep(fw::P_DLY_MIX, 5);
  setStep(fw::P_OSC1_ENGINE, fw::ENG_VA);
  setPot("volume", 0.33);
  CHECK(fw::gPatch.edited(), "changing a control should mark the patch as changed");
  int physCut = fw::controlGet(cut), physRes = fw::controlGet(res);

  press("enc1");  // load the slot showing
  int wrong = 0;
  for (int i = 0; i < fw::P_COUNT; ++i)
    if (i != fw::P_VOLUME && i != fw::P_HOLD) wrong += fw::gParams.val[i] != stored[i];
  CHECK(wrong == 0, "%d parameters differ from the saved patch after loading", wrong);
  CHECK(fw::gParams.val[fw::P_VOLUME] == std::lround(0.33 * 4095), "master volume is not part of a patch");
  CHECK(fw::controlGet(cut) == physCut && fw::controlGet(res) == physRes && fw::controlGet(wave) == fw::WAVE_SAW,
        "loading must not move the physical controls");
  CHECK(fw::gPatch.following(cut) && fw::gPatch.following(res) && fw::gPatch.following(wave), "the three moved controls should be following the patch");
  CHECK(!fw::gPatch.following(dest) && !fw::gPatch.offPatch(dest), "a control already at the patch's value stays live");
  CHECK(fw::gPatch.followCount() == 3 && fw::gPatch.offCount() == 3, "expected 3 controls following, got %d (%d off)",
        fw::gPatch.followCount(), fw::gPatch.offCount());
  CHECK(!fw::gPatch.edited(), "a freshly loaded patch is unchanged");
  fw::tick(33);
  saveFrame("mode_patch_loaded");
  CHECK(countColour(fw::kLine, 130, 150) > 200, "the Patch page should say how many controls follow the patch");

  section("patches: the label lights show which controls are off the patch, and which way to go");
  CHECK(fw::ledColour(dest) == fw::kLedIdle, "a control at the patch's value has a plain label light");
  CHECK(fw::ledColour(ctl("enc1")) == 0 && fw::ledColour(ctl("btn_hold")) == 0, "encoders and buttons have no label light");
  CHECK(fw::gPatch.direction(cut) == -1 && fw::gPatch.direction(res) == 1 && fw::gPatch.direction(wave) == 1, "directions: %d %d %d",
        fw::gPatch.direction(cut), fw::gPatch.direction(res), fw::gPatch.direction(wave));
  // Resonance is far below the patch and Cutoff far above it.
  uint32_t upFar = fw::ledColour(res), downFar = fw::ledColour(cut);
  CHECK(upFar == fw::kLedUpFar, "a long way below the patch the light should be deep red (got %06x)", upFar);
  CHECK(downFar == fw::kLedDownFar, "a long way above the patch the light should be deep cold blue (got %06x)", downFar);
  bool steady = true;
  for (int t = 0; t < 90; ++t) {  // three seconds: the lights must not flash
    fw::tick(33);
    steady = steady && fw::ledColour(res) == upFar && fw::ledColour(cut) == downFar;
  }
  CHECK(steady, "the guide lights should hold steady, not blink");

  section("patches: a following pot takes over only on passing through the stored value");
  fw::tick(2000);
  fw::controlSet(cut, physCut - 300);  // towards the patch, but not there
  CHECK(fw::gParams.val[fw::P_CUTOFF] == stored[fw::P_CUTOFF] && fw::gPatch.following(cut), "moving without reaching the value must not change the sound");
  CHECK(fw::gUi.popupParam() == fw::P_CUTOFF, "touching a following control should bring up its pick-up picture");
  fw::tick(33);
  saveFrame("popup_pickup");
  std::vector<uint8_t> pick = grabFrame();
  CHECK(countColour(fw::kInk, 100, 130) > 80 && countColour(fw::kAccent, 90, 135) > 150, "pick-up picture should mark the panel and the patch positions");
  fw::controlSet(cut, 4095);  // away from the patch
  CHECK(fw::gParams.val[fw::P_CUTOFF] == stored[fw::P_CUTOFF], "moving away must not change the sound either");
  fw::controlSet(cut, stored[fw::P_CUTOFF] - 150);  // through the stored value
  CHECK(!fw::gPatch.following(cut) && fw::gParams.val[fw::P_CUTOFF] == stored[fw::P_CUTOFF] - 150, "passing through the stored value should hand over to the control");
  CHECK(fw::ledColour(cut) == fw::kLedIdle, "a control that has taken over has a plain label light again");
  fw::tick(33);
  CHECK(grabFrame() != pick, "once live, the control's own picture should replace the pick-up picture");
  fw::controlSet(cut, 2222);
  CHECK(fw::gParams.val[fw::P_CUTOFF] == 2222, "a live control sets the value directly");

  section("patches: landing within the tolerance also takes over; the light warms from red to amber on the way");
  int greenFar = ledG(res);
  fw::controlSet(res, stored[fw::P_RESONANCE] - 1000);
  int greenMid = ledG(res);
  fw::controlSet(res, stored[fw::P_RESONANCE] - 400);
  CHECK(fw::gPatch.following(res), "400 short of the value is still following");
  int greenNear = ledG(res);
  CHECK(greenFar < greenMid && greenMid < greenNear && ledR(res) > 200 && ledB(res) < 20,
        "coming up, the light should run from red towards amber (green %d, %d, %d)", greenFar, greenMid, greenNear);
  CHECK(fw::gPatch.distance(res) > 0 && fw::gPatch.distance(res) < 80, "400 short should count as near (%d)", fw::gPatch.distance(res));
  {  // and coming down, from deep blue to a pale cold blue
    setPot("mix_noise", 0.95);
    setCC(fw::P_MIX_NOISE, 10);  // the value is now far below the fader
    int noise = ctl("mix_noise");
    int want = fw::gParams.val[fw::P_MIX_NOISE];
    uint32_t deep = fw::ledColour(noise);
    fw::controlSet(noise, want + 300);
    uint32_t pale = fw::ledColour(noise);
    CHECK(deep == fw::kLedDownFar, "far above: deep blue (got %06x)", deep);
    CHECK((int)(pale & 255) > 200 && (int)((pale >> 16) & 255) > (int)((deep >> 16) & 255) + 60 && (int)(pale & 255) > (int)((pale >> 16) & 255),
          "nearly down to the value the blue should pale but stay blue (got %06x)", pale);
    fw::controlSet(noise, want);
    CHECK(fw::ledColour(noise) == fw::kLedIdle, "at the value the light goes plain");
  }
  fw::controlSet(res, stored[fw::P_RESONANCE] - 90);  // inside 2.5% of the travel, without crossing
  CHECK(!fw::gPatch.following(res) && fw::gParams.val[fw::P_RESONANCE] == stored[fw::P_RESONANCE] - 90, "within the tolerance the control should take over");
  // pitch is tighter: 1%
  int det = ctl("osc2_detune");
  setCC(fw::P_OSC2_DETUNE, 100);
  int detTarget = fw::gParams.val[fw::P_OSC2_DETUNE];
  fw::controlSet(det, detTarget - 80);
  CHECK(fw::gPatch.following(det), "80 short is inside the usual tolerance but not the tighter one for pitch");
  fw::controlSet(det, detTarget - 30);
  CHECK(!fw::gPatch.following(det), "30 short is inside the pitch tolerance");

  section("patches: a selector goes live on any change; its light stays on until it is on the patch's stop");
  setSel("osc1_wave", fw::WAVE_SQR);
  CHECK(fw::gParams.step(fw::P_OSC1_WAVE) == fw::WAVE_SQR && !fw::gPatch.following(wave), "one click and the selector is live");
  CHECK(fw::gPatch.offPatch(wave) && fw::ledColour(wave) != fw::kLedIdle && ledR(wave) > ledB(wave), "its light should still point up towards the patch's stop");
  setSel("osc1_wave", fw::WAVE_SIN);
  CHECK(!fw::gPatch.offPatch(wave) && fw::ledColour(wave) == fw::kLedIdle, "on the patch's stop the light goes plain");
  CHECK(fw::gPatch.followCount() == 0 && fw::gPatch.offCount() == 0, "every light out: the panel matches the patch");

  section("patches: guide setting, manual, overwrite, and the name");
  setPot("filter_cutoff", 0.9);
  press("enc1");  // reload: cutoff follows again
  CHECK(fw::gPatch.following(cut), "reloading should set the changed control following again");
  fw::encoderTurn(ctl("enc3"), -1);
  CHECK(fw::gPatch.guide() == fw::GUIDE_TINT && fw::ledColour(cut) == fw::kLedTint, "TINT: a steady tint on following controls");
  fw::encoderTurn(ctl("enc3"), -1);
  CHECK(fw::gPatch.guide() == fw::GUIDE_OFF && fw::ledColour(cut) == fw::kLedIdle, "OFF: every label plain");
  fw::encoderTurn(ctl("enc3"), 5);
  CHECK(fw::gPatch.guide() == fw::GUIDE_ON, "guide should stop at ON");
  press("enc4");  // manual
  CHECK(fw::gPatch.followCount() == 0 && fw::gParams.val[fw::P_CUTOFF] == fw::controlGet(cut), "Manual should make every control live where it stands");
  CHECK(fw::gPatch.edited(), "going manual changes the sound, so the patch counts as changed");

  int saves = gPatchSaves;
  press("enc2");
  CHECK(gPatchSaves == saves, "a used slot must not be overwritten on one push");
  fw::tick(3200);
  press("enc2");
  CHECK(gPatchSaves == saves, "the second push only counts within three seconds");
  press("enc2");
  CHECK(gPatchSaves == saves + 1 && !fw::gPatch.edited(), "push twice to overwrite");
  int slot5Cutoff = fw::gParams.val[fw::P_CUTOFF];  // slot 5 now holds the sound as it is after going manual

  press("btn_patch");  // page 2: the name
  CHECK(fw::gUi.mode() == fw::MB_PATCH && fw::gUi.page() == 1, "second press should show the Name page");
  CHECK(!std::strcmp(fw::gPatch.name(), "INIT    "), "a new sound is called INIT (got '%s')", fw::gPatch.name());
  static const char kWant[] = "BASS 2";
  for (int i = 0; i < fw::kPatchNameLen; ++i) {
    char want = i < 6 ? kWant[i] : ' ';
    fw::encoderTurn(ctl("enc3"), fw::nameCharIndex(want) - fw::nameCharIndex(fw::gPatch.name()[i]));
    fw::encoderTurn(ctl("enc1"), 1);
  }
  CHECK(!std::strcmp(fw::gPatch.name(), "BASS 2  "), "name after editing: '%s'", fw::gPatch.name());
  fw::tick(33);
  saveFrame("mode_patch_name");
  while (fw::gUi.mode() != fw::MB_NONE) press("btn_patch");  // past the Setup page, home
  fw::tick(2100);
  std::vector<uint8_t> homeEdited = grabFrame();
  savePatchTo(6);
  CHECK(fw::gPatch.slot() == 6 && !std::strcmp(fw::gPatch.slotName(6), "BASS 2  "), "saved under its name in slot 7");
  while (fw::gUi.mode() != fw::MB_NONE) press("btn_patch");
  fw::tick(2100);
  saveFrame("home_patch");
  CHECK(countColour(fw::kAccent, 0, 30) > 150, "home should show the slot number and name");
  CHECK(grabFrame() != homeInit && grabFrame() != homeEdited, "the title should change with the patch and lose its changed mark on saving");

  section("patches: MIDI program change loads a slot");
  setPot("filter_cutoff", 0.05);
  midi({0xC0, 4});
  CHECK(fw::gPatch.slot() == 4 && fw::gParams.val[fw::P_CUTOFF] == slot5Cutoff, "program change 4 should load slot 5");
  midi({0xC0, 12});
  CHECK(fw::gPatch.slot() == 4, "a program change to an empty slot does nothing");

  section("patches: the loaded patch, the following controls and the stored slots survive a restart");
  setPot("mix_sub", 0.9);
  midi({0xC0, 6});
  int sub = ctl("mix_sub");
  CHECK(fw::gPatch.following(sub), "set-up: Sub should be following");
  uint16_t want[fw::P_COUNT];
  for (int i = 0; i < fw::P_COUNT; ++i) want[i] = fw::gParams.val[i];
  int followed = fw::gPatch.followCount();
  fw::encoderTurn(ctl("enc3"), 0);
  fw::tick(1100);
  CHECK(gStoreLen == fw::kStateSize, "saved state is %u bytes, expected %u", gStoreLen, fw::kStateSize);
  boot(48000, true);
  wrong = 0;
  for (int i = 0; i < fw::P_COUNT; ++i) wrong += fw::gParams.val[i] != want[i];
  CHECK(wrong == 0, "%d parameters came back different", wrong);
  CHECK(fw::gPatch.slot() == 6 && !std::strcmp(fw::gPatch.name(), "BASS 2  "), "the loaded patch should still be slot 7, BASS 2");
  CHECK(fw::gPatch.followCount() == followed && fw::gPatch.following(sub) && fw::controlGet(sub) == std::lround(0.9 * 4095),
        "the controls that were following should still be, at the same physical positions");
  CHECK(fw::gPatch.slotUsed(4) && fw::gPatch.slotUsed(6) && !fw::gPatch.slotUsed(0), "stored slots should be found again at start-up");

  section("patches: a damaged slot reads as empty; a patch from an older build still loads");
  gBank[4][20] ^= 0x40;
  boot(48000, true);
  CHECK(!fw::gPatch.slotUsed(4) && !fw::patchLoad(4), "a damaged patch must not load");
  {
    const int oldCount = fw::P_COUNT - 4;
    uint8_t blob[fw::kPatchSize] = {'P', 'A', 'T', 1, (uint8_t)oldCount, 'O', 'L', 'D', ' ', ' ', ' ', ' ', ' '};
    uint16_t sum = 0;
    for (int i = 0; i < fw::kPatchNameLen; ++i) sum = (uint16_t)(sum * 31u + blob[5 + i] + 1u);
    for (int i = 0; i < oldCount; ++i) {
      const fw::ParamDef& d = fw::kParams[i];
      uint16_t v = d.kind == fw::PK_STEP ? (uint16_t)(d.steps - 1) : (uint16_t)(700 + i * 9);
      blob[13 + i * 2] = (uint8_t)(v & 0xFF);
      blob[14 + i * 2] = (uint8_t)(v >> 8);
      sum = (uint16_t)(sum * 31u + v + 1u);
    }
    blob[13 + oldCount * 2] = (uint8_t)(sum & 0xFF);
    blob[14 + oldCount * 2] = (uint8_t)(sum >> 8);
    std::memcpy(gBank[9], blob, 15 + oldCount * 2);
    gBankLen[9] = 15 + oldCount * 2;
    boot(48000, true);
    setPot("voice_drift", 0.8);  // one of the four newest parameters
    CHECK(fw::patchLoad(9), "an older, shorter patch should load");
    CHECK(fw::gParams.val[fw::P_CUTOFF] == 700 + fw::P_CUTOFF * 9, "its values should be applied");
    wrong = 0;
    for (int i = oldCount; i < fw::P_COUNT; ++i) wrong += fw::gParams.val[i] != fw::kParams[i].def;
    CHECK(wrong == 0, "%d of the parameters it lacks are not at their defaults", wrong);
  }

  section("patches: a loaded patch sounds as it did when it was saved");
  auto theSound = [] {
    setPot("filter_cutoff", 0.35);
    setPot("filter_resonance", 0.6);
    setSel("osc1_wave", fw::WAVE_SQR);
    setPot("mix_sub", 0.5);
    setPot("chorus", 0.6);
  };
  auto anotherSound = [] {
    setPot("filter_cutoff", 0.9);
    setPot("filter_resonance", 0.1);
    setSel("osc1_wave", fw::WAVE_TRI);
    setPot("mix_sub", 0.0);
  };
  boot(48000);
  theSound();
  savePatchTo(2);
  // each take starts from a fresh start-up, so the three are comparable sample for sample
  boot(48000, true);
  theSound();
  renderNew(0.05);
  std::vector<float> asSaved = phrase();
  boot(48000, true);
  anotherSound();
  renderNew(0.05);
  std::vector<float> changed = phrase();
  boot(48000, true);
  anotherSound();
  CHECK(fw::patchLoad(2), "set-up: slot 3 should load");
  renderNew(0.05);
  std::vector<float> recalled = phrase();
  CHECK(rmsDiff(asSaved, changed) > 1e-3, "set-up: the changed sound should differ");
  CHECK(rmsDiff(asSaved, recalled) < 1e-9, "recalled sound differs from the saved one by %.9f", rmsDiff(asSaved, recalled));
}

// Hands-on finding, 2026-11-14: after a few minutes with the chorus on, the
// sound suddenly collapsed into a roar. A delay-line read position, wrapped
// by adding the buffer length, can round up to exactly that length and read
// the memory after the buffer; in the browser build that memory held wild
// values, which then rang in the reverb. It happened about once in three
// minutes, so no short test saw it. This one plays for minutes with every
// delay line in use. Built normally it checks the level never runs away;
// built with the sanitizers (make sanitize) it also catches the stray read
// itself, whatever the memory next door happens to hold.
static void testLongRun() {
  for (double sr : {44100.0, 48000.0}) {
    char title[96];
    std::snprintf(title, sizeof title, "long run: eight minutes at %.0f Hz with every delay line busy stays in bounds", sr);
    section(title);
    boot(sr);
    setPot("chorus", 1.0);
    setStep(fw::P_DLY_MIX, 60);
    setStep(fw::P_DLY_FDBK, 60);
    setStep(fw::P_RVB_MIX, 60);
    setStep(fw::P_OSC2_ENGINE, fw::ENG_PLUCK);
    setPot("mix_osc2", 0.7);
    for (int k : {48, 55, 60, 63}) midi({0x90, k, 100});
    float l[fw::kMaxBlock], r[fw::kMaxBlock];
    const long perSecond = std::lround(sr / fw::kMaxBlock);
    double normal = 0, worst = 0;
    int worstAt = 0;
    bool finite = true;
    for (int sec = 0; sec < 480; ++sec) {
      if (sec % 3 == 0) {  // a fresh pluck, and a nudge to the delay time so its read position keeps moving
        midi({0x80, 72, 0});
        midi({0x90, 72, 90});
        setStep(fw::P_DLY_TIME, 30 + (sec * 7) % 60);
      }
      double acc = 0;
      for (long b = 0; b < perSecond; ++b) {
        fw::process(l, r, fw::kMaxBlock);
        for (int i = 0; i < fw::kMaxBlock; ++i) {
          finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
          acc += (double)l[i] * l[i] + (double)r[i] * r[i];
        }
        if (b % 12 == 0) fw::tick(32);
      }
      double rms = std::sqrt(acc / (double)(2 * perSecond * fw::kMaxBlock));
      if (sec >= 3 && sec < 13) normal = std::max(normal, rms);
      else if (sec >= 13 && rms > worst) {
        worst = rms;
        worstAt = sec;
      }
    }
    CHECK(finite, "the output stopped being a number");
    CHECK(normal > 0.01, "set-up: there should be sound (level %.4f)", normal);
    CHECK(worst < 2.0 * normal, "the level ran away: %.4f at %d s, against %.4f in the first seconds", worst, worstAt, normal);
  }
}

// Hands-on finding, 2026-11-14: some controls showed the same picture
// whatever they were set to (Macro on a saw, Octave, Detune, Glide...).
// The rule since then: every control's picture follows the control, or
// says in words why it has nothing to show.
static std::vector<uint8_t> pictureRows(int y0, int y1) {
  std::vector<uint8_t> v;
  for (int y = y0; y < y1; ++y)
    for (int x = 0; x < fw::kDisplayW; ++x) v.push_back(fw::gfxGet(x, y));
  return v;
}

static void testPictures() {
  section("display: every panel control's picture changes with the control");
  int blind = 0;
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    boot(48000);
    fw::ControlInfo info = fw::controlInfo(c);
    int id = fw::controlParam(c);
    if (id == fw::P_OSC1_MACRO) setSel("osc1_wave", fw::WAVE_SQR);  // pulse width needs the square
    fw::tick(2000);
    int cur = fw::controlGet(c);
    int a = info.kind == fw::CK_SELECTOR ? (cur + 1) % info.steps : (cur < 2048 ? 3000 : 1000);
    int b = info.kind == fw::CK_SELECTOR ? (info.steps > 2 ? (cur + 2) % info.steps : cur) : (cur < 2048 ? 3900 : 150);
    fw::controlSet(c, a);
    fw::tick(33);
    std::vector<uint8_t> first = pictureRows(48, 218);  // everything under the header, which always shows the value
    fw::controlSet(c, b);
    fw::tick(33);
    if (pictureRows(48, 218) == first) {
      ++blind;
      CHECK(false, "%s: the picture is the same at %d and %d", info.name, a, b);
    }
  }
  CHECK(blind == 0, "%d panel controls have a picture that does not follow them", blind);

  section("display: a control with nothing to do says so");
  boot(48000);
  setPot("osc1_macro", 0.2);
  fw::tick(33);
  std::vector<uint8_t> low = pictureRows(48, 218);
  CHECK(countColour(fw::kGrey, 54, 74) > 150, "Macro on a saw should say it has no effect");
  setPot("osc1_macro", 0.8);
  fw::tick(33);
  CHECK(pictureRows(48, 218) == low, "and the saw's picture should not pretend to change");
  saveFrame("popup_macro_idle");
  setSel("osc1_wave", fw::WAVE_SQR);
  setPot("osc1_macro", 0.5);
  fw::tick(33);
  CHECK(countColour(fw::kGrey, 54, 74) == 0, "on a square there is no such note");
  static const char* const kShots[] = {"osc2_detune", "osc_xmod", "osc_sync", "voice_glide", "voice_mode", "voice_drift", "aenv_velocity",
                                       "lfo_rate", "arp_mode", "mix_sub", "osc1_octave", "volume"};
  for (const char* name : kShots) {
    int c = ctl(name);
    fw::ControlInfo info = fw::controlInfo(c);
    fw::controlSet(c, info.kind == fw::CK_SELECTOR ? 1 : 2900);
    fw::tick(33);
    saveFrame((std::string("picture_") + name).c_str());
  }

  section("display: every value on an encoder page has a picture that follows it");
  static const char* const kButtons[] = {"btn_fx1", "btn_fx2", "btn_mod", "btn_engine"};
  static const char* const kEnc[4] = {"enc1", "enc2", "enc3", "enc4"};
  int shown = 0;
  blind = 0;
  for (const char* button : kButtons) {
    for (int page = 0; page < 2; ++page) {
      boot(48000);
      setStep(fw::P_OSC1_ENGINE, fw::ENG_FM);  // so Color means something
      setStep(fw::P_OSC2_ENGINE, fw::ENG_FM);
      setPot("osc1_macro", 0.6);               // and feedback has some FM to work on
      setStep(fw::P_OSC2_MACRO, 60);
      setStep(fw::P_DLY_MIX, 50);              // repeats and a tail to draw
      setStep(fw::P_RVB_MIX, 50);
      fw::tick(2000);
      press(button);
      if (page == 1) press(button);
      if (fw::gUi.mode() == fw::MB_NONE) continue;  // this mode has no second page
      for (int e = 0; e < 4; ++e) {
        fw::tick(2000);
        std::vector<uint8_t> scope = pictureRows(76, 154);
        uint16_t was[fw::P_COUNT];
        for (int i = 0; i < fw::P_COUNT; ++i) was[i] = fw::gParams.val[i];
        fw::encoderTurn(ctl(kEnc[e]), 1);
        int id = -1;
        for (int i = 0; i < fw::P_COUNT; ++i)
          if (was[i] != fw::gParams.val[i]) id = i;
        if (id < 0) {  // already at the top of its range, or an empty corner
          fw::encoderTurn(ctl(kEnc[e]), -1);
          for (int i = 0; i < fw::P_COUNT; ++i)
            if (was[i] != fw::gParams.val[i]) id = i;
          if (id < 0) continue;
        }
        ++shown;
        fw::tick(33);
        std::vector<uint8_t> first = pictureRows(76, 154);
        bool small = fw::kParams[id].steps <= 3;
        fw::tick(40);
        fw::encoderTurn(ctl(kEnc[e]), small ? (fw::gParams.step(id) > 0 ? -1 : 1) : 6);
        fw::tick(33);
        std::vector<uint8_t> second = pictureRows(76, 154);
        if (first == scope || first == second) {
          ++blind;
          CHECK(false, "%s: %s", fw::kParams[id].key, first == scope ? "no picture, only the scope" : "the picture does not change with the value");
        }
        if (e == 0 || id == fw::P_RVB_SIZE || id == fw::P_FENV_PITCH) saveFrame((std::string("soft_") + fw::kParams[id].key).c_str());
      }
    }
  }
  CHECK(shown == 23, "expected 23 values across the encoder pages, found %d", shown);
  CHECK(blind == 0, "%d encoder-page values have no picture of their own", blind);
  boot(48000);
  press("btn_fx1");
  fw::encoderTurn(ctl("enc1"), 2);
  fw::tick(33);
  std::vector<uint8_t> pic = pictureRows(76, 154);
  fw::tick(1600);
  fw::tick(33);
  CHECK(pictureRows(76, 154) != pic, "the scope should come back about a second and a half after the encoder stops");

  section("display: two palettes, chosen on the Setup page and kept over a restart");
  boot(48000);
  fw::tick(33);
  uint16_t amberRow[fw::kDisplayW], whiteRow[fw::kDisplayW];
  fw::frameRow565(12, amberRow);
  std::vector<uint8_t> homeFrame = grabFrame();
  CHECK(fw::gfxPalette() == fw::PALETTE_AMBER, "the display starts in amber");
  for (int i = 1; i < fw::kColourCount; ++i) {  // amber means amber: every colour warm, none bluish
    uint16_t p = fw::gfxPalette565()[i];
    int r = (p >> 11) << 3, g = ((p >> 5) & 63) << 2, b = (p & 31) << 3;
    CHECK(r >= g && g >= b, "amber palette colour %d is not warm (%d, %d, %d)", i, r, g, b);
  }
  press("btn_patch");
  press("btn_patch");
  press("btn_patch");
  fw::tick(33);
  saveFrame("mode_setup");
  fw::encoderTurn(ctl("enc1"), 1);
  CHECK(fw::gfxPalette() == fw::PALETTE_WHITE, "encoder 1 on the Setup page should switch to the white palette");
  while (fw::gUi.mode() != fw::MB_NONE) press("btn_patch");
  fw::tick(2000);
  fw::tick(33);
  fw::frameRow565(12, whiteRow);
  CHECK(grabFrame() == homeFrame, "the palette changes colours only, not what is drawn");
  CHECK(std::memcmp(amberRow, whiteRow, sizeof amberRow) != 0, "the title row should come out in different colours");
  saveFrame("home_white");
  fw::tick(1200);
  CHECK(gStoreLen == fw::kStateSize, "state should be saved with the palette (%u bytes)", gStoreLen);
  boot(48000, true);
  CHECK(fw::gfxPalette() == fw::PALETTE_WHITE, "the palette should survive a restart");
  {  // state saved before the palette, the lights setting and the settings existed is shorter and must still load
    uint8_t old[1024];
    uint32_t n = gStoreLen - 2 - fw::SET_COUNT;
    std::memcpy(old, gStore, n - 2);
    uint16_t sum = 0;
    for (int i = 0; i < fw::P_COUNT; ++i) sum = (uint16_t)(sum * 31u + (old[6 + i * 2] | (old[7 + i * 2] << 8)) + 1u);
    for (uint32_t i = 6 + fw::P_COUNT * 2; i < n - 2; ++i) sum = (uint16_t)(sum * 31u + old[i] + 1u);
    old[n - 2] = (uint8_t)(sum & 0xFF);
    old[n - 1] = (uint8_t)(sum >> 8);
    setPot("filter_cutoff", 0.123);
    std::memcpy(gStore, old, n);
    gStoreLen = n;
    boot(48000, true);
    CHECK(fw::gfxPalette() == fw::PALETTE_AMBER, "older state has no palette: amber");
    CHECK(fw::gParams.val[fw::P_CUTOFF] != std::lround(0.123 * 4095), "older state should still load its values");
  }
}

// ------------------------------------------------- findings of the independent review, 2026-11-15

// Lets go of everything a player could be holding.
static void letGoOfEverything(const std::vector<int>& keys) {
  for (int k : keys) midi({0x80, k, 0});
  midi({0xB0, 64, 0});
  if (fw::gParams.step(fw::P_HOLD)) press("btn_hold");
}

static void testNotesEndInSilence() {
  section("review: the sustain pedal held across an arp on/off switch leaves nothing stuck");
  boot(48000);
  midi({0xB0, 64, 127});                    // pedal down, then the arp goes on
  setSel("arp_mode", fw::ARP_UP);
  midi({0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100});
  renderNew(0.5);
  midi({0x80, 60, 0, 0x80, 64, 0, 0x80, 67, 0, 0xB0, 64, 0});
  renderNew(2.0);
  CHECK(fw::gSynth.activeVoices() == 0, "arp on with the pedal down: %d voices still sounding", fw::gSynth.activeVoices());
  setSel("arp_mode", fw::ARP_OFF);
  midi({0x90, 62, 100});
  renderNew(0.2);
  midi({0x80, 62, 0});
  renderNew(2.0);
  CHECK(fw::gSynth.activeVoices() == 0, "a plain note afterwards stuck (%d voices)", fw::gSynth.activeVoices());
  boot(48000);
  setSel("arp_mode", fw::ARP_UP);           // the pedal as a temporary Hold, then the arp goes off
  midi({0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100, 0xB0, 64, 127});
  renderNew(0.5);
  midi({0x80, 60, 0, 0x80, 64, 0, 0x80, 67, 0});
  renderNew(0.5);
  setSel("arp_mode", fw::ARP_OFF);
  renderNew(0.2);
  midi({0xB0, 64, 0});
  renderNew(3.0);
  CHECK(fw::gSynth.activeVoices() == 0, "arp off with latched keys: %d voices still sounding", fw::gSynth.activeVoices());

  section("review: any mix of keys, pedal, Hold, Chord, arp and voice mode ends in silence once everything is let go");
  uint32_t rng = 0xC0FFEEu;
  auto rnd = [&rng](int n) {
    rng = rng * 1664525u + 1013904223u;
    return (int)((rng >> 10) % (uint32_t)n);
  };
  int stuck = 0;
  for (int run = 0; run < 150 && stuck == 0; ++run) {
    boot(48000);
    setPot("aenv_release", 0.2);
    std::vector<int> keys;
    std::string story;
    for (int step = 0; step < 50; ++step) {
      int a = rnd(100);
      char what[40];
      if (a < 40) {
        int k = 40 + rnd(30);
        midi({0x90, k, 100});
        keys.push_back(k);
        std::snprintf(what, sizeof what, "down %d", k);
      } else if (a < 62 && !keys.empty()) {
        int at = rnd((int)keys.size());
        midi({0x80, keys[at], 0});
        std::snprintf(what, sizeof what, "up %d", keys[at]);
        keys.erase(keys.begin() + at);
      } else if (a < 72) {
        int v = rnd(2) * 127;
        midi({0xB0, 64, v});
        std::snprintf(what, sizeof what, "pedal %d", v);
      } else if (a < 80) {
        press("btn_hold");
        std::snprintf(what, sizeof what, "hold");
      } else if (a < 86) {
        press("btn_chord");
        std::snprintf(what, sizeof what, "chord");
      } else if (a < 94) {
        int m = rnd(6);
        setSel("arp_mode", m);
        std::snprintf(what, sizeof what, "arp %d", m);
      } else {
        int m = rnd(3);
        setSel("voice_mode", m);
        std::snprintf(what, sizeof what, "mode %d", m);
      }
      story += std::string(what) + ", ";
      renderNew(0.01 * (1 + rnd(8)));
    }
    letGoOfEverything(keys);
    renderNew(3.0);
    if (fw::gSynth.activeVoices() != 0) {
      ++stuck;
      CHECK(false, "run %d: %d voices never stopped after: %s", run, fw::gSynth.activeVoices(), story.c_str());
    }
  }
  CHECK(stuck == 0, "stuck notes found");

  section("review: a burst of MIDI between two audio blocks loses no note-off");
  boot(48000);
  for (int i = 0; i < 100; ++i) midi({0x90, 30 + i % 60, 100});
  for (int i = 0; i < 100; ++i) midi({0x80, 30 + i % 60, 0});
  renderNew(2.0);
  CHECK(fw::gSynth.activeVoices() == 0, "200 events in one block: %d voices stuck", fw::gSynth.activeVoices());
  for (int i = 0; i < 400; ++i) midi({0x90, 30 + i % 60, 100});  // more than the queue holds: notes are lost,
  renderNew(0.1);                                                  // so the engine lets everything go
  for (int i = 0; i < 60; ++i) midi({0x80, 30 + i, 0});
  renderNew(2.0);
  CHECK(fw::gSynth.activeVoices() == 0, "after an overflow: %d voices stuck", fw::gSynth.activeVoices());
  midi({0x90, 60, 100});
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() == 1, "and the engine should play normally again afterwards");
  for (int i = 0; i < 2000; ++i) midi({0xE0, i % 128, 64 + i % 60, 0xB0, 1, i % 128});  // wheels cannot crowd the notes out
  midi({0x80, 60, 0});
  renderNew(2.0);
  CHECK(fw::gSynth.activeVoices() == 0, "a note-off after a storm of wheel messages was lost");

  section("review: Panic silences the delay and the reverb as well as the voices");
  boot(48000);
  setStep(fw::P_DLY_MIX, 80);
  setStep(fw::P_DLY_FDBK, 100);
  setStep(fw::P_RVB_MIX, 80);
  setStep(fw::P_RVB_SIZE, 100);
  midi({0x90, 60, 120, 0x90, 67, 120});
  renderNew(1.0);
  midi({0xB0, 120, 0});
  renderNew(0.02);
  CHECK(peakOf(renderNew(1.0)) == 0.0, "sound left after Panic: peak %.5f", peakOf(renderNew(1.0)));

  section("review: a note-on and note-off arriving together still sounds");
  for (int engine : {(int)fw::ENG_VA, (int)fw::ENG_PLUCK}) {
    boot(48000);
    setStep(fw::P_OSC1_ENGINE, engine);
    renderNew(0.05);
    midi({0x90, 60, 110, 0x80, 60, 0});
    double peak = peakOf(renderNew(0.2));
    CHECK(peak > 0.02, "engine %d: a note shorter than one block made no sound (peak %.5f)", engine, peak);
    renderNew(3.0);
    CHECK(fw::gSynth.activeVoices() == 0, "and it should still end");
  }

  section("review: a plucked string does not wait for its fader");
  boot(48000);
  setStep(fw::P_OSC2_ENGINE, fw::ENG_PLUCK);
  setPot("mix_osc1", 0.0);
  setPot("mix_osc2", 0.0);
  setPot("aenv_sustain", 1.0);
  renderNew(0.05);
  midi({0x90, 60, 110});
  renderNew(2.0);              // the pluck happens, unheard
  setPot("mix_osc2", 0.8);
  double late = rmsOf(renderNew(0.3), 0.0, 0.3);
  CHECK(late < 0.003, "raising the fader two seconds later replayed the pluck (level %.4f)", late);

  section("review: a pluck too low for its string plays an octave up, not a wrong note");
  boot(48000);
  setStep(fw::P_OSC1_ENGINE, fw::ENG_PLUCK);
  setSel("osc1_octave", 0);    // two octaves down
  setPot("mix_osc2", 0.0);
  setPot("filter_cutoff", 1.0);
  setStep(fw::P_OSC1_COLOR, 100);
  renderNew(0.05);
  midi({0x90, 36, 110});       // C2 two octaves down is 16.4 Hz: its period does not fit the string
  std::vector<float> low = renderNew(1.5);
  // The wave should repeat every 1/32.7 s (C1, the nearest octave that fits),
  // and not at the clamped length it used to play, which was no note at all.
  auto repeatsAt = [&low](double hz) {
    size_t lag = (size_t)std::lround(gSr / hz), from = (size_t)(0.4 * gSr), n = (size_t)(0.8 * gSr);
    double best = 0;
    for (size_t d = lag - 3; d <= lag + 3; ++d) {
      double ab = 0, aa = 0, bb = 0;
      for (size_t i = from; i < from + n; ++i) {
        ab += (double)low[i] * low[i + d];
        aa += (double)low[i] * low[i];
        bb += (double)low[i + d] * low[i + d];
      }
      best = std::max(best, ab / std::sqrt(aa * bb + 1e-30));
    }
    return best;
  };
  CHECK(repeatsAt(32.703) > 0.9, "the pluck should play C1; its likeness to itself one C1 period later is only %.2f", repeatsAt(32.703));
  CHECK(repeatsAt(23.46) < 0.6, "it still repeats at the old clamped length (%.2f)", repeatsAt(23.46));
}

static void testHighSampleRates() {
  for (double sr : {96000.0, 192000.0}) {
    char title[96];
    std::snprintf(title, sizeof title, "review: at %.0f Hz everything stays in bounds and finite, effects on", sr);
    section(title);
    boot(sr);
    setPot("chorus", 1.0);
    setStep(fw::P_DLY_MIX, 70);
    setStep(fw::P_DLY_TIME, 100);
    setStep(fw::P_DLY_FDBK, 70);
    setStep(fw::P_RVB_MIX, 70);
    setStep(fw::P_RVB_SIZE, 100);
    setStep(fw::P_OSC2_ENGINE, fw::ENG_PLUCK);
    setSel("osc1_wave", fw::WAVE_SUPER);
    for (int k : {36, 48, 60, 72}) midi({0x90, k, 100});
    std::vector<float> x = renderNew(1.5);
    bool finite = true;
    for (float v : x) finite = finite && std::isfinite(v);
    CHECK(finite && peakOf(x) > 0.01 && peakOf(x) <= 1.0, "output at %.0f Hz: finite %d, peak %.3f", sr, (int)finite, peakOf(x));
    midi({0xB0, 123, 0});
    setPot("aenv_release", 0.1);
    setStep(fw::P_DLY_MIX, 0);
    renderNew(1.0);
    // the reverb must still be a reverb: after the notes stop its tail should fade smoothly, not ring at one pitch
    std::vector<float> tail = renderNew(0.5);
    double early = rmsOf(tail, 0.0, 0.12), later = rmsOf(tail, 0.38, 0.5);
    CHECK(early > 0 && later < early, "reverb tail should be dying away (%.5f then %.5f)", early, later);
  }
}

static void testOverloadAndSaving() {
  section("review: when the processor is overloaded the engine sheds voices instead of falling behind");
  boot(48000);
  for (int k = 0; k < 8; ++k) midi({0x90, 40 + k * 3, 100});
  renderNew(0.2);
  CHECK(fw::gSynth.activeVoices() == 8 && fw::gSynth.voiceLimit() == 8, "set-up: eight voices, no limit");
  fw::reportLoad(0.5f);
  renderNew(0.5);
  CHECK(fw::gSynth.activeVoices() == 8, "half the time used is no reason to drop anything");
  fw::reportLoad(0.97f);
  renderNew(0.02);
  int after = fw::gSynth.activeVoices();
  CHECK(after < 8 && after >= 2, "a sustained overload should drop a voice within a few blocks (%d sounding)", after);
  renderNew(1.0);
  CHECK(fw::gSynth.activeVoices() == 2 && fw::gSynth.voiceLimit() == 2, "it keeps shedding while overloaded, but never below two (%d sounding, limit %d)",
        fw::gSynth.activeVoices(), fw::gSynth.voiceLimit());
  midi({0x90, 90, 100});
  renderNew(0.05);
  CHECK(fw::gSynth.activeVoices() == 2, "a new note should take a voice over, not add one (%d sounding)", fw::gSynth.activeVoices());
  std::vector<float> x = renderNew(0.2);
  bool finite = true;
  for (float v : x) finite = finite && std::isfinite(v);
  CHECK(finite, "output while shedding");
  fw::reportLoad(0.3f);
  renderNew(1.0);
  CHECK(fw::gSynth.voiceLimit() == 2, "one calm second is not enough to give a voice back");
  renderNew(14.0);
  CHECK(fw::gSynth.voiceLimit() == 8, "after a calm spell every voice should be allowed again (limit %d)", fw::gSynth.voiceLimit());
  fw::tick(33);
  CHECK(countColour(fw::kDarkGrey, 225, 233) + countColour(fw::kLine, 225, 233) >= 8 * 64, "the status strip should show all eight voice squares again");

  section("review: saving waits for silence, so a save cannot interrupt a held note");
  boot(48000);
  gSaveCount = 0;
  midi({0x90, 60, 100});
  renderNew(0.1);
  setPot("filter_cutoff", 0.4);
  for (int i = 0; i < 100; ++i) {  // five seconds with the note held
    renderNew(0.05);
    fw::tick(50);
  }
  CHECK(gSaveCount == 0, "saved while a note was sounding");
  midi({0x80, 60, 0});
  for (int i = 0; i < 100 && gSaveCount == 0; ++i) {
    renderNew(0.05);
    fw::tick(50);
  }
  CHECK(gSaveCount == 1, "it should save once the note has died away (%d saves)", gSaveCount);
  midi({0x90, 60, 100});
  renderNew(0.1);
  setPot("filter_cutoff", 0.6);
  for (int i = 0; i < 700 && gSaveCount == 1; ++i) {  // a drone: it does not wait for ever
    renderNew(0.05);
    fw::tick(50);
  }
  CHECK(gSaveCount == 2, "after half a minute it should save anyway");

  section("review: Hold, like Chord, does not come back on by itself after a restart");
  boot(48000);
  press("btn_hold");
  CHECK(fw::gParams.step(fw::P_HOLD) == 1, "set-up: Hold on");
  fw::tick(1200);
  boot(48000, true);
  CHECK(fw::gParams.step(fw::P_HOLD) == 0 && fw::controlGet(ctl("btn_hold")) == 0, "Hold should be off after a restart");
  CHECK(fw::kParams[fw::P_HOLD].section == fw::SEC_ARP, "Hold belongs with the arp, where its button is");

  section("review: master volume by MIDI behaves like every other control: the knob stays put and follows");
  boot(48000);
  int vol = ctl("volume");
  int was = fw::controlGet(vol);
  setCC(fw::P_VOLUME, 30);
  CHECK(fw::controlGet(vol) == was && fw::gPatch.following(vol), "the volume knob must not move by itself");
  CHECK(fw::gParams.val[fw::P_VOLUME] == (30 * 4095 + 63) / 127, "the sound takes the value");
  fw::controlSet(vol, was - 400);
  CHECK(fw::gPatch.following(vol), "moving without reaching the value does not take over");
  fw::controlSet(vol, 500);
  CHECK(!fw::gPatch.following(vol) && fw::gParams.val[fw::P_VOLUME] == 500, "moving through the value does");
}

// A real pot's converter reading: the true position plus jitter.
static int roughReading(int truth, uint32_t& rng, int jitter = 8) {
  rng = rng * 1664525u + 1013904223u;
  int r = truth + (int)((rng >> 9) % (uint32_t)(2 * jitter + 1)) - jitter;
  return r < 0 ? 0 : (r > 4095 ? 4095 : r);
}

static void testPotReadings() {
  section("pots: jitter on a resting knob is not a movement");
  boot(48000);
  int cut = ctl("filter_cutoff"), res = ctl("filter_resonance");
  uint32_t rng = 99;
  int truth = fw::controlGet(cut);
  fw::tick(2000);
  gSaveCount = 0;
  int reports = 0, last = fw::controlGet(cut);
  for (int i = 0; i < 20000; ++i) {  // more than three minutes of scanning at 100 Hz
    fw::potReading(cut, roughReading(truth, rng));
    if (fw::controlGet(cut) != last) {
      ++reports;
      last = fw::controlGet(cut);
    }
    if (i % 3 == 0) fw::tick(30);
  }
  CHECK(reports == 0, "a still knob reported %d movements", reports);
  CHECK(fw::gUi.popupParam() < 0, "jitter must not bring up the control's picture");
  CHECK(gSaveCount == 0, "nor keep restarting the save timer (%d saves)", gSaveCount);

  section("pots: a turned knob is followed closely and both ends can be reached");
  int worst = 0;
  for (int step = 0; step <= 40; ++step) {
    truth = step * 4095 / 40;
    for (int i = 0; i < 40; ++i) fw::potReading(cut, roughReading(truth, rng));  // 0.4 s at 100 Hz
    worst = std::max(worst, std::abs(fw::controlGet(cut) - truth));
  }
  CHECK(worst <= 16, "settled readings should be within 16 counts of the true position (worst %d)", worst);
  CHECK(fw::controlGet(cut) == 4095 && fw::gParams.val[fw::P_CUTOFF] == 4095, "the top must be reachable (got %d)", fw::controlGet(cut));
  truth = 3;
  for (int i = 0; i < 60; ++i) fw::potReading(cut, roughReading(truth, rng));
  CHECK(fw::controlGet(cut) == 0, "the bottom must be reachable (got %d)", fw::controlGet(cut));
  truth = 2000;
  int need = 0;
  while (std::abs(fw::controlGet(cut) - truth) > 40 && need < 100) {
    fw::potReading(cut, roughReading(truth, rng));
    ++need;
  }
  CHECK(need <= 6, "a quick turn should be followed within a few readings (took %d)", need);

  section("pots: a little bleed from a neighbour is absorbed; a lot is not, and has to be dealt with in the circuit");
  int resTruth = fw::controlGet(res), moves = 0;
  last = resTruth;
  for (int sweep = 0; sweep < 4095; sweep += 15) {  // the neighbouring pot swept end to end, 0.15% of it leaking in
    int reading = roughReading(resTruth, rng) + (int)std::lround(0.0015 * (sweep - resTruth));
    fw::potReading(res, reading);
    moves += fw::controlGet(res) != last;
    last = fw::controlGet(res);
  }
  CHECK(moves == 0, "0.15%% of a neighbour's swing moved a resting pot %d times", moves);
  moves = 0;
  for (int sweep = 0; sweep < 4095; sweep += 15) {  // 2%: what an unbuffered multiplexer read too quickly can do
    int reading = roughReading(resTruth, rng) + (int)std::lround(0.02 * (sweep - resTruth));
    fw::potReading(res, reading);
    moves += fw::controlGet(res) != last;
    last = fw::controlGet(res);
  }
  std::printf("    (for information: with 2%% bleed the resting pot was moved %d times by its neighbour; the firmware cannot hide that)\n", moves);

  section("pots: taking over from a patch still works through rough readings");
  boot(48000);
  rng = 7;
  setPot("filter_cutoff", 0.3);
  savePatchTo(0);
  while (fw::gUi.mode() != fw::MB_NONE) press("btn_patch");
  setPot("filter_cutoff", 0.8);
  fw::patchLoad(0);
  int stored = fw::gParams.val[fw::P_CUTOFF];
  truth = fw::controlGet(cut);
  CHECK(fw::gPatch.following(cut), "set-up: Cutoff should be following the patch");
  for (int i = 0; i < 3000; ++i) fw::potReading(cut, roughReading(truth, rng));
  CHECK(fw::gPatch.following(cut) && fw::gParams.val[fw::P_CUTOFF] == stored, "jitter on a following knob must not take over or change the sound");
  bool jumped = false;
  for (; truth > stored - 300; truth -= 12) {  // turned down through the stored value
    int before = fw::gParams.val[fw::P_CUTOFF];
    for (int i = 0; i < 3; ++i) fw::potReading(cut, roughReading(truth, rng));
    jumped = jumped || std::abs((int)fw::gParams.val[fw::P_CUTOFF] - before) > 140;
  }
  CHECK(!fw::gPatch.following(cut), "turning through the stored value should take over");
  CHECK(!jumped, "and the sound should not jump on the way");

  section("pots: at power-up a knob left where it was is not a movement, and one that was turned is");
  boot(48000);
  fw::tick(2000);
  rng = 3;
  int restAt = fw::controlGet(cut);
  fw::potReading(cut, restAt + 13);  // the worst a first reading should be: full jitter plus a trace of a neighbour
  for (int i = 0; i < 200; ++i) fw::potReading(cut, roughReading(restAt, rng));
  CHECK(fw::controlGet(cut) == restAt && fw::gUi.popupParam() < 0, "first readings within the jitter should change nothing");
  fw::potReading(res, 3900);
  CHECK(fw::controlGet(res) == 3900, "a knob turned while the power was off should be picked up at once");
}

static void testReviewPictures() {
  section("review: pictures that misled under particular settings now tell the truth");
  auto hintShown = [] { return countColour(fw::kGrey, 54, 74) > 150; };

  boot(48000);  // Filter Type under the ladder
  setStep(fw::P_FILT_SLOPE, fw::SLOPE_LADDER);
  setSel("filter_type", fw::FILT_HP);
  fw::tick(33);
  CHECK(hintShown(), "Type under the ladder slope should say the ladder is low-pass only");
  saveFrame("review_type_ladder");

  boot(48000);  // Sync
  setStep(fw::P_OSC2_ENGINE, fw::ENG_PLUCK);
  setSel("osc_sync", 1);
  fw::tick(33);
  CHECK(hintShown() && countColour(fw::kAccent, 70, 90) == 0, "Sync with oscillator 2 plucked should say it has no effect and draw no restarts");
  boot(48000);
  setSel("osc2_octave", 1);  // an octave below oscillator 1: sync chops it, which is plainly audible
  setSel("osc_sync", 1);
  fw::tick(33);
  std::vector<uint8_t> on = pictureRows(76, 216);
  setSel("osc_sync", 0);
  fw::tick(33);
  CHECK(pictureRows(76, 216) != on, "Sync with oscillator 2 below oscillator 1 should show the wave being cut short");
  saveFrame("review_sync_below");

  boot(48000);  // X-Mod
  setStep(fw::P_OSC1_ENGINE, fw::ENG_PLUCK);
  setPot("osc_xmod", 0.3);
  fw::tick(33);
  std::vector<uint8_t> a = pictureRows(76, 216);
  setPot("osc_xmod", 0.9);
  fw::tick(33);
  CHECK(hintShown() && pictureRows(76, 216) == a, "X-Mod on a plucked oscillator 1 should say so and not pretend to bend it");
  boot(48000);
  setPot("mix_osc1", 0.0);
  setPot("osc_xmod", 0.7);
  fw::tick(33);
  CHECK(hintShown(), "X-Mod with oscillator 1's fader down should say so");
  boot(48000);  // the table read past its end that the review's sanitizer run caught
  setSel("osc1_wave", fw::WAVE_SIN);
  setSel("osc2_wave", fw::WAVE_SUPER);
  for (int raw : {1, 2, 3, 4094}) {
    fw::controlSet(ctl("osc_xmod"), raw);
    fw::tick(33);
  }

  boot(48000);  // Spread in mono
  setSel("voice_mode", fw::MODE_MONO);
  setPot("voice_spread", 0.2);
  fw::tick(33);
  a = pictureRows(80, 216);
  setPot("voice_spread", 0.9);
  fw::tick(33);
  CHECK(hintShown() && pictureRows(80, 216) == a, "Spread in mono should say the voice stays in the middle and draw it there");

  boot(48000);  // Wave under the pluck engine: five exciters, five pictures
  setStep(fw::P_OSC1_ENGINE, fw::ENG_PLUCK);
  std::vector<std::vector<uint8_t>> seen;
  for (int w = 0; w < fw::WAVE_COUNT; ++w) {
    setSel("osc1_wave", (w + 1) % fw::WAVE_COUNT);
    fw::tick(33);
    std::vector<uint8_t> pic = pictureRows(48, 218);
    bool fresh = true;
    for (const auto& other : seen) fresh = fresh && other != pic;
    if (fresh) seen.push_back(pic);
  }
  CHECK(seen.size() == (size_t)fw::WAVE_COUNT, "the five pluck exciters should each look different (%zu distinct)", seen.size());
  saveFrame("review_pluck_exciter");

  boot(48000);  // Env>Pitch settles where the filter envelope's Sustain holds it
  setPot("fenv_sustain", 0.9);
  setCC(fw::P_FENV_PITCH, 70);
  setCC(fw::P_FENV_PITCH, 62);
  fw::tick(33);
  a = pictureRows(48, 218);
  setPot("fenv_sustain", 0.1);
  setCC(fw::P_FENV_PITCH, 70);
  setCC(fw::P_FENV_PITCH, 62);
  fw::tick(33);
  CHECK(pictureRows(48, 218) != a, "the Env>Pitch picture should show the pitch held up by the filter envelope's Sustain");
  saveFrame("review_env_pitch");

  boot(48000);  // an encoder turned while a control's picture is up
  press("btn_fx1");
  setPot("filter_cutoff", 0.3);
  CHECK(fw::gUi.popupParam() == fw::P_CUTOFF, "set-up: the Cutoff picture is up");
  fw::encoderTurn(ctl("enc1"), 2);
  CHECK(fw::gUi.popupParam() < 0, "turning an encoder should bring its page back, so the value being changed is seen");
}

// The firmware's whole API driven at random, at several sample rates and
// with controls at the very ends of their travel. On its own this checks
// only that the output stays finite; under the sanitizers (make sanitize)
// it is the search for reads and writes outside a buffer.
static void testRandomUse() {
  section("review: random use of every control, button, encoder and MIDI message, 44.1 to 192 kHz");
  uint32_t rng = 0xBADC0DEu;
  auto rnd = [&rng](int n) {
    rng = rng * 1664525u + 1013904223u;
    return (int)((rng >> 9) % (uint32_t)n);
  };
  static const double kRates[] = {44100, 48000, 88200, 96000, 192000};
  static const int kEdges[] = {0, 1, 2, 2047, 2048, 4093, 4094, 4095};
  bool finite = true;
  for (int run = 0; run < 10; ++run) {
    boot(kRates[run % 5]);
    float l[fw::kMaxBlock], r[fw::kMaxBlock];
    for (int step = 0; step < 1500; ++step) {
      int a = rnd(100);
      if (a < 30) {
        int c = rnd(fw::kFirstEncoder);
        fw::ControlInfo info = fw::controlInfo(c);
        fw::controlSet(c, info.kind == fw::CK_SELECTOR ? rnd(info.steps) : (rnd(2) ? kEdges[rnd(8)] : rnd(4096)));
      } else if (a < 40) {
        fw::potReading(rnd(fw::kFirstEncoder), rnd(4200) - 50);
      } else if (a < 55) {
        fw::midiByte((uint8_t)(rnd(2) ? 0x90 : 0x80));
        fw::midiByte((uint8_t)rnd(128));
        fw::midiByte((uint8_t)rnd(128));
      } else if (a < 70) {
        fw::midiByte(0xB0);
        fw::midiByte((uint8_t)rnd(128));
        fw::midiByte((uint8_t)rnd(128));
      } else if (a < 75) {
        fw::midiByte((uint8_t)rnd(256));  // anything at all
      } else if (a < 85) {
        int c = fw::kFirstEncoder + rnd(fw::kNumEncoders + fw::kNumButtons);
        fw::controlSet(c, 1);
        fw::controlSet(c, 0);
      } else if (a < 93) {
        fw::encoderTurn(fw::kFirstEncoder + rnd(4), rnd(41) - 20);
      } else if (a < 96) {
        fw::midiByte(0xC0);
        fw::midiByte((uint8_t)rnd(20));
      } else {
        fw::reportLoad((float)rnd(120) * 0.01f);
      }
      int len = 1 + rnd(fw::kMaxBlock);
      fw::process(l, r, len);
      for (int i = 0; i < len; ++i) finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]) && std::fabs(l[i]) <= 1.0f && std::fabs(r[i]) <= 1.0f;
      if (step % 7 == 0) fw::tick((uint32_t)(1 + rnd(60)));
    }
  }
  CHECK(finite, "the output left the range -1..1 or stopped being a number");
}

// The label lights' dance mode: each section's labels follow what that section is doing.
static void testDanceLights() {
  section("lights: steady by default; Dance is chosen on the Setup page and kept over a restart");
  boot(48000);
  int cut = ctl("filter_cutoff"), rate = ctl("lfo_rate"), tempo = ctl("arp_tempo"), vol = ctl("volume");
  int fa = ctl("fenv_attack"), fd = ctl("fenv_decay"), fs = ctl("fenv_sustain"), fr = ctl("fenv_release");
  auto bright = [](int c) {
    uint32_t v = fw::ledColour(c);
    return (int)((v >> 16) & 255) + (int)((v >> 8) & 255) + (int)(v & 255);
  };
  // advances sound and lights together, as the hardware loop does; returns the brightest a light got
  auto play = [&](double sec, int c) {
    int most = 0;
    for (int i = 0; i < (int)(sec * 30); ++i) {
      renderNew(1.0 / 30);
      fw::tick(33);
      most = std::max(most, bright(c));
    }
    return most;
  };
  midi({0x90, 60, 110});
  play(0.3, cut);
  bool plain = true;
  for (int c = 0; c < fw::kFirstEncoder; ++c) plain = plain && fw::ledColour(c) == fw::kLedIdle;
  CHECK(plain && fw::lightsMode() == fw::LIGHTS_STEADY, "with Dance off every label is plain, whatever is playing");
  midi({0x80, 60, 0});
  play(1.5, cut);
  press("btn_patch");
  press("btn_patch");
  press("btn_patch");
  fw::encoderTurn(ctl("enc3"), 1);
  CHECK(fw::lightsMode() == fw::LIGHTS_DANCE, "encoder 3 on the Setup page should switch Dance on");
  fw::tick(33);
  saveFrame("mode_setup_dance");
  press("btn_patch");
  fw::tick(1200);
  boot(48000, true);
  CHECK(fw::lightsMode() == fw::LIGHTS_DANCE, "the setting should survive a restart");

  section("lights: in Dance, each section follows its own part of the sound, in ambers only");
  setPot("fenv_attack", 0.62);   // slow enough to see each stage: about 0.3 s up, 0.8 s down
  setPot("fenv_decay", 0.72);
  setPot("fenv_sustain", 0.5);
  setPot("fenv_release", 0.6);
  setPot("aenv_release", 0.6);
  setPot("lfo_rate", 0.55);      // about 1.7 Hz
  int rest = play(0.5, cut);
  bool warm = true, still = true;
  uint32_t first[64];
  for (int c = 0; c < fw::kFirstEncoder; ++c) first[c] = fw::ledColour(c);
  play(1.0, cut);
  for (int c = 0; c < fw::kFirstEncoder; ++c)
    if (fw::kParams[fw::controlParam(c)].section != fw::SEC_LFO) still = still && fw::ledColour(c) == first[c];
  CHECK(still, "with nothing playing only the LFO's labels should be moving");
  int lfoLow = 9999, lfoHigh = 0;
  for (int i = 0; i < 60; ++i) {
    renderNew(1.0 / 30);
    fw::tick(33);
    lfoLow = std::min(lfoLow, bright(rate));
    lfoHigh = std::max(lfoHigh, bright(rate));
  }
  CHECK(lfoHigh > lfoLow + 200, "the LFO's labels should rise and fall with it (%d..%d)", lfoLow, lfoHigh);

  midi({0x90, 60, 127});
  int cutPeak = 0, volPeak = 0, attackPeak = 0, stage[4] = {0, 0, 0, 0};
  for (int i = 0; i < 150; ++i) {  // five seconds held
    renderNew(1.0 / 30);
    fw::tick(33);
    cutPeak = std::max(cutPeak, bright(cut));
    volPeak = std::max(volPeak, bright(vol));
    attackPeak = std::max(attackPeak, bright(fa));
    int lit = 0;
    for (int k = 1; k < 4; ++k)
      if (bright(fa + k) > bright(fa + lit)) lit = k;
    stage[lit]++;
    for (int c = 0; c < fw::kFirstEncoder; ++c) {
      uint32_t v = fw::ledColour(c);
      warm = warm && ((v >> 16) & 255) >= ((v >> 8) & 255) && ((v >> 8) & 255) >= (v & 255);
    }
  }
  CHECK(fd == fa + 1 && fs == fa + 2 && fr == fa + 3, "set-up: the four filter envelope faders are neighbours");
  CHECK(cutPeak > rest + 250, "Cutoff's label should follow the filter envelope's sweep (%d at rest, %d at the peak)", rest, cutPeak);
  CHECK(volPeak > rest + 100, "Volume's label should follow the output level");
  CHECK(stage[0] > 3 && stage[1] > 3 && stage[2] > 3 && stage[3] == 0,
        "held note: Attack, then Decay, then Sustain should be the brightest envelope label in turn (%d, %d, %d, %d frames)", stage[0], stage[1],
        stage[2], stage[3]);
  midi({0x80, 60, 0});
  play(0.15, cut);
  int lit = 0;
  for (int k = 1; k < 4; ++k)
    if (bright(fa + k) > bright(fa + lit)) lit = k;
  CHECK(lit == 3, "after the key is let go, Release should be the brightest (it was fader %d)", lit);
  CHECK(warm, "every dance colour should be an amber: red at least green, green at least blue");
  play(4.0, cut);
  CHECK(bright(cut) <= rest + 5, "once the note has died the labels should be back at rest");

  section("lights: the arp flashes on its steps; a loaded patch's guide lights take the panel back");
  setSel("arp_mode", fw::ARP_UP);
  setPot("arp_tempo", 0.3);
  midi({0x90, 48, 100, 0x90, 55, 100});
  int flashes = 0, wasBright = 0;
  for (int i = 0; i < 90; ++i) {  // three seconds: about 20 steps at 100 BPM
    renderNew(1.0 / 30);
    fw::tick(33);
    int now = bright(tempo) > rest + 150;
    flashes += now && !wasBright;
    wasBright = now;
  }
  CHECK(flashes >= 8, "Tempo's label should flash with the arp's steps (%d flashes in three seconds)", flashes);
  midi({0x80, 48, 0, 0x80, 55, 0});
  setSel("arp_mode", fw::ARP_OFF);
  play(2.0, cut);
  savePatchTo(1);
  while (fw::gUi.mode() != fw::MB_NONE) press("btn_patch");
  setPot("filter_cutoff", 0.9);
  fw::patchLoad(1);
  midi({0x90, 60, 110});
  play(0.4, cut);
  CHECK((fw::ledColour(cut) & 255) > 200 && ((fw::ledColour(cut) >> 16) & 255) < 120, "a control off the loaded patch shows its guide colour (blue: turn down), not the dance (%06x)", fw::ledColour(cut));
  CHECK(fw::ledColour(rate) == fw::kLedIdle, "and the dance waits while the guide has the panel");
  midi({0x80, 60, 0});
}

// ---------------------------------------- per-voice LFO, ensemble, compressor, duck, sub pitch

// The level in each window of `ms` milliseconds.
static std::vector<double> levels(const std::vector<float>& x, double ms) {
  size_t win = (size_t)(ms * 0.001 * gSr);
  std::vector<double> out;
  for (size_t at = 0; at + win <= x.size(); at += win) {
    double sum = 0;
    for (size_t i = at; i < at + win; ++i) sum += (double)x[i] * x[i];
    out.push_back(std::sqrt(sum / (double)win));
  }
  return out;
}

static void testNewSoundFeatures() {
  section("LFO sync VOICE: every voice has its own LFO, so a chord no longer moves as one");
  double swing[3];
  for (int mode : {(int)fw::LFOSYNC_FREE, (int)fw::LFOSYNC_VOICE}) {
    boot(48000);
    cleanTone(fw::WAVE_SIN);
    setSel("lfo_destination", fw::LFO_AMP);
    setSel("lfo_shape", fw::LFOS_TRI);
    setPot("lfo_rate", 0.6);  // about 2.3 Hz
    setPot("lfo_depth", 1.0);
    setSel("lfo_sync", mode);
    for (int k = 0; k < 8; ++k) midi({0x90, 48 + k * 5, 100});
    renderNew(1.0);
    std::vector<double> lv = levels(renderNew(4.0), 40);
    double lo = 1e9, hi = 0;
    for (double v : lv) {
      lo = std::min(lo, v);
      hi = std::max(hi, v);
    }
    swing[mode] = lo / hi;  // how far the whole chord dips: near 0 = all the way, near 1 = hardly at all
  }
  CHECK(swing[fw::LFOSYNC_FREE] < 0.2, "FREE: with one shared LFO the chord's level should swing nearly to nothing (%.2f)", swing[fw::LFOSYNC_FREE]);
  CHECK(swing[fw::LFOSYNC_VOICE] > swing[fw::LFOSYNC_FREE] + 0.3,
        "VOICE: with the voices out of step the chord as a whole should dip far less (%.2f against %.2f)", swing[fw::LFOSYNC_VOICE], swing[fw::LFOSYNC_FREE]);
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  setSel("lfo_destination", fw::LFO_AMP);
  setSel("lfo_shape", fw::LFOS_TRI);
  setPot("lfo_rate", 0.6);
  setPot("lfo_depth", 1.0);
  setSel("lfo_sync", fw::LFOSYNC_VOICE);
  midi({0x90, 60, 100});
  renderNew(0.5);
  std::vector<double> one = levels(renderNew(3.0), 40);
  CHECK(*std::min_element(one.begin(), one.end()) < 0.2 * *std::max_element(one.begin(), one.end()), "VOICE: a single note still gets its full tremolo");

  section("ensemble: a second kind of chorus, on the Type switch; exactly off at zero either way");
  std::vector<float> take[3], right[3];
  for (int kind = 0; kind < 3; ++kind) {  // 0 = chorus knob at zero (type set to ensemble), 1 = chorus, 2 = ensemble
    boot(48000);
    setSel("chorus_type", kind == 1 ? fw::CHORUS_PLAIN : fw::CHORUS_ENSEMBLE);
    setPot("chorus", kind == 0 ? 0.0 : 0.8);
    renderNew(0.05);
    take[kind] = phrase();
    right[kind] = gRight;
  }
  boot(48000);
  renderNew(0.05);
  std::vector<float> dry = phrase();
  CHECK(rmsDiff(take[0], dry) == 0.0, "with Chorus at zero the Type switch must change nothing");
  CHECK(rmsDiff(take[1], dry) > 1e-3 && rmsDiff(take[2], dry) > 1e-3, "both kinds should be heard when turned up");
  CHECK(rmsDiff(take[1], take[2]) > 1e-3, "and the ensemble should not sound like the chorus (difference %.5f)", rmsDiff(take[1], take[2]));
  CHECK(rmsDiff(take[2], right[2]) > 1e-3, "the ensemble should be stereo");
  bool finite = true;
  for (float v : take[2]) finite = finite && std::isfinite(v);
  CHECK(finite && peakOf(take[2]) <= 1.0, "ensemble output in range");

  section("compressor: one note and a full chord come out nearer the same level; exactly off at zero");
  double ratio[2], quiet[2];
  for (int on = 0; on < 2; ++on) {
    boot(48000);
    setPot("aenv_sustain", 1.0);
    setPot("aenv_velocity", 1.0);
    if (on) setStep(fw::P_COMPRESS, 100);
    midi({0x90, 60, 127});
    renderNew(0.5);
    double single = rmsOf(renderNew(1.0), 0.0, 1.0);
    for (int k = 1; k < 8; ++k) midi({0x90, 60 + k * 3, 127});
    renderNew(0.5);
    double chord = rmsOf(renderNew(1.0), 0.0, 1.0);
    ratio[on] = chord / single;
    midi({0xB0, 123, 0});
    renderNew(3.0);
    midi({0x90, 60, 30});  // a soft note
    renderNew(0.5);
    quiet[on] = rmsOf(renderNew(1.0), 0.0, 1.0);
  }
  CHECK(ratio[1] < 0.75 * ratio[0], "a chord should be less far above a single note with the compressor on (%.2f times against %.2f)", ratio[1], ratio[0]);
  CHECK(quiet[1] > 1.5 * quiet[0], "and a soft note should be lifted (%.4f against %.4f)", quiet[1], quiet[0]);
  boot(48000);
  renderNew(0.05);
  std::vector<float> plain = phrase();
  boot(48000);
  setStep(fw::P_COMPRESS, 40);
  setStep(fw::P_COMPRESS, 0);
  renderNew(0.05);
  CHECK(rmsDiff(plain, phrase()) == 0.0, "at zero the compressor must leave the sound exactly as it was");

  section("duck: the level dips once a beat at the arp's Tempo, and lets a note's attack through");
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  setPot("aenv_sustain", 1.0);
  setPot("arp_tempo", 0.4);  // 120 BPM: a beat is half a second
  setStep(fw::P_DUCK, 100);
  renderNew(0.2);
  midi({0x90, 57, 110});
  std::vector<float> held = renderNew(4.0);
  std::vector<double> lv = levels(held, 10);
  double top = *std::max_element(lv.begin() + 5, lv.end());
  std::vector<double> dips;
  for (size_t i = 1; i + 1 < lv.size(); ++i)
    if (lv[i] < 0.45 * top && lv[i] <= lv[i - 1] && lv[i] < lv[i + 1] && (dips.empty() || (double)i * 0.01 - dips.back() > 0.2)) dips.push_back((double)i * 0.01);
  CHECK(dips.size() == 8, "four seconds at 120 BPM should have eight dips (%zu found)", dips.size());
  double gapSum = 0;
  for (size_t i = 1; i < dips.size(); ++i) gapSum += dips[i] - dips[i - 1];
  double beat = dips.size() > 1 ? gapSum / (double)(dips.size() - 1) : 0;
  CHECK(std::fabs(beat - 0.5) < 0.02, "the dips should be one beat apart: %.3f s, expected 0.500", beat);
  CHECK(lv[0] > 0.5 * top, "the first hundredth of a second of the note should come through before the dip (%.2f of full)", lv[0] / top);
  CHECK(dips[0] < 0.08, "and the dip should follow straight after (first dip at %.2f s)", dips.empty() ? -1.0 : dips[0]);
  // with the arp running, the beat is counted from its steps: four steps to a dip
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  setPot("aenv_sustain", 1.0);
  setPot("arp_tempo", 0.4);
  setPot("arp_gate", 1.0);
  setSel("arp_mode", fw::ARP_UP);
  setStep(fw::P_DUCK, 100);
  midi({0x90, 57, 110});
  renderNew(1.0);
  for (int sweep = 0; sweep < 60; ++sweep) {  // tempo wound up and down: the dips must stay on the arp's beat
    setPot("arp_tempo", 0.3 + 0.4 * std::fabs(std::sin(sweep * 0.3)));
    renderNew(0.1);
  }
  setPot("arp_tempo", 0.4);
  renderNew(1.5);
  uint32_t stepsBefore = fw::gSynth.arpStepCount();
  std::vector<float> run = renderNew(2.0);
  CHECK(fw::gSynth.arpStepCount() - stepsBefore == 16, "set-up: two seconds at 120 BPM is sixteen steps");
  lv = levels(run, 5);
  top = *std::max_element(lv.begin(), lv.end());
  dips.clear();
  for (size_t i = 2; i + 1 < lv.size(); ++i)
    if (lv[i] < 0.45 * top && lv[i] <= lv[i - 1] && lv[i] < lv[i + 1] && (dips.empty() || (double)i * 0.005 - dips.back() > 0.2)) dips.push_back((double)i * 0.005);
  CHECK(dips.size() == 4, "sixteen steps should carry four dips (%zu found)", dips.size());

  section("sub pitch: two octaves or one below oscillator 1, a fifth above, or an octave above; and its detune");
  static const double kWant[4] = {0.25, 0.5, 1.4983, 2.0};
  for (int iv = 0; iv < 4; ++iv) {
    boot(48000);
    cleanTone(fw::WAVE_SAW);
    setPot("mix_osc1", 0.0);
    setPot("mix_sub", 0.9);
    setPot("filter_cutoff", 0.55);  // leaves little but the fundamental
    setSel("sub_pitch", iv);
    midi({0x90, 69, 100});
    std::vector<float> x = renderNew(1.0);
    double hz = measureFreq(x, (size_t)(0.3 * gSr), x.size());
    CHECK(std::fabs(hz / (440.0 * kWant[iv]) - 1.0) < 0.003, "sub pitch stop %d: %.2f Hz, expected %.2f", iv, hz, 440.0 * kWant[iv]);
  }
  boot(48000);
  cleanTone(fw::WAVE_SAW);
  setPot("mix_osc1", 0.0);
  setPot("mix_sub", 0.9);
  setPot("filter_cutoff", 0.55);
  setStep(fw::P_SUB_DETUNE, 50);  // +25 cents
  midi({0x90, 69, 100});
  std::vector<float> det = renderNew(1.0);
  double cents = 1200.0 * std::log2(measureFreq(det, (size_t)(0.3 * gSr), det.size()) / 220.0);
  CHECK(std::fabs(cents - 25.0) < 2.0, "sub detune at the top should be +25 cents (measured %.1f)", cents);

  section("older saved data: a chorus set on its old encoder page and an old sub setting come through unchanged in meaning");
  {
    // state as the build before this one wrote it: version 4, 62 parameters, 40 panel controls
    const int oldParams = fw::P_CHORUS_TYPE, oldPanel = 40;
    std::vector<uint8_t> blob = {'S', 'Y', 'N', 'T', 4, (uint8_t)oldParams};
    uint16_t sum = 0;
    for (int i = 0; i < oldParams; ++i) {
      uint16_t v = fw::kParams[i].def;
      if (i == fw::P_CHORUS) v = 60;      // 60% on the old 0..100 encoder
      if (i == fw::P_SUB_OCTAVE) v = 1;   // old meaning: two octaves down
      if (i == fw::P_LFO_SYNC) v = 1;     // KEY
      blob.push_back((uint8_t)(v & 0xFF));
      blob.push_back((uint8_t)(v >> 8));
      sum = (uint16_t)(sum * 31u + v + 1u);
    }
    size_t extAt = blob.size();
    blob.push_back((uint8_t)oldPanel);
    for (int c = 0; c < oldPanel; ++c) {
      uint16_t v = fw::kParams[fw::kPanelParams[c]].def;
      for (uint8_t b : {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8), (uint8_t)(v & 0xFF), (uint8_t)(v >> 8), (uint8_t)0}) blob.push_back(b);
    }
    for (uint8_t b : {(uint8_t)0, (uint8_t)2, (uint8_t)0}) blob.push_back(b);  // no slot, guide on, not edited
    for (char ch : std::string("OLDSTATE")) blob.push_back((uint8_t)ch);
    for (uint8_t b : {(uint8_t)0, (uint8_t)0}) blob.push_back(b);              // palette, lights
    for (size_t i = extAt; i < blob.size(); ++i) sum = (uint16_t)(sum * 31u + blob[i] + 1u);
    blob.push_back((uint8_t)(sum & 0xFF));
    blob.push_back((uint8_t)(sum >> 8));
    std::memcpy(gStore, blob.data(), blob.size());
    gStoreLen = (uint32_t)blob.size();
    std::memset(gBankLen, 0, sizeof gBankLen);
    fw::init(48000.0f);
    CHECK(!std::strcmp(fw::gPatch.name(), "OLDSTATE"), "set-up: the old state should load (name '%s')", fw::gPatch.name());
    CHECK(fw::gParams.val[fw::P_CHORUS] == 60 * 4095 / 100, "an old chorus of 60%% should still be 60%% (pot value %d)", (int)fw::gParams.val[fw::P_CHORUS]);
    CHECK(fw::gParams.step(fw::P_SUB_OCTAVE) == fw::SUB_2_OCT_DOWN, "an old 'two octaves down' should still be two octaves down");
    CHECK(fw::gParams.step(fw::P_LFO_SYNC) == fw::LFOSYNC_KEY, "an old LFO SYNC of KEY should still be KEY");
    int chorusKnob = ctl("chorus");
    CHECK(fw::controlGet(chorusKnob) == fw::gParams.val[fw::P_CHORUS] && !fw::gPatch.following(chorusKnob),
          "the new Chorus knob should start where the old setting was, live");
    CHECK(fw::gParams.val[fw::P_COMPRESS] == 0 && fw::gParams.val[fw::P_DUCK] == 0 && fw::gParams.step(fw::P_SUB_DETUNE) == 25,
          "the new values should be at their defaults: compressor and duck off, sub in tune");

    // and a patch stored by that build
    uint8_t patch[fw::kPatchSize] = {'P', 'A', 'T', 1, (uint8_t)oldParams, 'O', 'L', 'D', 'P', 'A', 'T', 'C', 'H'};
    sum = 0;
    for (int i = 0; i < fw::kPatchNameLen; ++i) sum = (uint16_t)(sum * 31u + patch[5 + i] + 1u);
    for (int i = 0; i < oldParams; ++i) {
      uint16_t v = fw::kParams[i].def;
      if (i == fw::P_CHORUS) v = 35;
      if (i == fw::P_SUB_OCTAVE) v = 0;   // old meaning: one octave down
      patch[13 + i * 2] = (uint8_t)(v & 0xFF);
      patch[14 + i * 2] = (uint8_t)(v >> 8);
      sum = (uint16_t)(sum * 31u + v + 1u);
    }
    patch[13 + oldParams * 2] = (uint8_t)(sum & 0xFF);
    patch[14 + oldParams * 2] = (uint8_t)(sum >> 8);
    std::memcpy(gBank[3], patch, 15 + oldParams * 2);
    gBankLen[3] = 15 + oldParams * 2;
    CHECK(fw::patchLoad(3), "set-up: the old patch should load");
    CHECK(fw::gParams.val[fw::P_CHORUS] == 35 * 4095 / 100 && fw::gParams.step(fw::P_SUB_OCTAVE) == fw::SUB_1_OCT_DOWN,
          "an old patch's chorus and sub should keep their meaning (chorus %d, sub %d)", (int)fw::gParams.val[fw::P_CHORUS], fw::gParams.step(fw::P_SUB_OCTAVE));
    CHECK(fw::gPatch.following(chorusKnob), "and the Chorus knob, which is elsewhere, should follow the patch");
  }
}

// ------------------------------------------------- connections: clock, thru, channel, audio input

static std::vector<int> drainMidi(int port) {
  std::vector<int> v;
  uint8_t b[64];
  for (uint32_t n = fw::midiOutRead(port, b, sizeof b); n > 0; n = fw::midiOutRead(port, b, sizeof b)) v.insert(v.end(), b, b + n);
  return v;
}

// A clock sender. Renders `sec` seconds in small blocks while feeding MIDI
// clock at `bpm` into `port`, each tick up to `jitterMs` early or late.
// Returns when each arp step happened, in seconds from the first call since clockBegin().
static double gClockT = 0, gClockIdeal = 0, gClockSend = 0;
static uint32_t gClockRng = 1;
static int gClockTicks = 0, gClockBeats = 0;
static std::vector<float> gClockAudio;
static void clockBegin() {
  gClockT = gClockIdeal = gClockSend = 0;
  gClockRng = 0x1234567u;
  gClockTicks = gClockBeats = 0;
  gClockAudio.clear();
}
static std::vector<double> runClock(double sec, double bpm, int port = fw::PORT_MIDI, double jitterMs = 0) {
  const int block = 16;
  std::vector<double> steps;
  double period = bpm > 0 ? gSr * 60.0 / (bpm * 24.0) : 0;
  double end = gClockT + sec * gSr;
  float l[fw::kMaxBlock], r[fw::kMaxBlock];
  float beatBefore = fw::gSynth.beatNow();
  while (gClockT < end) {
    while (bpm > 0 && gClockSend <= gClockT) {
      fw::midiByte(0xF8, port);
      ++gClockTicks;
      gClockIdeal += period;
      gClockRng = gClockRng * 1664525u + 1013904223u;
      gClockSend = gClockIdeal + ((double)(gClockRng >> 8) / 16777216.0 * 2.0 - 1.0) * jitterMs * 0.001 * gSr;
    }
    uint32_t before = fw::gSynth.arpStepCount();
    fw::process(l, r, block);
    gClockAudio.insert(gClockAudio.end(), l, l + block);
    if (fw::gSynth.arpStepCount() != before) steps.push_back(gClockT / gSr);
    float beat = fw::gSynth.beatNow();
    if (beat < beatBefore - 0.5f) ++gClockBeats;
    beatBefore = beat;
    gClockT += block;
  }
  if (bpm <= 0) gClockIdeal = gClockSend = gClockT;  // a sender switched back on starts with a tick
  return steps;
}

// The largest and the root-mean-square difference of the gaps between steps from what they should be, in milliseconds.
static void gapError(const std::vector<double>& steps, double expectSec, double& worstMs, double& rmsMs, size_t skip = 2) {
  worstMs = rmsMs = 0;
  int n = 0;
  for (size_t i = skip + 1; i < steps.size(); ++i) {
    double e = ((steps[i] - steps[i - 1]) - expectSec) * 1000.0;
    worstMs = std::max(worstMs, std::fabs(e));
    rmsMs += e * e;
    ++n;
  }
  rmsMs = n ? std::sqrt(rmsMs / n) : 0;
}

static void testClock() {
  section("clock: the arp follows MIDI clock at the MIDI jack, whatever the Tempo knob says");
  boot(48000);
  arpTone();
  CHECK(fw::gSynth.clockState() == fw::CLK_INTERNAL, "out of the box the synth keeps its own time");
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  setPot("arp_tempo", 0.9);  // far from the clock's tempo
  setSel("arp_mode", fw::ARP_UP);
  midi({0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100});
  clockBegin();
  midi({0xFA});
  auto steps = runClock(6.0, 100);
  double worst, rms;
  gapError(steps, 0.15, worst, rms);
  CHECK(steps.size() >= 40 && steps.size() <= 41, "six seconds at 100 BPM is 40 sixteenth notes, got %zu", steps.size());
  CHECK(!steps.empty() && steps[0] < 0.002, "the first step should fall on the first tick after start (%.4f s)", steps.empty() ? -1 : steps[0]);
  CHECK(worst < 0.7, "steps should be 150 ms apart; the worst was %.2f ms out", worst);
  CHECK(fw::gSynth.clockState() == fw::CLK_LOCKED, "the clock should be locked");
  CHECK(std::fabs(fw::gSynth.tempoBpm() - 100.0) < 0.3, "the measured tempo should be 100 BPM, got %.2f", fw::gSynth.tempoBpm());
  std::printf("    steady clock at 100 BPM: worst step %.2f ms out, tempo read as %.2f\n", worst, fw::gSynth.tempoBpm());

  section("clock: ticks that arrive early and late (as from a computer) still give even steps");
  steps = runClock(8.0, 100, fw::PORT_MIDI, 4.0);  // every tick up to 4 ms out
  gapError(steps, 0.15, worst, rms);
  CHECK(steps.size() >= 53 && steps.size() <= 54, "eight more seconds is 53 steps, got %zu", steps.size());
  CHECK(rms < 1.5 && worst < 4.0, "steps wander %.2f ms (worst %.2f) with ticks up to 4 ms out", rms, worst);
  CHECK(std::fabs(fw::gSynth.tempoBpm() - 100.0) < 1.5, "the tempo should still read about 100, got %.2f", fw::gSynth.tempoBpm());
  std::printf("    ticks up to 4 ms early or late: steps wander %.2f ms on average, %.2f at worst\n", rms, worst);

  section("clock: a change of tempo is followed");
  runClock(3.0, 140);
  CHECK(std::fabs(fw::gSynth.tempoBpm() - 140.0) < 1.0, "after three seconds at 140 the tempo should read 140, got %.2f", fw::gSynth.tempoBpm());
  steps = runClock(3.0, 140);
  gapError(steps, 60.0 / 140.0 / 4.0, worst, rms);
  CHECK(steps.size() >= 27 && steps.size() <= 29 && worst < 1.0, "three seconds at 140 is 28 steps (got %zu, worst %.2f ms out)", steps.size(), worst);

  section("clock: stop holds the arp, continue carries on, start begins the pattern again");
  midi({0xFC});
  steps = runClock(1.5, 140);  // many senders keep the ticks coming while stopped
  CHECK(steps.empty(), "no steps while stopped, got %zu", steps.size());
  CHECK(fw::gSynth.clockState() == fw::CLK_STOPPED, "the clock should read stopped");
  CHECK(fw::gSynth.activeVoices() == 0, "the last note should have died away (%d voices)", fw::gSynth.activeVoices());
  midi({0xFB});
  steps = runClock(1.5, 140);
  CHECK(steps.size() >= 13 && steps.size() <= 15, "continue: 1.5 seconds at 140 is 14 steps, got %zu", steps.size());
  midi({0xFC});
  runClock(0.3, 140);
  midi({0xFA});
  size_t from = gClockAudio.size();
  steps = runClock(0.1, 140);
  CHECK(steps.size() == 1 && steps[0] - (gClockT / gSr - 0.1) < 0.02, "start: the first step falls on the first tick");
  double hz = measureFreq(gClockAudio, from + 480, from + 4800);
  CHECK(std::fabs(hz - 261.63) < 4.0, "start: the pattern begins again from its first note (C4), got %.1f Hz", hz);

  section("clock: a song position moves the beat to where the sender's playhead is");
  boot(48000);
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  clockBegin();
  midi({0xFA});
  runClock(1.0, 120);
  midi({0xFC, 0xF2, 2, 0, 0xFB});  // stop, move to two sixteenth notes in (half a beat), continue
  runClock(0.01, 120);
  CHECK(std::fabs(fw::gSynth.beatNow() - 0.5) < 0.06, "after moving to half a beat in, the beat should read 0.5, got %.2f", fw::gSynth.beatNow());

  section("clock: with no clock arriving, the Tempo knob is used; clock at the other input is ignored");
  boot(48000);
  arpTone();
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  setSel("arp_mode", fw::ARP_UP);
  midi({0x90, 60, 100});
  clockBegin();
  steps = runClock(2.0, 0);
  CHECK(fw::gSynth.clockState() == fw::CLK_WAITING, "no clock yet: waiting");
  CHECK(steps.size() >= 16 && steps.size() <= 17, "the arp runs at the knob's 120 BPM meanwhile (%zu steps in 2 s)", steps.size());
  steps = runClock(2.0, 90, fw::PORT_USB);
  CHECK(fw::gSynth.clockState() == fw::CLK_WAITING && steps.size() >= 15 && steps.size() <= 17,
        "clock at USB while set to MIDI is not followed (%zu steps)", steps.size());
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_USB);
  runClock(0.1, 0);
  steps = runClock(3.0, 90, fw::PORT_USB);
  CHECK(fw::gSynth.clockState() == fw::CLK_LOCKED && std::fabs(fw::gSynth.tempoBpm() - 90.0) < 0.5,
        "set to USB, clock at USB is followed (tempo %.2f)", fw::gSynth.tempoBpm());
  CHECK(steps.size() >= 17 && steps.size() <= 19, "three seconds at 90 is 18 steps, got %zu", steps.size());

  section("clock: if the clock goes away the arp carries on from the Tempo knob");
  steps = runClock(2.5, 0);
  CHECK(fw::gSynth.clockState() == fw::CLK_WAITING, "half a second without a tick: waiting again");
  CHECK(steps.size() >= 17 && steps.size() <= 21, "the arp keeps playing (%zu steps in 2.5 s)", steps.size());
  steps = runClock(3.0, 90, fw::PORT_USB);
  CHECK(fw::gSynth.clockState() == fw::CLK_LOCKED && steps.size() >= 17 && steps.size() <= 19, "and locks again when the clock returns (%zu steps)", steps.size());

  section("clock: a key pressed a little late still gets the step it was aimed at");
  boot(48000);
  arpTone();
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  setSel("arp_mode", fw::ARP_UP);
  clockBegin();
  midi({0xFA});
  runClock(1.0 + 0.02, 120);  // 20 ms after a step (steps are 125 ms apart)
  midi({0x90, 60, 100});
  steps = runClock(0.5, 120);
  CHECK(!steps.empty() && steps[0] - 1.02 < 0.003, "20 ms late: the note should sound at once, not wait for the next step");
  CHECK(steps.size() >= 2 && std::fabs(steps[1] - 1.125) < 0.004, "and the next step is back on the clock's grid (%.4f s)", steps.size() >= 2 ? steps[1] : -1);
  midi({0x80, 60, 0});
  runClock(0.5 + 0.06, 120);  // now 60 ms after a step: nearer the next one
  midi({0x90, 60, 100});
  steps = runClock(0.5, 120);
  CHECK(!steps.empty() && std::fabs(steps[0] - 2.125) < 0.004, "60 ms late: the note waits for the next step (%.4f s)", steps.empty() ? -1 : steps[0]);

  section("clock: the duck keeps time with the clock too");
  boot(48000);
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  clockBegin();
  midi({0xFA});
  runClock(8.0, 90);
  CHECK(gClockBeats >= 11 && gClockBeats <= 12, "eight seconds at 90 BPM is 12 beats, the duck counted %d", gClockBeats);

  section("clock: the Tempo knob's picture says when a clock is setting the tempo instead");
  boot(48000);
  setPot("arp_tempo", 0.3);
  setSel("arp_mode", fw::ARP_UP);
  setPot("arp_tempo", 0.31);
  fw::tick(33);
  auto internalPicture = grabFrame();
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  clockBegin();
  runClock(2.0, 150);
  setPot("arp_tempo", 0.3);
  setPot("arp_tempo", 0.31);
  fw::tick(33);
  CHECK(grabFrame() != internalPicture, "the Tempo picture should differ while a clock sets the tempo");
  saveFrame("popup_tempo_clock");
  midi({0xFC});
  runClock(0.2, 150);
  setPot("arp_tempo", 0.3);
  fw::tick(33);
  saveFrame("popup_tempo_stopped");
}

static void testMidiOutAndChannel() {
  section("MIDI: with Thru off nothing is passed on");
  boot(48000);
  midi({0x90, 60, 100, 0x80, 60, 0});
  CHECK(drainMidi(fw::PORT_MIDI).empty() && drainMidi(fw::PORT_USB).empty(), "nothing should leave with Thru off");

  section("MIDI: Thru passes whole messages on; USB is never sent its own messages back");
  fw::settingSet(fw::SET_THRU, 1);
  midi({0x90, 60, 100, 62, 90, 0x80, 60, 0});  // the second note uses running status
  std::vector<int> expect = {0x90, 60, 100, 0x90, 62, 90, 0x80, 60, 0};
  CHECK(drainMidi(fw::PORT_MIDI) == expect, "the MIDI out jack should carry what came in at the MIDI jack, every message complete");
  CHECK(drainMidi(fw::PORT_USB) == expect, "and so should USB");
  for (int b : {0x91, 50, 64}) fw::midiByte((uint8_t)b, fw::PORT_USB);
  CHECK(drainMidi(fw::PORT_MIDI) == std::vector<int>({0x91, 50, 64}), "what arrives by USB goes to the MIDI out jack");
  CHECK(drainMidi(fw::PORT_USB).empty(), "but not back to USB");
  midi({0xF0, 1, 2, 3, 0xF7});
  CHECK(drainMidi(fw::PORT_MIDI).empty(), "system-exclusive messages are not passed on");
  for (int i = 0; i < 400; ++i) midi({0x90, 60, 100});  // far more than the output can hold, with nothing reading it
  auto flood = drainMidi(fw::PORT_MIDI);
  bool whole = flood.size() % 3 == 0 && !flood.empty();
  for (size_t i = 0; i + 2 < flood.size(); i += 3) whole = whole && flood[i] == 0x90 && flood[i + 1] == 60 && flood[i + 2] == 100;
  CHECK(whole, "an output that is not being read drops whole messages, never part of one (%zu bytes kept)", flood.size());
  drainMidi(fw::PORT_USB);
  midi({0xB0, 123, 0});
  renderNew(0.5);

  section("MIDI: the channel setting picks which channel the synth plays; every channel is still passed on");
  boot(48000);
  fw::settingSet(fw::SET_THRU, 1);
  fw::settingSet(fw::SET_CHANNEL, 3);
  midi({0x90, 60, 100});
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() == 0, "a note on channel 1 should not play when the synth is on channel 3");
  CHECK(drainMidi(fw::PORT_MIDI) == std::vector<int>({0x90, 60, 100}), "but it is passed on");
  midi({0x92, 60, 100});
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() == 1, "a note on channel 3 should play");
  midi({0x82, 60, 0});
  int before = fw::gParams.val[fw::P_CUTOFF];
  midi({0xB0, fw::kParams[fw::P_CUTOFF].cc, 5});
  CHECK(fw::gParams.val[fw::P_CUTOFF] == before, "a controller on another channel should not move the synth's parameters");
  fw::settingSet(fw::SET_CHANNEL, 0);
  midi({0x95, 64, 100});
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() >= 1, "set to ALL, any channel plays");

  section("MIDI: Clock Out sends the synth's own clock, 24 ticks to a beat at the Tempo knob's setting");
  boot(48000);
  renderNew(1.0);
  CHECK(drainMidi(fw::PORT_MIDI).empty(), "no clock is sent with Clock Out off");
  fw::settingSet(fw::SET_CLOCK_OUT, 1);
  std::vector<int> out, outUsb;
  for (int i = 0; i < 100; ++i) {  // ten seconds; the knob's default is 120 BPM
    renderNew(0.1);
    for (int b : drainMidi(fw::PORT_MIDI)) out.push_back(b);
    for (int b : drainMidi(fw::PORT_USB)) outUsb.push_back(b);
  }
  bool allClock = true;
  for (int b : out) allClock = allClock && b == 0xF8;
  CHECK(allClock && out.size() >= 479 && out.size() <= 481, "ten seconds at 120 BPM is 480 ticks, got %zu", out.size());
  CHECK(outUsb == out, "and the same at USB");

  section("MIDI: following a clock, Clock Out passes that clock on (but never back to USB)");
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  renderNew(0.1);
  drainMidi(fw::PORT_MIDI);
  drainMidi(fw::PORT_USB);
  clockBegin();
  midi({0xFA});
  runClock(1.0, 120);
  midi({0xFC});
  drainMidi(fw::PORT_MIDI).swap(out);
  CHECK((int)out.size() == gClockTicks + 2 && out.front() == 0xFA && out.back() == 0xFC, "start, %d ticks and stop should be passed on (%zu bytes)",
        gClockTicks, out.size());
  CHECK(drainMidi(fw::PORT_USB).size() == out.size(), "to USB as well, when the clock came in at the MIDI jack");
  fw::midiByte(0xF8, fw::PORT_USB);
  CHECK(drainMidi(fw::PORT_MIDI).empty(), "clock at an input that is not being followed is dropped: two clocks never go out");
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_USB);
  renderNew(0.1);
  for (int i = 0; i < 5; ++i) fw::midiByte(0xF8, fw::PORT_USB);
  CHECK(drainMidi(fw::PORT_MIDI).size() == 5 && drainMidi(fw::PORT_USB).empty(), "USB clock goes to the MIDI out jack only");

  section("settings: they survive a restart, are not part of a patch, and older saved state gets the defaults");
  boot(48000);
  for (int i = 0; i < fw::SET_COUNT; ++i)
    CHECK(fw::setting(i) == fw::kSettings[i].def, "setting %s should start at its default", fw::kSettings[i].name);
  savePatchTo(0);
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_USB);
  fw::settingSet(fw::SET_CHANNEL, 7);
  fw::settingSet(fw::SET_THRU, 1);
  fw::settingSet(fw::SET_CLOCK_OUT, 1);
  fw::settingSet(fw::SET_INPUT_SOURCE, fw::INSRC_MIC);
  fw::settingSet(fw::SET_INPUT_GAIN, 24);
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_FX);
  const int want[fw::SET_COUNT] = {fw::CLOCK_USB, 7, 1, 1, fw::INSRC_MIC, 24, fw::INPUT_FX};
  CHECK(fw::patchLoad(0), "the patch should load");
  bool kept = true;
  for (int i = 0; i < fw::SET_COUNT; ++i) kept = kept && fw::setting(i) == want[i];
  CHECK(kept, "loading a patch should not change any setting");
  fw::tick(1200);
  CHECK(gStoreLen == fw::kStateSize, "the settings should be saved with the state (%u bytes)", gStoreLen);
  boot(48000, true);
  kept = true;
  for (int i = 0; i < fw::SET_COUNT; ++i) kept = kept && fw::setting(i) == want[i];
  CHECK(kept, "every setting should survive a restart");
  CHECK(fw::inputGainDb() == 24 && fw::inputSource() == fw::INSRC_MIC, "the hardware is told the gain and the source");
  CHECK(!fw::settingSet(fw::SET_CHANNEL, 99) || fw::setting(fw::SET_CHANNEL) == 16, "a setting cannot go past its range");

  section("settings: the MIDI and Input pages set them with the encoders");
  boot(48000);
  for (int i = 0; i < 4; ++i) press("btn_patch");
  fw::tick(33);
  auto midiPage = grabFrame();
  saveFrame("mode_midi");
  fw::encoderTurn(ctl("enc1"), 1);
  fw::encoderTurn(ctl("enc2"), 3);
  press("enc3");
  press("enc4");
  CHECK(fw::setting(fw::SET_CLOCK) == fw::CLOCK_MIDI && fw::setting(fw::SET_CHANNEL) == 3 && fw::setting(fw::SET_THRU) == 1 &&
            fw::setting(fw::SET_CLOCK_OUT) == 1,
        "the four encoders set Clock, Channel, Thru and Clock Out");
  renderNew(0.05);
  fw::tick(33);
  CHECK(grabFrame() != midiPage, "the page should show the new values");
  CHECK(fw::gSynth.clockState() == fw::CLK_WAITING, "set to MIDI with no clock arriving: waiting");
  saveFrame("mode_midi_waiting");
  clockBegin();
  midi({0xFA});
  runClock(2.0, 128);
  fw::tick(33);
  saveFrame("mode_midi_locked");
  press("enc2");
  CHECK(fw::setting(fw::SET_CHANNEL) == 0, "pushing the Channel encoder goes back to ALL");
  press("btn_patch");
  fw::tick(33);
  saveFrame("mode_input");
  fw::encoderTurn(ctl("enc1"), 1);
  fw::encoderTurn(ctl("enc2"), 1);
  fw::tick(33);
  for (int i = 0; i < 11; ++i) {
    fw::encoderTurn(ctl("enc2"), 1);
    fw::tick(33);
  }
  fw::encoderTurn(ctl("enc3"), 2);
  CHECK(fw::setting(fw::SET_INPUT_SOURCE) == fw::INSRC_MIC && fw::setting(fw::SET_INPUT_GAIN) == 12 && fw::setting(fw::SET_INPUT_TO) == fw::INPUT_FX,
        "the encoders set Source, Gain and Send To (gain %d)", fw::setting(fw::SET_INPUT_GAIN));
  uint16_t params[fw::P_COUNT];
  for (int i = 0; i < fw::P_COUNT; ++i) params[i] = fw::gParams.val[i];
  fw::encoderTurn(ctl("enc4"), 3);
  press("enc4");
  bool untouched = true;
  for (int i = 0; i < fw::P_COUNT; ++i) untouched = untouched && params[i] == fw::gParams.val[i];
  CHECK(untouched, "the spare encoder on the Input page does nothing");
  press("btn_patch");
  CHECK(fw::gUi.mode() == fw::MB_NONE, "after the Input page the Patch button goes home");
}

// Renders with an audio input: gen(i) gives the left and right input for sample i since the call began.
template <typename Gen>
static std::vector<float> renderIn(double sec, Gen gen) {
  std::vector<float> out;
  gRight.clear();
  long n = std::lround(sec * gSr), at = 0;
  float l[fw::kMaxBlock], r[fw::kMaxBlock], il[fw::kMaxBlock], ir[fw::kMaxBlock];
  while (n > 0) {
    int len = n > fw::kMaxBlock ? fw::kMaxBlock : (int)n;
    for (int i = 0; i < len; ++i) {
      std::pair<float, float> x = gen(at + i);
      il[i] = x.first;
      ir[i] = x.second;
    }
    fw::process(il, ir, l, r, len);
    out.insert(out.end(), l, l + len);
    gRight.insert(gRight.end(), r, r + len);
    n -= len;
    at += len;
  }
  return out;
}

static void testAudioInput() {
  auto sine = [](double level, double hz) {
    return [=](long i) {
      float x = (float)(level * std::sin(2.0 * M_PI * hz * (double)i / gSr));
      return std::make_pair(x, x);
    };
  };
  auto leftOnly = [](long i) { return std::make_pair((float)(0.25 * std::sin(2.0 * M_PI * 330.0 * (double)i / gSr)), 0.0f); };

  section("input: with Send To off, an input changes nothing at all");
  boot(48000);
  midi({0x90, 60, 100, 0x90, 67, 100});
  auto plain = renderNew(1.0);
  auto plainRight = gRight;
  boot(48000);
  midi({0x90, 60, 100, 0x90, 67, 100});
  auto withInput = renderIn(1.0, sine(0.5, 440));
  CHECK(plain == withInput && plainRight == gRight, "the output should be bit for bit the same with an unused input connected");
  CHECK(std::fabs(fw::gSynth.inputLevel() - 0.5) < 0.02, "the input is still metered, so its gain can be set first (level %.3f)", fw::gSynth.inputLevel());

  section("input: MIX puts it straight on the output, stereo for a line input, both sides for a microphone");
  boot(48000);
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_MIX);
  auto out = renderIn(1.0, leftOnly);
  bool same = true;
  for (size_t i = 24000; i < out.size(); ++i) same = same && std::fabs(out[i] - leftOnly((long)i).first) < 1e-5;
  CHECK(same, "once faded in, the left output should be the left input exactly");
  CHECK(peakOf(gRight) < 1e-6, "a line input is stereo: nothing on the right when only the left is fed (%.5f)", peakOf(gRight));
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_OFF);
  renderIn(0.5, leftOnly);
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_MIX);
  out = renderIn(0.1, [](long) { return std::make_pair(0.5f, 0.5f); });  // an input that is already at a high level
  double step = std::fabs(out[0]);
  for (size_t i = 1; i < out.size(); ++i) step = std::max(step, (double)std::fabs(out[i] - out[i - 1]));
  CHECK(step < 0.004 && std::fabs(out.back() - 0.5) < 1e-4, "the input fades in rather than switching on (largest jump %.4f)", step);
  fw::settingSet(fw::SET_INPUT_SOURCE, fw::INSRC_MIC);
  out = renderIn(0.5, leftOnly);
  CHECK(rmsDiff(out, gRight) < 1e-6 && peakOf(gRight) > 0.2, "a microphone is one channel: it should come out on both sides");
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_OFF);
  out = renderIn(0.5, leftOnly);
  CHECK(peakOf(out, 12000) < 1e-6, "switched off again, it fades out to nothing (%.6f)", peakOf(out, 12000));

  section("input: FX sends it through the effects; MIX does not");
  auto burst = [](long i) {
    float x = i < 9600 ? (float)(0.4 * std::sin(2.0 * M_PI * 440.0 * (double)i / gSr)) : 0.0f;
    return std::make_pair(x, x);
  };
  boot(48000);
  setStep(fw::P_RVB_MIX, 60);
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_MIX);
  out = renderIn(1.0, burst);
  double tailMix = rmsOf(out, 0.4, 0.9);
  boot(48000);
  setStep(fw::P_RVB_MIX, 60);
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_FX);
  out = renderIn(1.0, burst);
  double tailFx = rmsOf(out, 0.4, 0.9);
  CHECK(tailMix < 1e-5, "MIX: nothing after the input stops (%.6f)", tailMix);
  CHECK(tailFx > 0.003, "FX: the reverb's tail carries on after the input stops (%.5f)", tailFx);
  CHECK(rmsOf(out, 0.05, 0.19) > 0.1, "FX: the input itself is heard");

  section("input: the light by the jack is dark, amber or red; red for long enough to see");
  boot(48000);
  renderIn(0.2, sine(0.0, 440));
  CHECK(fw::inputLight() == 0, "no signal: dark");
  renderIn(0.2, sine(0.05, 440));
  uint32_t dim = fw::inputLight();
  renderIn(0.2, sine(0.5, 440));
  uint32_t bright = fw::inputLight();
  CHECK(dim != 0 && bright == 0xFF9A00 && dim != bright, "a signal: dim amber when quiet, full amber at a healthy level (%06X, %06X)", dim, bright);
  CHECK(!fw::gSynth.inputClipped(), "half scale is not clipping");
  renderIn(0.05, sine(1.0, 440));
  CHECK(fw::gSynth.inputClipped() && fw::inputLight() == 0xE00000, "at the converter's limit: red");
  renderIn(0.2, sine(0.0, 440));
  CHECK(fw::inputLight() == 0xE00000, "still red 0.2 s later");
  renderIn(0.5, sine(0.0, 440));
  CHECK(!fw::gSynth.inputClipped() && fw::inputLight() != 0xE00000, "the red goes out within a second");
  renderIn(1.5, sine(0.0, 440));
  CHECK(fw::inputLight() == 0, "and the light is dark again once the input has been quiet");
  for (int i = 0; i < 5; ++i) press("btn_patch");
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_FX);
  renderIn(0.3, sine(0.3, 440));
  fw::tick(33);
  saveFrame("mode_input_level");
  auto level = grabFrame();
  renderIn(0.1, sine(1.0, 440));
  fw::tick(33);
  CHECK(grabFrame() != level, "the Input page shows the level and the clip");
  saveFrame("mode_input_clip");

  section("input: nonsense from the converter cannot upset the sound");
  boot(48000);
  setStep(fw::P_RVB_MIX, 60);
  setStep(fw::P_DLY_MIX, 60);
  fw::settingSet(fw::SET_INPUT_TO, fw::INPUT_FX);
  out = renderIn(0.5, [](long i) {
    float x = i % 7 == 0 ? std::numeric_limits<float>::quiet_NaN() : i % 5 == 0 ? 1e30f : i % 3 == 0 ? -std::numeric_limits<float>::infinity() : 0.1f;
    return std::make_pair(x, -x);
  });
  auto after = renderIn(2.0, sine(0.0, 440));
  bool finite = true;
  for (float x : out) finite = finite && std::isfinite(x) && std::fabs(x) <= 1.0f;
  for (float x : after) finite = finite && std::isfinite(x) && std::fabs(x) <= 1.0f;
  CHECK(finite, "the output should stay finite and in range");
  CHECK(rmsOf(after, 1.9, 2.0) < 0.02, "and die away afterwards (%.5f)", rmsOf(after, 1.9, 2.0));
}

// ------------------------------------------------- the vocoder

// A made-up voice: a buzz at `f0` Hz whose harmonics are strongest round
// three formants, which is what makes one vowel differ from another.
struct Vowel {
  double f0, formant[3];
  std::vector<double> amp;
  Vowel(double pitch, double f1, double f2, double f3, double peak = 0.3) : f0(pitch), formant{f1, f2, f3} {
    double sum = 0;
    for (int h = 1; h * f0 < 5000; ++h) {
      double a = 0;
      // (a real voice's formants get weaker as they get higher)
      for (double f : formant) a += formant[0] / f * std::exp(-std::pow((h * f0 - f) / (0.12 * f + 60.0), 2.0));
      amp.push_back(a + 0.002);
      sum += a + 0.002;
    }
    for (double& a : amp) a *= peak / sum * 1.6;
  }
  std::pair<float, float> operator()(long i) const {
    double x = 0, t = (double)i / gSr;
    for (size_t h = 0; h < amp.size(); ++h) x += amp[h] * std::sin(2.0 * M_PI * (double)(h + 1) * f0 * t);
    return std::make_pair((float)x, (float)x);
  }
};

// The energy of `x` between two frequencies, from `from` on.
static double bandEnergy(const std::vector<float>& x, size_t from, double loHz, double hiHz) {
  const size_t n = 16384;
  std::vector<double> m = spectrum(x, from, n);
  double e = 0;
  for (size_t i = (size_t)(loHz * n / gSr); i <= (size_t)(hiHz * n / gSr) && i < m.size(); ++i) e += m[i] * m[i];
  return e;
}

// A bright, steady sound for the vocoder to shape: saws with the filter wide open.
static void vocoderCarrier() {
  setSel("osc1_wave", fw::WAVE_SAW);
  setSel("osc2_wave", fw::WAVE_SAW);
  setPot("mix_osc2", 0.0);  // one oscillator: two would beat, and the level would come and go
  setPot("filter_cutoff", 1.0);
  setPot("filter_env_amount", 0.5);
  setPot("filter_resonance", 0.0);
  setPot("aenv_attack", 0.0);
  setPot("aenv_sustain", 1.0);
  setPot("aenv_release", 0.1);
  renderNew(0.05);
}

static void testVocoder() {
  const Vowel ah(120, 700, 1200, 2600), ee(120, 300, 2300, 3000);
  auto silence = [](long) { return std::make_pair(0.0f, 0.0f); };
  // an "s": hiss with nothing below about 4 kHz in it
  auto ess = [](long i) {
    static uint32_t rng = 1;
    static float last = 0, last2 = 0;
    if (i == 0) rng = 1, last = last2 = 0;
    rng = rng * 1664525u + 1013904223u;
    float white = (float)(int32_t)rng / 2147483648.0f;
    float high = white - 2.0f * last + last2;  // two differences: a steep high-pass
    last2 = last;
    last = white;
    return std::make_pair(0.08f * high, 0.08f * high);
  };

  section("vocoder: off, it changes nothing, whatever its other controls are set to");
  boot(48000);
  vocoderCarrier();
  midi({0x90, 57, 100});
  auto plain = renderIn(0.5, ah);
  boot(48000);
  vocoderCarrier();
  setStep(fw::P_VOC_FORMANT, 20);
  setStep(fw::P_VOC_CONSONANTS, 100);
  setStep(fw::P_VOC_SMOOTH, 90);
  midi({0x90, 57, 100});
  auto same = renderIn(0.5, ah);
  CHECK(plain == same, "with the amount at 0 the sound should be bit for bit what it is without a vocoder");

  section("vocoder: the notes are the synth's, the vowel is the voice's");
  setStep(fw::P_VOC_CONSONANTS, 0);
  setStep(fw::P_VOC_FORMANT, 12);
  setStep(fw::P_VOC_SMOOTH, 35);
  setStep(fw::P_VOCODER, 100);
  auto outAh = renderIn(1.0, ah);
  // the synth plays A3 (220 Hz), the voice buzzes at 120 Hz
  double synthPitch = bandEnergy(outAh, 20000, 652, 668) + bandEnergy(outAh, 20000, 872, 888);    // 3 x 220, 4 x 220
  double voicePitch = bandEnergy(outAh, 20000, 712, 728) + bandEnergy(outAh, 20000, 832, 848);    // 6 x 120, 7 x 120
  CHECK(synthPitch > 100 * voicePitch, "the pitch should be the synth's 220 Hz, not the voice's 120 (%.1f dB apart)", 10 * std::log10(synthPitch / voicePitch));
  auto outEe = renderIn(1.0, ee);
  saveSound("vocoder_ah", outAh);
  saveSound("vocoder_ee", outEe);
  // "ah" is strong round 700 to 1200 Hz and weak round 2300; "ee" is the other way about
  double ahMid = bandEnergy(outAh, 20000, 600, 1400), ahHigh = bandEnergy(outAh, 20000, 2000, 3400);
  double eeMid = bandEnergy(outEe, 20000, 600, 1400), eeHigh = bandEnergy(outEe, 20000, 2000, 3400);
  CHECK(ahMid > 4 * ahHigh && eeHigh > 4 * eeMid, "\"ah\" and \"ee\" should come out with their own shapes (ah %.1f dB, ee %.1f dB mid over high)",
        10 * std::log10(ahMid / ahHigh), 10 * std::log10(eeMid / eeHigh));
  double eeLow = bandEnergy(outEe, 20000, 180, 420), ahLow = bandEnergy(outAh, 20000, 180, 420);
  CHECK(eeLow > 4 * ahLow, "\"ee\" has its low formant near 300 Hz, \"ah\" has not (%.1f dB)", 10 * std::log10(eeLow / ahLow));
  std::printf("    ah: mid %.1f dB over high; ee: high %.1f dB over mid\n", 10 * std::log10(ahMid / ahHigh), 10 * std::log10(eeHigh / eeMid));

  section("vocoder: a voice at a healthy level gives about the synth's own loudness; no voice, no sound");
  double vocoded = rmsOf(outAh, 0.4, 1.0), dry = rmsOf(plain, 0.2, 0.5);
  CHECK(vocoded > dry * 0.25 && vocoded < dry * 2.5, "vocoded level %.4f against the plain synth's %.4f (%.1f dB)", vocoded, dry, 20 * std::log10(vocoded / dry));
  std::printf("    vocoded level is %.1f dB against the plain synth\n", 20 * std::log10(vocoded / dry));
  auto quiet = renderIn(1.0, silence);
  CHECK(rmsOf(quiet, 0.6, 1.0) < dry * 0.003, "with the voice silent the vocoded synth is silent (%.6f)", rmsOf(quiet, 0.6, 1.0));
  setStep(fw::P_VOCODER, 50);
  quiet = renderIn(0.5, silence);
  CHECK(std::fabs(rmsOf(quiet, 0.3, 0.5) / dry - 0.5) < 0.05, "at 50%% half of the plain synth is left (%.3f of it)", rmsOf(quiet, 0.3, 0.5) / dry);
  setStep(fw::P_VOCODER, 0);
  renderIn(0.3, ah);
  auto back = renderIn(0.2, ah);
  CHECK(rmsOf(back, 0.0, 0.2) > dry * 0.9 && !fw::gVocoder.on(), "back at 0 the plain synth returns and the vocoder is out of the path");
  boot(48000);
  vocoderCarrier();
  setStep(fw::P_VOCODER, 100);
  midi({0x90, 57, 100});
  auto noInput = renderNew(0.5);  // a target with no audio input at all
  CHECK(peakOf(noInput, 12000) < 1e-5, "with no input connected the vocoded synth is silent, not broken");

  section("vocoder: Smooth sets how fast a band's level falls when the voice stops");
  double fallMs[2];
  for (int c = 0; c < 2; ++c) {
    boot(48000);
    vocoderCarrier();
    setStep(fw::P_VOCODER, 100);
    setStep(fw::P_VOC_SMOOTH, c == 0 ? 0 : 100);
    midi({0x90, 57, 100});
    auto out = renderIn(1.5, [&](long i) { return i < 24000 ? ah(i) : std::make_pair(0.0f, 0.0f); });
    std::vector<double> lv = levels(out, 5.0);
    double full = lv[90];  // at 0.45 s, with the voice steady
    size_t i = 100;
    while (i < lv.size() && lv[i] > 0.1 * full) ++i;
    fallMs[c] = ((double)i - 100.0) * 5.0;
  }
  CHECK(fallMs[0] < 60 && fallMs[1] > 300 && fallMs[1] < 1200, "crisp at 0 (%.0f ms to fall to a tenth), slurred at 100 (%.0f ms)", fallMs[0], fallMs[1]);
  std::printf("    time to fall to a tenth after the voice stops: %.0f ms at Smooth 0, %.0f ms at 100\n", fallMs[0], fallMs[1]);

  section("vocoder: Formant moves the voice's shape up or down the synth's sound");
  double centre[3];
  for (int c = 0; c < 3; ++c) {
    boot(48000);
    vocoderCarrier();
    setStep(fw::P_VOCODER, 100);
    setStep(fw::P_VOC_CONSONANTS, 0);
    setStep(fw::P_VOC_FORMANT, c == 0 ? 0 : c == 1 ? 12 : 24);  // an octave down, none, an octave up
    midi({0x90, 45, 100});
    auto out = renderIn(1.0, ah);
    std::vector<double> m = spectrum(out, 20000, 16384);
    double num = 0, den = 0;
    for (size_t i = 4; i < m.size(); ++i) {
      double hz = (double)i * gSr / 16384.0;
      num += std::log2(hz) * m[i] * m[i];
      den += m[i] * m[i];
    }
    centre[c] = num / den;  // the middle of the sound, in octaves
  }
  CHECK(centre[1] - centre[0] > 0.6 && centre[2] - centre[1] > 0.6, "each octave of Formant should move the sound's middle most of an octave (%.2f, %.2f)",
        centre[1] - centre[0], centre[2] - centre[1]);

  section("vocoder: Consonants lets an S through a dull sound, but only while a note is held");
  double hiss[3];
  for (int c = 0; c < 3; ++c) {
    boot(48000);
    vocoderCarrier();
    setPot("filter_cutoff", 0.35);  // a dull sound: almost nothing up where an S lives
    setPot("filter_env_amount", 0.5);
    setStep(fw::P_VOCODER, 100);
    setStep(fw::P_VOC_CONSONANTS, c == 0 ? 0 : 100);
    if (c != 2) midi({0x90, 45, 100});
    auto out = renderIn(1.0, ess);
    hiss[c] = std::sqrt(bandEnergy(out, 20000, 3500, 9000));
  }
  CHECK(hiss[1] > 8 * hiss[0], "at 100%% the S should be far stronger than at 0 (%.1f dB)", 20 * std::log10(hiss[1] / (hiss[0] + 1e-12)));
  CHECK(hiss[2] < 0.02 * hiss[1], "with no note held there is no hiss on its own (%.1f dB down)", 20 * std::log10(hiss[1] / (hiss[2] + 1e-12)));
  boot(48000);
  vocoderCarrier();
  setStep(fw::P_VOCODER, 100);
  setStep(fw::P_VOC_CONSONANTS, 100);
  midi({0x90, 45, 100});
  auto vowelOnly = renderIn(1.0, ah);
  double lows = bandEnergy(vowelOnly, 20000, 200, 3000), highs = bandEnergy(vowelOnly, 20000, 4000, 9000);
  CHECK(highs < 0.05 * lows, "a vowel does not bring the hiss up with it (%.1f dB below the vowel)", 10 * std::log10(lows / highs));

  section("vocoder: a loud voice on a full chord stays in bounds; high sample rates are safe");
  for (double sr : {48000.0, 96000.0}) {
    boot(sr);
    vocoderCarrier();
    setStep(fw::P_VOCODER, 100);
    setStep(fw::P_VOC_CONSONANTS, 100);
    setStep(fw::P_VOC_FORMANT, 24);
    midi({0x90, 36, 127, 0x90, 43, 127, 0x90, 48, 127, 0x90, 52, 127, 0x90, 55, 127, 0x90, 60, 127, 0x90, 64, 127, 0x90, 67, 127});
    const Vowel loud(140, 700, 1200, 2600, 1.0);
    auto out = renderIn(2.0, loud);
    bool fine = true;
    for (float x : out) fine = fine && std::isfinite(x) && std::fabs(x) <= 1.0f;
    CHECK(fine && peakOf(out) > 0.05, "at %.0f Hz the output should stay finite, in range and audible (peak %.3f)", sr, peakOf(out));
  }

  section("vocoder: its page shows the voice's bands live, and says why when nothing can be heard");
  boot(48000);
  vocoderCarrier();
  press("btn_fx2");
  press("btn_fx2");
  fw::tick(33);
  auto offPage = pictureRows(76, 154);
  fw::encoderTurn(ctl("enc1"), 60);
  fw::tick(2000);
  fw::tick(33);
  auto noSignal = pictureRows(76, 154);
  CHECK(noSignal != offPage, "turned up, the page shows the bands in place of the scope");
  renderIn(0.5, ah);
  fw::tick(33);
  auto noNote = pictureRows(76, 154);
  CHECK(noNote != noSignal, "with a voice arriving the bands move");
  saveFrame("mode_vocoder_voice");
  midi({0x90, 57, 100});
  renderIn(0.5, ee);
  fw::tick(33);
  auto playing = pictureRows(76, 154);
  CHECK(playing != noNote, "and a different vowel gives a different picture");
  saveFrame("mode_vocoder_playing");
  double low = 0, high = 0;
  for (int k = 0; k < 6; ++k) low += fw::gVocoder.band(k);
  for (int k = 10; k < 14; ++k) high += fw::gVocoder.band(k);
  CHECK(low > 0.3 && high > 0.3 && fw::gVocoder.band(8) < 0.5 * fw::gVocoder.band(12), "\"ee\" lights the low bands and the high ones, with a gap between");
  for (int e = 1; e < 4; ++e) {
    fw::encoderTurn(ctl(e == 1 ? "enc2" : e == 2 ? "enc3" : "enc4"), 3);
    fw::tick(33);
    saveFrame(e == 1 ? "soft_vocoder_formant" : e == 2 ? "soft_vocoder_consonants" : "soft_vocoder_smooth");
    fw::tick(2000);
  }

  section("vocoder: its four values are part of a patch");
  boot(48000);
  setStep(fw::P_VOCODER, 80);
  setStep(fw::P_VOC_FORMANT, 17);
  setStep(fw::P_VOC_CONSONANTS, 70);
  setStep(fw::P_VOC_SMOOTH, 10);
  savePatchTo(2);
  setStep(fw::P_VOCODER, 0);
  setStep(fw::P_VOC_FORMANT, 12);
  setStep(fw::P_VOC_CONSONANTS, 50);
  setStep(fw::P_VOC_SMOOTH, 35);
  CHECK(fw::patchLoad(2), "the patch should load");
  CHECK(fw::gParams.step(fw::P_VOCODER) == 80 && fw::gParams.step(fw::P_VOC_FORMANT) == 17 && fw::gParams.step(fw::P_VOC_CONSONANTS) == 70 &&
            fw::gParams.step(fw::P_VOC_SMOOTH) == 10,
        "loading the patch should bring the vocoder's settings back");

  section("vocoder: what it costs on this machine (information only)");
  for (int on = 0; on < 2; ++on) {
    boot(48000);
    vocoderCarrier();
    setStep(fw::P_VOCODER, on ? 100 : 0);
    midi({0x90, 57, 100});
    auto t0 = std::chrono::steady_clock::now();
    renderIn(10.0, ah);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("    ten seconds, one note, vocoder %s: %.0f ms\n", on ? "on " : "off", ms);
  }
}

// ------------------------------------------------- findings of the second independent review, 2026-11-21

// How many times a square-wave LFO on level goes round in four seconds, rendered in blocks of `block` samples.
static int lfoCycles(int sync, int block) {
  boot(48000);
  cleanTone(fw::WAVE_SIN);
  setSel("lfo_shape", fw::LFOS_SQUARE);
  setSel("lfo_destination", fw::LFO_AMP);
  setPot("lfo_depth", 1.0);
  setPot("lfo_rate", 0.6);
  setSel("lfo_sync", sync);
  renderNew(0.3);
  midi({0x90, 69, 100});
  std::vector<float> x;
  float l[fw::kMaxBlock], r[fw::kMaxBlock];
  for (long n = 4 * 48000; n > 0; n -= block) {
    fw::process(l, r, block);
    x.insert(x.end(), l, l + block);
  }
  std::vector<double> lv = levels(x, 2.0);
  double peak = 0;
  for (double v : lv) peak = std::max(peak, v);
  int count = 0;
  bool high = lv[0] > 0.5 * peak;
  for (double v : lv) {
    bool now = v > (high ? 0.3 : 0.7) * peak;
    if (now && !high) ++count;
    high = now;
  }
  return count;
}

static void testSecondReview() {
  auto tone = [](long i) {
    float x = (float)(0.3 * std::sin(2.0 * M_PI * 440.0 * (double)i / gSr));
    return std::make_pair(x, x);
  };

  section("review 2: changing Channel lets go of whatever was held on the old one");
  boot(48000);
  midi({0x90, 60, 100});
  renderNew(0.2);
  fw::settingSet(fw::SET_CHANNEL, 2);
  midi({0x80, 60, 0});  // on channel 1: no longer listened to
  renderNew(3.0);
  CHECK(fw::gSynth.activeVoices() == 0, "a key held across a Channel change should not sound for ever (%d voices)", fw::gSynth.activeVoices());
  boot(48000);
  midi({0xB0, 64, 127, 0x90, 60, 100, 0x80, 60, 0, 0xE0, 0, 0x7F});
  renderNew(0.2);
  fw::settingSet(fw::SET_CHANNEL, 2);
  midi({0xB0, 64, 0});
  renderNew(3.0);
  CHECK(fw::gSynth.activeVoices() == 0, "nor a note held by the pedal (%d voices)", fw::gSynth.activeVoices());
  boot(48000, true);
  CHECK(fw::setting(fw::SET_CHANNEL) == 0 || fw::gSynth.activeVoices() == 0, "a saved Channel setting loads without upsetting anything");

  section("review 2: MIDI Start with a chord held plays nothing until the first tick, then the pattern's first note");
  for (double gapMs : {2.0, 20.0}) {
    boot(48000);
    arpTone();
    fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
    setSel("arp_mode", fw::ARP_UP);
    midi({0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100});
    renderNew(0.4);  // no clock yet: the arp is running from the Tempo knob
    uint32_t before = fw::gSynth.arpStepCount();
    midi({0xFA});
    renderNew(gapMs * 0.001);
    CHECK(fw::gSynth.arpStepCount() == before, "%.0f ms between start and the first tick: no step should play in the gap", gapMs);
    clockBegin();
    auto steps = runClock(0.11, 120);
    double hz = measureFreq(gClockAudio, 480, 4800);
    CHECK(steps.size() == 1 && steps[0] < 0.002 && std::fabs(hz - 261.63) < 4.0, "the first tick plays the pattern's first note (%zu steps, %.1f Hz)", steps.size(), hz);
  }
  boot(48000);
  arpTone();
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  setSel("arp_mode", fw::ARP_UP);
  midi({0x90, 60, 100, 0xFA});
  clockBegin();
  auto steps = runClock(3.0, 0);
  CHECK(steps.size() >= 12 && steps.size() <= 17 && steps[0] > 0.9, "a start with no clock after it: the arp waits a second, then runs from the Tempo knob (%zu steps)", steps.size());

  section("review 2: continue carries on with the pattern; start begins it again");
  boot(48000);
  arpTone();
  setPot("aenv_release", 0.0);
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  setSel("arp_mode", fw::ARP_UP);
  midi({0x90, 60, 100, 0x90, 64, 100, 0x90, 67, 100, 0x90, 71, 100});
  clockBegin();
  midi({0xFA});
  steps = runClock(0.24, 120);  // two steps: C and E
  CHECK(steps.size() == 2, "two steps before the stop (%zu)", steps.size());
  midi({0xFC});
  runClock(0.5, 120);
  midi({0xFB});
  steps = runClock(0.2, 120);
  size_t at = steps.empty() ? 0 : (size_t)(steps[0] * gSr);
  double next = steps.empty() ? 0 : measureFreq(gClockAudio, at + 480, at + 4800);
  CHECK(std::fabs(next - 392.0) < 6.0, "after continue the next note is the pattern's third (G4, 392 Hz), got %.1f Hz", next);
  saveSound("arp_stop_continue", gClockAudio);

  section("review 2: after a stop the arp waits; with the clock gone too, a fresh chord plays from the Tempo knob");
  boot(48000);
  arpTone();
  fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
  setSel("arp_mode", fw::ARP_UP);
  press("btn_hold");
  midi({0x90, 60, 100, 0x80, 60, 0});  // latched by Hold
  clockBegin();
  midi({0xFA});
  runClock(1.0, 120);
  midi({0xFC});
  steps = runClock(2.0, 0);  // the sender stops its clock as well, as a DAW does
  CHECK(steps.empty() && fw::gSynth.clockState() == fw::CLK_STOPPED, "what was latched stays silent after a stop (%zu steps)", steps.size());
  midi({0x90, 64, 100});
  steps = runClock(1.0, 0);
  CHECK(steps.size() >= 7 && fw::gSynth.clockState() == fw::CLK_WAITING, "a fresh chord is not kept waiting for a clock that has gone (%zu steps)", steps.size());
  midi({0x80, 64, 0});
  press("btn_hold");

  section("review 2: the Volume knob turns the audio input down with the synth");
  for (int to : {fw::INPUT_MIX, fw::INPUT_FX}) {
    boot(48000);
    fw::settingSet(fw::SET_INPUT_TO, to);
    auto usual = renderIn(0.5, tone);
    setPot("volume", 0.0);
    renderIn(0.3, tone);
    auto off = renderIn(0.2, tone);
    CHECK(std::fabs(peakOf(usual, 12000) - 0.3) < 0.002 && peakOf(off) < 1e-4, "Send To %s: %.3f at the usual volume, %.5f with Volume at zero",
          to == fw::INPUT_MIX ? "MIX" : "FX", peakOf(usual, 12000), peakOf(off));
  }

  section("review 2: the duck keeps the beat with Arp mode on and no key held");
  boot(48000);
  setSel("arp_mode", fw::ARP_UP);
  setStep(fw::P_DUCK, 100);
  midi({0x90, 60, 100});
  renderNew(0.52);
  midi({0x80, 60, 0});
  {
    int beats = 0;
    float l[16], r[16], last = fw::gSynth.beatNow();
    for (int i = 0; i < 2 * 48000 / 16; ++i) {
      fw::process(l, r, 16);
      float now = fw::gSynth.beatNow();
      if (now < last - 0.5f) ++beats;
      last = now;
    }
    CHECK(beats >= 3 && beats <= 4, "two seconds at 120 BPM is four beats; the duck counted %d", beats);
  }

  section("review 2: a target that can save at any time does not wait for silence");
  boot(48000);
  midi({0x90, 60, 100});
  renderNew(0.1);
  fw::saveWhilePlaying(true);
  setPot("filter_cutoff", 0.9);
  int saves = gSaveCount;
  fw::tick(1200);
  CHECK(gSaveCount == saves + 1, "with a note sounding, the state should still be saved a second after the change");
  fw::saveWhilePlaying(false);
  setPot("filter_cutoff", 0.8);
  renderNew(0.05);
  fw::tick(1200);
  CHECK(gSaveCount == saves + 1, "hardware still waits for silence");
  midi({0x80, 60, 0});

  section("review 2: one LFO per voice runs at the same rate whatever the audio block length");
  {
    int whole = lfoCycles(fw::LFOSYNC_VOICE, 128), eight = lfoCycles(fw::LFOSYNC_VOICE, 8), one = lfoCycles(fw::LFOSYNC_VOICE, 1);
    CHECK(whole >= 8 && std::abs(whole - eight) <= 1 && std::abs(whole - one) <= 1, "cycles in four seconds: %d in blocks of 128, %d in 8s, %d in 1s", whole, eight, one);
  }

  section("review 2: Thru never leaves a note hanging further down the chain");
  auto allNotesOff = [](const std::vector<int>& v) {
    if (v.size() < 48) return false;
    size_t from = v.size() - 48;
    for (int ch = 0; ch < 16; ++ch)
      if (v[from + ch * 3] != (0xB0 | ch) || v[from + ch * 3 + 1] != 123 || v[from + ch * 3 + 2] != 0) return false;
    return true;
  };
  boot(48000);
  fw::settingSet(fw::SET_THRU, 1);
  midi({0x90, 60, 100});
  drainMidi(fw::PORT_MIDI);
  drainMidi(fw::PORT_USB);
  fw::settingSet(fw::SET_THRU, 0);  // with the note still held
  fw::tick(33);
  CHECK(allNotesOff(drainMidi(fw::PORT_MIDI)) && allNotesOff(drainMidi(fw::PORT_USB)), "switching Thru off sends all notes off on every channel");
  fw::settingSet(fw::SET_THRU, 1);
  for (int i = 0; i < 400; ++i) midi({0x90, 60, 100});  // more than the output holds: some are dropped
  fw::tick(33);
  drainMidi(fw::PORT_MIDI);
  fw::tick(33);
  CHECK(allNotesOff(drainMidi(fw::PORT_MIDI)), "after messages were dropped, all notes off follows once there is room");
  fw::tick(33);
  CHECK(drainMidi(fw::PORT_MIDI).empty(), "and it is sent once, not for ever");

  section("review 2: the MIDI page says in words what Channel, Thru and Clock Out amount to");
  boot(48000);
  for (int i = 0; i < 4; ++i) press("btn_patch");
  fw::tick(33);
  auto words = pictureRows(126, 162);
  static const int kThree[3] = {fw::SET_CHANNEL, fw::SET_THRU, fw::SET_CLOCK_OUT};
  for (int id : kThree) {
    fw::settingSet(id, 1);
    fw::tick(33);
    auto now = pictureRows(126, 162);
    CHECK(now != words, "%s: the words on the page should change with it", fw::kSettings[id].name);
    words = now;
  }
  saveFrame("mode_midi_words");

  section("review 2: the duck's picture shows the beat it is really running at when the clock is stopped");
  std::vector<uint8_t> duckPicture[3];
  for (int c = 0; c < 3; ++c) {  // the synth's own clock; a clock at 60 BPM, stopped; that clock running
    boot(48000);
    if (c > 0) {
      fw::settingSet(fw::SET_CLOCK, fw::CLOCK_MIDI);
      clockBegin();
      midi({0xFA});
      runClock(2.0, 60);
      if (c == 1) {
        midi({0xFC});
        runClock(0.2, 60);
      }
    }
    press("btn_mod");
    fw::encoderTurn(ctl("enc4"), 10);
    fw::tick(33);
    duckPicture[c] = pictureRows(76, 154);
  }
  CHECK(duckPicture[1] == duckPicture[0], "stopped, the duck runs at the Tempo knob, and its picture should say so");
  CHECK(duckPicture[2] != duckPicture[0], "running, it follows the clock, and its picture shows the slower beat");
}

static void benchmark() {
  section("speed on this machine (information only; says nothing about the Teensy)");
  boot(48000);
  setPot("mix_osc1", 0.8);
  setPot("mix_osc2", 0.8);
  setPot("mix_sub", 0.8);
  setPot("mix_noise", 0.3);
  setSel("osc1_wave", fw::WAVE_SQR);
  setPot("filter_drive", 0.5);
  setPot("lfo_depth", 0.4);
  for (int i = 0; i < 8; ++i) midi({0x90, 40 + i * 4, 100});
  float buf[fw::kMaxBlock], buf2[fw::kMaxBlock];
  timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  const int blocks = 3750;  // 10 s
  for (int b = 0; b < blocks; ++b) fw::process(buf, buf2, fw::kMaxBlock);
  clock_gettime(CLOCK_MONOTONIC, &t1);
  double sec = (double)(t1.tv_sec - t0.tv_sec) + (double)(t1.tv_nsec - t0.tv_nsec) * 1e-9;
  std::printf("    8 voices, every source on, 24 dB filter with drive: 10 s of audio in %.0f ms (%.0fx real time)\n",
              sec * 1000, 10.0 / sec);
}

// ---------------------------------------------------------- docs/controls.md

static void printControls() {
  std::printf("# Controls and MIDI controller numbers\n\n");
  std::printf("Generated from the firmware's parameter table by `make docs`. Do not edit by hand.\n\n");
  std::printf("The synth listens on every MIDI channel unless one is chosen on the MIDI page (Patch button,\n");
  std::printf("fourth page). Besides the numbers below it responds to\n");
  std::printf("notes with velocity, pitch bend, mod wheel (CC 1, adds LFO depth), sustain pedal (CC 64),\n");
  std::printf("all sound off (CC 120) and all notes off (CC 123). A program change loads that patch slot\n");
  std::printf("(0 to %d), if it holds a patch.\n\n", fw::kPatchSlots - 1);
  std::printf("A controller message for a panel control sets the sound, not the control: the control\n");
  std::printf("follows that value until it is moved through it (pots and faders) or changed (selectors).\n\n");
  std::printf("## Panel controls\n\n");
  std::printf("| Control id | Section | Name | Type | Stops | CC |\n|---|---|---|---|---|---|\n");
  int order[fw::P_COUNT], n = 0;
  for (int i = 0; i < fw::kPanelCount; ++i) order[n++] = fw::kPanelParams[i];
  for (int i = 0; i < fw::P_COUNT; ++i)
    if (!fw::isPanelParam(i)) order[n++] = i;
  for (int k = 0; k < fw::P_COUNT; ++k) {
    int i = order[k];
    if (k == fw::kPanelCount) {
      std::printf("\n## Soft parameters (encoder pages and buttons)\n\n");
      std::printf("Wave, Macro and Color mean different things under each oscillator engine:\n");
      std::printf("VA: wave, pulse width, none. FM: ratio, FM amount, feedback. ");
      std::printf("Wavetable: table, position, none. Pluck: exciter, damping, decay.\n\n");
      std::printf("| Parameter id | Page | Name | Values | CC |\n|---|---|---|---|---|\n");
    }
    const fw::ParamDef& d = fw::kParams[i];
    std::string stops;
    if (d.kind == fw::PK_STEP && d.labels) {
      for (int s = 0; s < d.steps; ++s) stops += std::string(s ? ", " : "") + d.labels[s];
    } else if (d.kind == fw::PK_STEP) {
      stops = std::to_string(d.intMin) + " to " + std::to_string(d.intMin + d.steps - 1);
    } else {
      stops = "";
    }
    if (k < fw::kPanelCount) {
      std::printf("| `%s` | %s | %s | %s | %s | %d |\n", d.key, fw::kSectionNames[d.section], d.name,
                  d.kind == fw::PK_STEP ? "selector" : "pot", stops.c_str(), d.cc);
    } else {
      std::printf("| `%s` | %s | %s | %s | %d |\n", d.key, fw::kSectionNames[d.section], d.name, stops.c_str(), d.cc);
    }
  }
  std::printf("\n## Settings (Patch button, MIDI and Input pages)\n\n");
  std::printf("Kept over a restart; not part of a patch; no controller numbers.\n\n| Setting | Values |\n|---|---|\n");
  for (int i = 0; i < fw::SET_COUNT; ++i) {
    const fw::SettingDef& d = fw::kSettings[i];
    std::string values;
    if (d.labels) {
      for (int v = 0; v < d.count; ++v) values += std::string(v ? ", " : "") + d.labels[v];
    } else {
      values = i == fw::SET_CHANNEL ? "ALL, 1 to 16" : "0 to " + std::to_string(d.count - 1) + " dB";
    }
    std::printf("| %s | %s |\n", d.name, values.c_str());
  }
  std::printf("\nWith Clock on MIDI or USB the synth follows MIDI clock, start, stop, continue and song position\n");
  std::printf("from that input.\n");
  std::printf("\n## Encoders and buttons\n\n| Control id | Type |\n|---|---|\n");
  for (int i = fw::kFirstEncoder; i < fw::kNumControls; ++i) {
    fw::ControlInfo info = fw::controlInfo(i);
    std::printf("| `%s` | %s |\n", info.name, info.kind == fw::CK_ENCODER ? "push-encoder" : "button");
  }
}

int main(int argc, char** argv) {
  if (argc > 1 && !std::strcmp(argv[1], "--controls")) {
    printControls();
    return 0;
  }
  if (argc > 1) gOutDir = argv[1];

  testMath();
  testSampleRateIndependence();
  testAliasing();
  testNoAllocation();
  testVoices();
  testPerformanceControls();
  testEveryControl();
  testMidi();
  testUi();
  testSoftPages();
  testTextSize();
  testPackedFrame();
  testState();
  testOlderSavedState();
  testVersion2State();
  testPanelRegistry();
  testDriftAndSpread();
  testSupersawSyncXmod();
  testLadderFilter();
  testChordMemory();
  testPatches();
  testPictures();
  testNotesEndInSilence();
  testHighSampleRates();
  testOverloadAndSaving();
  testPotReadings();
  testReviewPictures();
  testRandomUse();
  testDanceLights();
  testNewSoundFeatures();
  testClock();
  testMidiOutAndChannel();
  testAudioInput();
  testVocoder();
  testSecondReview();
  testLfoShapes();
  testFilterControls();
  testArpAndHold();
  testEffects();
  testMod();
  testEngines();
  testEverySoftParameter();
  testLongRun();
  benchmark();

  std::printf("\n%d checks, %d failed\n", gChecks, gFails);
  return gFails ? 1 : 0;
}
