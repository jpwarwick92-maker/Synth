// Native tests for the firmware core.
//
// These build the same source files that go into the browser and Teensy
// builds, drive them only through firmware.h, and check the results by
// rendering audio and display frames with no browser involved.
//
//   build/tests <outdir>     run every test; writes display frames to <outdir>
//   build/tests --controls   print docs/controls.md

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include "firmware.h"
#include "fw_math.h"
#include "gfx.h"
#include "params.h"
#include "synth.h"
#include "tables.h"
#include "ui.h"

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
static uint8_t gStore[256];
static uint32_t gStoreLen = 0;
static int gSaveCount = 0;

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
  if (!keepStorage) gStoreLen = 0;
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
  section("AC4  each of the 32 panel controls changes the sound");
  boot(48000);
  phraseBaseline();
  renderNew(0.05);
  std::vector<float> base = phrase();
  int silent = 0;
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    boot(48000);
    phraseBaseline();
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
  section("AC9  a controller number moves its control exactly as dragging does");
  boot(48000);
  for (int c = 0; c < fw::kFirstEncoder; ++c) {
    fw::ControlInfo info = fw::controlInfo(c);
    int cc = fw::kParams[c].cc;
    midi({0xB0, cc, 0});
    midi({0xB0, cc, 100});
    int viaMidi = fw::controlGet(c);
    int expect = info.kind == fw::CK_SELECTOR ? 100 * info.steps / 128 : (100 * 4095 + 63) / 127;
    CHECK(viaMidi == expect, "%s via CC %d: got %d, expected %d", info.name, cc, viaMidi, expect);
    CHECK(fw::gUi.popupParam() == c, "%s via CC did not bring up its visual", info.name);
    fw::controlSet(c, 0);
    fw::controlSet(c, expect);
    CHECK(fw::controlGet(c) == viaMidi, "%s: dragging to the same place gives a different value", info.name);
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
  midi({0xC0, 5});                             // program change: ignored
  renderNew(0.1);
  CHECK(fw::gSynth.activeVoices() == 0, "sysex or program change started a voice");
}

static void testUi() {
  section("AC5  home screen: patch name, live scope, status strip");
  boot(48000);
  fw::tick(33);
  CHECK(countColour(fw::kAmber, 0, 30) > 100, "no patch name in the title row");
  long quietScope = countColour(fw::kTeal, 40, 208);
  for (int i = 0; i < 3; ++i) midi({0x90, 50 + i * 7, 110});
  renderNew(0.3);
  fw::tick(33);
  saveFrame("home");
  CHECK(countColour(fw::kTeal, 40, 208) > quietScope + 200, "scope shows no waveform while notes sound");
  CHECK(countColour(fw::kTeal, 222, 236) == 3 * 64, "status strip should show 3 voices lit");
  CHECK(countColour(fw::kAmber, 222, 236) >= 60, "MIDI activity light is not lit");
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
    CHECK(fw::gUi.popupParam() == c, "%s: visual not shown", info.name);
    CHECK(grabFrame() != home, "%s: display still shows the home screen", info.name);
    CHECK(countColour(fw::kAmber, 0, 44) > 60 && countColour(fw::kWhite, 0, 44) > 60, "%s: no name or value in the header",
          info.name);
    for (const char* shot : kShots)
      if (!std::strcmp(shot, info.name)) saveFrame((std::string("popup_") + info.name).c_str());
  }
  fw::tick(1400);
  CHECK(fw::gUi.popupParam() >= 0, "visual vanished before 1.5 s");
  fw::tick(200);
  CHECK(fw::gUi.popupParam() < 0, "visual still up after 1.6 s");
  CHECK(countColour(fw::kAmber, 0, 30) > 100, "home screen did not come back");

  section("AC7  Engine page, placeholders and the oscillator engine buttons");
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
  CHECK(fw::gParams.step(fw::P_SUB_OCTAVE) == 1, "enc2 should set the sub two octaves down");
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
  section("soft pages: FX 1, FX 2, Mod and Arp each put four values on the encoders");
  struct PageCase {
    const char* button;
    const char* shot;
    int params[4];
  };
  static const PageCase kCases[] = {
      {"btn_fx1", "mode_fx1", {fw::P_DLY_TIME, fw::P_DLY_TONE, fw::P_DLY_FDBK, fw::P_DLY_MIX}},
      {"btn_fx2", "mode_fx2", {fw::P_CHORUS, fw::P_RVB_SIZE, fw::P_RVB_TONE, fw::P_RVB_MIX}},
      {"btn_mod", "mode_mod", {fw::P_LFO_FADE, fw::P_VEL_CUTOFF, fw::P_FENV_PITCH, fw::P_LFO_SYNC}},
      {"btn_arp", "mode_arp", {fw::P_ARP_MODE, fw::P_ARP_TEMPO, fw::P_ARP_OCTAVES, fw::P_ARP_GATE}},
  };
  static const char* const kEncoders[4] = {"enc1", "enc2", "enc3", "enc4"};
  boot(48000);
  for (const PageCase& c : kCases) {
    press(c.button);
    fw::tick(33);
    CHECK(fw::controlGet(ctl(c.button)) == 1, "%s: light is off", c.button);
    CHECK(countColour(fw::kWhite, 44, 70) > 100 && countColour(fw::kWhite, 178, 204) > 100, "%s: values missing", c.button);
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

  section("soft pages: Patch is still a placeholder");
  press("btn_patch");
  fw::tick(33);
  saveFrame("mode_placeholder");
  CHECK(countColour(fw::kGrey, 90, 130) > 300, "no 'not built yet' message");
  uint16_t before[fw::P_COUNT];
  for (int i = 0; i < fw::P_COUNT; ++i) before[i] = fw::gParams.val[i];
  for (int e = 0; e < 4; ++e) fw::encoderTurn(ctl(kEncoders[e]), 2);
  int changed = 0;
  for (int i = 0; i < fw::P_COUNT; ++i) changed += before[i] != fw::gParams.val[i];
  CHECK(changed == 0, "encoders should do nothing on a placeholder page");
  press("btn_patch");

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
  long amberBefore = countColour(fw::kAmber, 222, 236);
  press("btn_hold");
  CHECK(fw::gParams.step(fw::P_HOLD) == 1 && fw::controlGet(ctl("btn_hold")) == 1, "Hold should be on and lit");
  fw::tick(1600);
  fw::tick(33);
  CHECK(countColour(fw::kAmber, 222, 236) > amberBefore + 60, "status strip should show HOLD");
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
    setStep(fw::P_ARP_GATE, gate);
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
    for (int w = 0; w < 4; ++w) {
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
      for (int w = 0; w < 3; ++w)  // the three whole-number ratios; 3.5:1 is deliberately not harmonic
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
      for (int w = 0; w < 4; ++w)
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
  for (int id = fw::P_PANEL_COUNT; id < fw::P_COUNT; ++id) {
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
    if (i == fw::P_LFO_SHAPE) continue;
    wrong += fw::gParams.val[i] != v;
  }
  CHECK(wrong == 0, "%d of the old parameters came back different", wrong);
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
  fw::gfxText(10, 10, "H", fw::kWhite, 1);
  long asked1 = countColour(fw::kWhite);
  fw::gfxClear(fw::kBlack);
  fw::gfxText(10, 10, "H", fw::kWhite, 2);
  long asked2 = countColour(fw::kWhite);
  CHECK(asked1 == asked2 && asked2 == 17 * 4, "scale 1 drew %ld pixels, scale 2 drew %ld (an H is 17 dots)", asked1, asked2);
  CHECK(fw::gfxTextWidth("AB", 1) == fw::gfxTextWidth("AB", 2), "text width must follow the same rule");

  // plot lines are two pixels thick in both directions, so a vertical edge
  // is as visible as a horizontal one
  fw::gfxClear(fw::kBlack);
  fw::gfxPlotLine(50, 20, 50, 60, fw::kTeal);
  long vertical = countColour(fw::kTeal);
  fw::gfxClear(fw::kBlack);
  fw::gfxPlotLine(20, 50, 60, 50, fw::kTeal);
  long horizontal = countColour(fw::kTeal);
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
  fw::gfxPixel(10, 5, fw::kAmber);
  fw::gfxPixel(13, 5, fw::kTeal);
  CHECK(fw::gfxGet(10, 5) == fw::kAmber && fw::gfxGet(11, 5) == fw::kGrid && fw::gfxGet(9, 5) == fw::kGrid, "even column");
  CHECK(fw::gfxGet(13, 5) == fw::kTeal && fw::gfxGet(12, 5) == fw::kGrid && fw::gfxGet(14, 5) == fw::kGrid, "odd column");
  // fills with every mix of odd and even edges cover exactly their rectangle
  int wrong = 0;
  for (int x0 = 20; x0 < 24; ++x0)
    for (int w = 1; w <= 6; ++w) {
      fw::gfxClear(fw::kBlack);
      fw::gfxFill(x0, 30, w, 3, fw::kWhite);
      for (int y = 28; y < 36; ++y)
        for (int x = 14; x < 36; ++x) {
          bool inside = x >= x0 && x < x0 + w && y >= 30 && y < 33;
          wrong += (fw::gfxGet(x, y) == fw::kWhite) != inside;
        }
    }
  CHECK(wrong == 0, "%d pixels wrong across the odd/even fill cases", wrong);
  // drawing off every edge must not wrap round or write outside the buffer
  fw::gfxClear(fw::kBlack);
  fw::gfxFill(-5, -5, 10, 10, fw::kTeal);
  fw::gfxFill(315, 235, 10, 10, fw::kTeal);
  fw::gfxPixel(320, 0, fw::kAmber);
  fw::gfxPixel(-1, 0, fw::kAmber);
  CHECK(countColour(fw::kTeal) == 50 && countColour(fw::kAmber) == 0, "clipping at the edges: %ld teal, %ld amber", countColour(fw::kTeal), countColour(fw::kAmber));
  // a row expands to the display's own colours
  uint16_t row[fw::kDisplayW];
  fw::gfxClear(fw::kBlack);
  fw::gfxPixel(7, 100, fw::kAmber);
  fw::frameRow565(100, row);
  CHECK(row[7] == fw::kPalette565[fw::kAmber] && row[6] == fw::kPalette565[fw::kBlack] && row[8] == 0, "row expansion");

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
  std::printf("The synth listens on every MIDI channel. Besides the numbers below it responds to\n");
  std::printf("notes with velocity, pitch bend, mod wheel (CC 1, adds LFO depth), sustain pedal (CC 64),\n");
  std::printf("all sound off (CC 120) and all notes off (CC 123).\n\n");
  std::printf("## Panel controls\n\n");
  std::printf("| Control id | Section | Name | Type | Stops | CC |\n|---|---|---|---|---|---|\n");
  for (int i = 0; i < fw::P_COUNT; ++i) {
    if (i == fw::P_PANEL_COUNT) {
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
    if (i < fw::P_PANEL_COUNT) {
      std::printf("| `%s` | %s | %s | %s | %s | %d |\n", d.key, fw::kSectionNames[d.section], d.name,
                  d.kind == fw::PK_STEP ? "selector" : "pot", stops.c_str(), d.cc);
    } else {
      std::printf("| `%s` | %s | %s | %s | %d |\n", d.key, fw::kSectionNames[d.section], d.name, stops.c_str(), d.cc);
    }
  }
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
  testLfoShapes();
  testArpAndHold();
  testEffects();
  testMod();
  testEngines();
  testEverySoftParameter();
  benchmark();

  std::printf("\n%d checks, %d failed\n", gChecks, gFails);
  return gFails ? 1 : 0;
}
