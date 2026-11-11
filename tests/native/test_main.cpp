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

static void render(std::vector<float>& out, double sec) {
  long n = std::lround(sec * gSr);
  float buf[fw::kMaxBlock];
  while (n > 0) {
    int len = n > fw::kMaxBlock ? fw::kMaxBlock : (int)n;
    fw::process(buf, len);
    out.insert(out.end(), buf, buf + len);
    n -= len;
  }
}

static std::vector<float> renderNew(double sec) {
  std::vector<float> v;
  render(v, sec);
  return v;
}

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

static AliasResult aliasCheck(const std::vector<float>& x) {
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
  const uint16_t* fb = fw::frame();
  for (int i = 0; i < fw::kDisplayW * fw::kDisplayH; ++i) {
    uint16_t p = fb[i];
    unsigned char rgb[3] = {(unsigned char)((p >> 11) << 3), (unsigned char)(((p >> 5) & 63) << 2),
                            (unsigned char)((p & 31) << 3)};
    std::fwrite(rgb, 1, 3, f);
  }
  std::fclose(f);
}

static long countColour(uint16_t c, int y0 = 0, int y1 = fw::kDisplayH) {
  const uint16_t* fb = fw::frame();
  long n = 0;
  for (int y = y0; y < y1; ++y)
    for (int x = 0; x < fw::kDisplayW; ++x)
      if (fb[y * fw::kDisplayW + x] == c) ++n;
  return n;
}

static std::vector<uint16_t> grabFrame() {
  return std::vector<uint16_t>(fw::frame(), fw::frame() + fw::kDisplayW * fw::kDisplayH);
}

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
  float buf[fw::kMaxBlock];
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
    for (int b = 0; b < 30; ++b) fw::process(buf, fw::kMaxBlock);
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
  setPot("lfo_shape", 0.1);
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
  CHECK(countColour(fw::kTeal, 220, 232) == 3 * 49, "status strip should show 3 voices lit");
  CHECK(countColour(fw::kAmber, 220, 232) >= 49, "MIDI activity light is not lit");
  midi({0xB0, 120, 0});

  section("AC6  moving a control shows its visual, home returns after 1.5 s");
  boot(48000);
  fw::tick(33);
  std::vector<uint16_t> home = grabFrame();
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
  std::vector<uint16_t> enginePage = grabFrame();

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

  static const char* const kOthers[] = {"btn_fx1", "btn_fx2", "btn_mod", "btn_arp", "btn_patch"};
  for (const char* b : kOthers) {
    press(b);
    fw::tick(33);
    CHECK(fw::controlGet(ctl(b)) == 1 && fw::controlGet(ctl("btn_engine")) == 0, "%s: lights wrong", b);
    CHECK(countColour(fw::kGrey, 90, 130) > 300, "%s: no 'not built yet' message", b);
    int v = fw::gParams.step(fw::P_UNI_DETUNE);
    fw::encoderTurn(enc1, 2);
    CHECK(fw::gParams.step(fw::P_UNI_DETUNE) == v, "%s: encoder should do nothing on a placeholder page", b);
  }
  saveFrame("mode_placeholder");
  press("btn_patch");  // pressing the lit button returns home
  fw::tick(33);
  CHECK(fw::gUi.mode() == fw::MB_NONE, "pressing the lit mode button should return to the home screen");

  press("btn_osc1_engine");
  fw::tick(33);
  saveFrame("popup_osc_engine");
  CHECK(countColour(fw::kWhite, 90, 125) > 500, "oscillator engine button: no 'VA only' message");
  fw::tick(1600);
  CHECK(countColour(fw::kWhite, 90, 125) == 0, "'VA only' message did not clear");
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
  float buf[fw::kMaxBlock];
  timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  const int blocks = 3750;  // 10 s
  for (int b = 0; b < blocks; ++b) fw::process(buf, fw::kMaxBlock);
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
      std::printf("\n## Engine page (soft controls)\n\n");
      std::printf("| Parameter id | Encoder | Name | Type | Values | CC |\n|---|---|---|---|---|---|\n");
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
      std::printf("| `%s` | %d | %s | stepped | %s | %d |\n", d.key, i - fw::P_PANEL_COUNT + 1, d.name, stops.c_str(),
                  d.cc);
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
  testState();
  benchmark();

  std::printf("\n%d checks, %d failed\n", gChecks, gFails);
  return gFails ? 1 : 0;
}
