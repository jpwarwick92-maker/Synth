// Build-wide constants for the synth firmware core.
//
// The core is plain C++17 with no standard library, no heap and no
// platform headers, so the same files build for the browser (WebAssembly),
// for the Teensy 4.1 and for the native test runner.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Large static buffers (wavetables, the display buffer) are tagged with
// FW_BIGMEM. A target can define it to place them in a specific memory
// region. On the Teensy 4.1 they go in RAM2, leaving the fast RAM for code.
#ifndef FW_BIGMEM
#if defined(__IMXRT1062__)  // Teensy 4.x: RAM2. Not cleared at start-up; the core initialises all of it.
#define FW_BIGMEM __attribute__((section(".dmabuffers"), used))
#else
#define FW_BIGMEM
#endif
#endif

namespace fw {

constexpr int kNumVoices = 8;
constexpr int kCtrlBlock = 16;   // samples between control-rate updates
constexpr int kMaxBlock = 128;   // largest block process() is called with

constexpr int kDisplayW = 320;
constexpr int kDisplayH = 240;

constexpr int kScopeLen = 2048;  // output history kept for the scope (power of 2)

}  // namespace fw
