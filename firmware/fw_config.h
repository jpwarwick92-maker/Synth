// Build-wide constants for the synth firmware core.
//
// The core is plain C++17 with no standard library, no heap and no
// platform headers, so the same files build for the browser (WebAssembly),
// for the Teensy 4.1 and for the native test runner.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Large static buffers that are read only now and then per sample (the
// wavetables, the delay line, the scope) are tagged with FW_BIGMEM. A target
// can define it, before this file is read, to place them in a particular
// memory region (the Teensy target does: targets/teensy/src/fw_target.h).
// Such a region need not be cleared at start-up: the core initialises all of it.
#ifndef FW_BIGMEM
#define FW_BIGMEM
#endif

namespace fw {

constexpr int kNumVoices = 8;
constexpr int kCtrlBlock = 16;   // samples between control-rate updates
constexpr int kMaxBlock = 128;   // largest block process() is called with

constexpr int kDisplayW = 320;
constexpr int kDisplayH = 240;

constexpr int kScopeLen = 2048;  // output history kept for the scope (power of 2)

}  // namespace fw
