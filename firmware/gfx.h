// Drawing into the display buffer.
//
// Only what a small SPI display driver on the Teensy can do: pixels, lines,
// filled rectangles and a bitmap font. The buffer is 320x240, 16 bits per
// pixel (RGB565), the native format of the ST7789.
#pragma once

#include "fw_config.h"

namespace fw {

constexpr uint16_t rgb565(int r, int g, int b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

constexpr uint16_t kBlack = rgb565(0, 0, 0);
constexpr uint16_t kWhite = rgb565(0xED, 0xED, 0xED);
constexpr uint16_t kAmber = rgb565(0xE8, 0xA3, 0x3D);
constexpr uint16_t kTeal = rgb565(0x7F, 0xE0, 0xC4);
constexpr uint16_t kDimTeal = rgb565(0x4F, 0x9E, 0x89);
constexpr uint16_t kGrey = rgb565(0x8E, 0x93, 0x9B);
constexpr uint16_t kDarkGrey = rgb565(0x2F, 0x32, 0x37);
constexpr uint16_t kGrid = rgb565(0x1C, 0x2A, 0x26);

extern uint16_t gFrame[kDisplayW * kDisplayH];

void gfxClear(uint16_t c);
void gfxPixel(int x, int y, uint16_t c);
void gfxFill(int x, int y, int w, int h, uint16_t c);
void gfxFrame(int x, int y, int w, int h, uint16_t c);
void gfxHLine(int x, int y, int w, uint16_t c);
void gfxVLine(int x, int y, int h, uint16_t c);
void gfxLine(int x0, int y0, int x1, int y1, uint16_t c);
// A line two pixels thick in every direction, for plots.
void gfxPlotLine(int x0, int y0, int x1, int y1, uint16_t c);

// Text. The font is 5x7 pixels per character on a 6-pixel pitch, multiplied
// by `scale`. On the real 2.4 inch display one pixel is 0.15 mm, so scale 1
// would be letters 1 mm tall: too small to read. Anything below
// kMinTextScale is drawn at kMinTextScale (letters 2.1 mm tall).
constexpr int kMinTextScale = 2;
int gfxTextWidth(const char* s, int scale);
void gfxText(int x, int y, const char* s, uint16_t c, int scale);
void gfxTextRight(int xRight, int y, const char* s, uint16_t c, int scale);
void gfxTextCentre(int xCentre, int y, const char* s, uint16_t c, int scale);

}  // namespace fw
