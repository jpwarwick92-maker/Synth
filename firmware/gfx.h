// Drawing into the display buffer.
//
// Only what a small SPI display driver on the Teensy can do: pixels, lines,
// filled rectangles and a bitmap font, in a handful of fixed colours.
#pragma once

#include "fw_config.h"

namespace fw {

// The buffer holds 4 bits per pixel: a colour number, two pixels to a byte
// (left pixel in the high half). That is 38 KB where 16 bits per pixel was
// 154 KB. kPalette565 gives each colour in the display's own 16-bit format,
// and gfxRow565 expands one row, which is all a display driver needs to
// send the frame a line at a time.
using Colour = uint8_t;
constexpr Colour kBlack = 0;
constexpr Colour kWhite = 1;
constexpr Colour kAmber = 2;
constexpr Colour kTeal = 3;
constexpr Colour kDimTeal = 4;
constexpr Colour kGrey = 5;
constexpr Colour kDarkGrey = 6;
constexpr Colour kGrid = 7;
constexpr int kColourCount = 8;  // room for 16

extern const uint16_t kPalette565[16];

constexpr int kFrameBytes = kDisplayW * kDisplayH / 2;
extern uint8_t gFrame[kFrameBytes];

Colour gfxGet(int x, int y);
void gfxRow565(int y, uint16_t* dst);  // dst: kDisplayW pixels

void gfxClear(Colour c);
void gfxPixel(int x, int y, Colour c);
void gfxFill(int x, int y, int w, int h, Colour c);
void gfxFrame(int x, int y, int w, int h, Colour c);
void gfxHLine(int x, int y, int w, Colour c);
void gfxVLine(int x, int y, int h, Colour c);
void gfxLine(int x0, int y0, int x1, int y1, Colour c);
// A line two pixels thick in every direction, for plots.
void gfxPlotLine(int x0, int y0, int x1, int y1, Colour c);

// Text. The font is 5x7 pixels per character on a 6-pixel pitch, multiplied
// by `scale`. On the real 2.4 inch display one pixel is 0.15 mm, so scale 1
// would be letters 1 mm tall: too small to read. Anything below
// kMinTextScale is drawn at kMinTextScale (letters 2.1 mm tall).
constexpr int kMinTextScale = 2;
int gfxTextWidth(const char* s, int scale);
void gfxText(int x, int y, const char* s, Colour c, int scale);
void gfxTextRight(int xRight, int y, const char* s, Colour c, int scale);
void gfxTextCentre(int xCentre, int y, const char* s, Colour c, int scale);

}  // namespace fw
