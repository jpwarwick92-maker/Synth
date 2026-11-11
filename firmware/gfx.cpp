#include "gfx.h"

#include "font5x7.h"

namespace fw {

namespace {
constexpr uint16_t rgb565(int r, int g, int b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
}  // namespace

const uint16_t kPalette565[16] = {
    rgb565(0, 0, 0),           // kBlack
    rgb565(0xED, 0xED, 0xED),  // kWhite
    rgb565(0xE8, 0xA3, 0x3D),  // kAmber
    rgb565(0x7F, 0xE0, 0xC4),  // kTeal
    rgb565(0x4F, 0x9E, 0x89),  // kDimTeal
    rgb565(0x8E, 0x93, 0x9B),  // kGrey
    rgb565(0x2F, 0x32, 0x37),  // kDarkGrey
    rgb565(0x1C, 0x2A, 0x26),  // kGrid
};

uint8_t gFrame[kFrameBytes];

void gfxClear(Colour c) {
  uint8_t both = (uint8_t)((c << 4) | c);
  for (int i = 0; i < kFrameBytes; ++i) gFrame[i] = both;
}

Colour gfxGet(int x, int y) {
  if ((unsigned)x >= (unsigned)kDisplayW || (unsigned)y >= (unsigned)kDisplayH) return kBlack;
  uint8_t b = gFrame[(y * kDisplayW + x) >> 1];
  return (x & 1) ? (b & 0x0F) : (b >> 4);
}

void gfxRow565(int y, uint16_t* dst) {
  const uint8_t* src = gFrame + y * (kDisplayW / 2);
  for (int i = 0; i < kDisplayW / 2; ++i) {
    dst[2 * i] = kPalette565[src[i] >> 4];
    dst[2 * i + 1] = kPalette565[src[i] & 0x0F];
  }
}

void gfxPixel(int x, int y, Colour c) {
  if ((unsigned)x >= (unsigned)kDisplayW || (unsigned)y >= (unsigned)kDisplayH) return;
  uint8_t& b = gFrame[(y * kDisplayW + x) >> 1];
  b = (x & 1) ? (uint8_t)((b & 0xF0) | c) : (uint8_t)((b & 0x0F) | (c << 4));
}

void gfxFill(int x, int y, int w, int h, Colour c) {
  int x1 = x + w, y1 = y + h;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x1 > kDisplayW) x1 = kDisplayW;
  if (y1 > kDisplayH) y1 = kDisplayH;
  if (x >= x1) return;
  uint8_t both = (uint8_t)((c << 4) | c);
  for (int yy = y; yy < y1; ++yy) {
    uint8_t* row = gFrame + yy * (kDisplayW / 2);
    int xx = x;
    if (xx & 1) {  // a lone right-hand pixel at the start
      row[xx >> 1] = (uint8_t)((row[xx >> 1] & 0xF0) | c);
      ++xx;
    }
    for (; xx + 1 < x1; xx += 2) row[xx >> 1] = both;  // whole bytes
    if (xx < x1) row[xx >> 1] = (uint8_t)((row[xx >> 1] & 0x0F) | (c << 4));  // a lone left-hand pixel at the end
  }
}

void gfxHLine(int x, int y, int w, Colour c) { gfxFill(x, y, w, 1, c); }
void gfxVLine(int x, int y, int h, Colour c) { gfxFill(x, y, 1, h, c); }

void gfxFrame(int x, int y, int w, int h, Colour c) {
  gfxHLine(x, y, w, c);
  gfxHLine(x, y + h - 1, w, c);
  gfxVLine(x, y, h, c);
  gfxVLine(x + w - 1, y, h, c);
}

// Bresenham, stamping a brush x brush square at every step.
static void lineWithBrush(int x0, int y0, int x1, int y1, Colour c, int brush) {
  int dx = x1 > x0 ? x1 - x0 : x0 - x1;
  int dy = y1 > y0 ? y0 - y1 : y1 - y0;  // negative
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    if (brush == 1) gfxPixel(x0, y0, c);
    else gfxFill(x0, y0, brush, brush, c);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void gfxLine(int x0, int y0, int x1, int y1, Colour c) { lineWithBrush(x0, y0, x1, y1, c, 1); }

void gfxPlotLine(int x0, int y0, int x1, int y1, Colour c) { lineWithBrush(x0, y0, x1, y1, c, 2); }

int gfxTextWidth(const char* s, int scale) {
  if (scale < kMinTextScale) scale = kMinTextScale;
  int n = 0;
  while (s[n]) ++n;
  return n == 0 ? 0 : (n * (kFontW + 1) - 1) * scale;
}

void gfxText(int x, int y, const char* s, Colour c, int scale) {
  if (scale < kMinTextScale) scale = kMinTextScale;
  for (; *s; ++s, x += (kFontW + 1) * scale) {
    int ch = (unsigned char)*s;
    if (ch < kFontFirst || ch > kFontLast) ch = '?';
    const uint8_t* glyph = kFont5x7 + (ch - kFontFirst) * kFontW;
    for (int col = 0; col < kFontW; ++col) {
      uint8_t bits = glyph[col];
      for (int row = 0; row < 8; ++row)
        if (bits & (1 << row)) gfxFill(x + col * scale, y + row * scale, scale, scale, c);
    }
  }
}

void gfxTextRight(int xRight, int y, const char* s, Colour c, int scale) {
  gfxText(xRight - gfxTextWidth(s, scale), y, s, c, scale);
}

void gfxTextCentre(int xCentre, int y, const char* s, Colour c, int scale) {
  gfxText(xCentre - gfxTextWidth(s, scale) / 2, y, s, c, scale);
}

}  // namespace fw
