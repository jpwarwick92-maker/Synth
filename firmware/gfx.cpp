#include "gfx.h"

#include "font5x7.h"

namespace fw {

namespace {
constexpr uint16_t rgb565(int r, int g, int b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
}  // namespace

const uint16_t kPalettes565[PALETTE_COUNT][16] = {
    {
        // PALETTE_AMBER
        rgb565(0, 0, 0),           // kBlack
        rgb565(0xFF, 0xE9, 0xC4),  // kInk: cream
        rgb565(0xFF, 0x6A, 0x1F),  // kAccent: hot orange
        rgb565(0xFF, 0xB0, 0x00),  // kLine: amber
        rgb565(0x96, 0x65, 0x0A),  // kLineDim
        rgb565(0xA8, 0x87, 0x5E),  // kGrey: warm grey
        rgb565(0x4B, 0x3A, 0x26),  // kDarkGrey
        rgb565(0x2B, 0x20, 0x15),  // kGrid
    },
    {
        // PALETTE_WHITE: told apart by brightness; the accent leans red, which amber acrylic lets through
        rgb565(0, 0, 0),
        rgb565(0xFF, 0xFF, 0xFF),
        rgb565(0xFF, 0x60, 0x40),
        rgb565(0xD8, 0xD4, 0xCC),
        rgb565(0x78, 0x76, 0x72),
        rgb565(0x98, 0x96, 0x90),
        rgb565(0x3C, 0x3B, 0x39),
        rgb565(0x22, 0x21, 0x20),
    },
};
const char* const kPaletteNames[PALETTE_COUNT] = {"AMBER", "WHITE"};

static uint8_t gPaletteIndex = PALETTE_AMBER;

void gfxSetPalette(int palette) { gPaletteIndex = (uint8_t)(palette >= 0 && palette < PALETTE_COUNT ? palette : PALETTE_AMBER); }
int gfxPalette() { return gPaletteIndex; }
const uint16_t* gfxPalette565() { return kPalettes565[gPaletteIndex]; }

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
  const uint16_t* pal = kPalettes565[gPaletteIndex];
  for (int i = 0; i < kDisplayW / 2; ++i) {
    dst[2 * i] = pal[src[i] >> 4];
    dst[2 * i + 1] = pal[src[i] & 0x0F];
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
