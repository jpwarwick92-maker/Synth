#include "gfx.h"

#include "font5x7.h"

namespace fw {

FW_BIGMEM uint16_t gFrame[kDisplayW * kDisplayH];

void gfxClear(uint16_t c) {
  for (int i = 0; i < kDisplayW * kDisplayH; ++i) gFrame[i] = c;
}

void gfxPixel(int x, int y, uint16_t c) {
  if ((unsigned)x < (unsigned)kDisplayW && (unsigned)y < (unsigned)kDisplayH) gFrame[y * kDisplayW + x] = c;
}

void gfxFill(int x, int y, int w, int h, uint16_t c) {
  int x1 = x + w, y1 = y + h;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x1 > kDisplayW) x1 = kDisplayW;
  if (y1 > kDisplayH) y1 = kDisplayH;
  for (int yy = y; yy < y1; ++yy) {
    uint16_t* row = gFrame + yy * kDisplayW;
    for (int xx = x; xx < x1; ++xx) row[xx] = c;
  }
}

void gfxHLine(int x, int y, int w, uint16_t c) { gfxFill(x, y, w, 1, c); }
void gfxVLine(int x, int y, int h, uint16_t c) { gfxFill(x, y, 1, h, c); }

void gfxFrame(int x, int y, int w, int h, uint16_t c) {
  gfxHLine(x, y, w, c);
  gfxHLine(x, y + h - 1, w, c);
  gfxVLine(x, y, h, c);
  gfxVLine(x + w - 1, y, h, c);
}

// Bresenham, stamping a brush x brush square at every step.
static void lineWithBrush(int x0, int y0, int x1, int y1, uint16_t c, int brush) {
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

void gfxLine(int x0, int y0, int x1, int y1, uint16_t c) { lineWithBrush(x0, y0, x1, y1, c, 1); }

void gfxPlotLine(int x0, int y0, int x1, int y1, uint16_t c) { lineWithBrush(x0, y0, x1, y1, c, 2); }

int gfxTextWidth(const char* s, int scale) {
  if (scale < kMinTextScale) scale = kMinTextScale;
  int n = 0;
  while (s[n]) ++n;
  return n == 0 ? 0 : (n * (kFontW + 1) - 1) * scale;
}

void gfxText(int x, int y, const char* s, uint16_t c, int scale) {
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

void gfxTextRight(int xRight, int y, const char* s, uint16_t c, int scale) {
  gfxText(xRight - gfxTextWidth(s, scale), y, s, c, scale);
}

void gfxTextCentre(int xCentre, int y, const char* s, uint16_t c, int scale) {
  gfxText(xCentre - gfxTextWidth(s, scale) / 2, y, s, c, scale);
}

}  // namespace fw
