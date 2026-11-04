/* toy_fb.c - framebuffer primitives. Display-only code: native
 * integer math throughout; never touches sim state. */
#include "toy_fb.h"

void toy_fb_clear(toy_fb_t *fb, uint16_t color)
{
    size_t n = (size_t)fb->w * (size_t)fb->h;
    for (size_t i = 0; i < n; i++) fb->px[i] = color;
}

void toy_fb_px(toy_fb_t *fb, int x, int y, uint16_t color)
{
    if ((unsigned)x < (unsigned)fb->w && (unsigned)y < (unsigned)fb->h)
        fb->px[(size_t)y * (size_t)fb->w + (size_t)x] = color;
}

void toy_fb_line(toy_fb_t *fb, int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int sx = x0 < x1 ? 1 : -1;
    int dy = -(y1 > y0 ? y1 - y0 : y0 - y1);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        toy_fb_px(fb, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void toy_fb_disc(toy_fb_t *fb, int cx, int cy, int r, uint16_t color)
{
    for (int y = -r; y <= r; y++)
        for (int x = -r; x <= r; x++)
            if (x * x + y * y <= r * r) toy_fb_px(fb, cx + x, cy + y, color);
}

void toy_fb_ring(toy_fb_t *fb, int cx, int cy, int r, uint16_t color)
{
    int x = r, y = 0, err = 0;
    while (x >= y) {
        toy_fb_px(fb, cx + x, cy + y, color); toy_fb_px(fb, cx + y, cy + x, color);
        toy_fb_px(fb, cx - y, cy + x, color); toy_fb_px(fb, cx - x, cy + y, color);
        toy_fb_px(fb, cx - x, cy - y, color); toy_fb_px(fb, cx - y, cy - x, color);
        toy_fb_px(fb, cx + y, cy - x, color); toy_fb_px(fb, cx + x, cy - y, color);
        y++;
        if (err <= 0) err += 2 * y + 1;
        if (err > 0) { x--; err -= 2 * x + 1; }
    }
}

void toy_fb_fade(toy_fb_t *fb, uint16_t bg, unsigned num)
{
    int br = (int)((bg >> 11) & 31u), bgc = (int)((bg >> 5) & 63u), bb = (int)(bg & 31u);
    size_t n = (size_t)fb->w * (size_t)fb->h;
    for (size_t i = 0; i < n; i++) {
        unsigned p = fb->px[i];
        int r = (int)((p >> 11) & 31u), g = (int)((p >> 5) & 63u), b = (int)(p & 31u);
        r -= ((r - br) * (int)num) >> 8;
        g -= ((g - bgc) * (int)num) >> 8;
        b -= ((b - bb) * (int)num) >> 8;
        fb->px[i] = (uint16_t)(((unsigned)r << 11) | ((unsigned)g << 5) | (unsigned)b);
    }
}

/* 3x5 micro font. 15 bits per glyph, row-major top-to-bottom,
 * bit 14 is the top-left pixel. Covers space, A-Z, 0-9 and a few
 * marks; unknown glyphs render as space. */
static uint16_t glyph3x5(char c)
{
    switch (c) {
    case 'A': return 0x2BEDu;
    case 'B': return 0x6BAEu;
    case 'C': return 0x3923u;
    case 'D': return 0x6B6Eu;
    case 'E': return 0x79A7u;
    case 'F': return 0x79A4u;
    case 'G': return 0x396Bu;
    case 'H': return 0x5BEDu;
    case 'I': return 0x7497u;
    case 'J': return 0x126Au;
    case 'K': return 0x5BADu;
    case 'L': return 0x4927u;
    case 'M': return 0x5FEDu;
    case 'N': return 0x5FFDu;
    case 'O': return 0x2B6Au;
    case 'P': return 0x6BA4u;
    case 'Q': return 0x2B73u;
    case 'R': return 0x6BADu;
    case 'S': return 0x388Eu;
    case 'T': return 0x7492u;
    case 'U': return 0x5B6Fu;
    case 'V': return 0x5B6Au;
    case 'W': return 0x5BFDu;
    case 'X': return 0x5AADu;
    case 'Y': return 0x5A92u;
    case 'Z': return 0x72A7u;
    case '0': return 0x7B6Fu;
    case '1': return 0x2C97u;
    case '2': return 0x62A7u;
    case '3': return 0x628Eu;
    case '4': return 0x5BC9u;
    case '5': return 0x798Eu;
    case '6': return 0x39AAu;
    case '7': return 0x7292u;
    case '8': return 0x2AAAu;
    case '9': return 0x2ACEu;
    case '.': return 0x0002u;
    case '-': return 0x01C0u;
    case ':': return 0x0410u;
    case '+': return 0x05D0u;
    case '/': return 0x12A4u;
    case '=': return 0x0E38u;
    case '%': return 0x52A5u;
    case '(': return 0x1511u;
    case ')': return 0x4454u;
    case '#': return 0x5F7Du;
    case '*': return 0x0AE8u;
    case '<': return 0x1511u;
    case '>': return 0x4454u;
    case '_': return 0x0007u;
    default:  return 0;
    }
}

int toy_fb_text(toy_fb_t *fb, int x, int y, int scale, uint16_t color, const char *s)
{
    if (scale < 1) scale = 1;
    for (; *s; s++) {
        uint16_t g = glyph3x5(*s);
        for (int row = 0; row < 5; row++)
            for (int col = 0; col < 3; col++)
                if (g & (uint16_t)(1u << (14 - (row * 3 + col))))
                    for (int dy = 0; dy < scale; dy++)
                        for (int dx = 0; dx < scale; dx++)
                            toy_fb_px(fb, x + col * scale + dx, y + row * scale + dy, color);
        x += 4 * scale;
    }
    return x;
}
