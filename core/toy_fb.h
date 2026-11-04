/* toy_fb.h - the framebuffer contract shared by device and emulator.
 *
 * Square RGB565; size is a runtime property (240 or 360 today).
 * Origin top-left. Cartridges draw through these helpers in
 * framebuffer coordinates derived from centre + radius fractions;
 * the round-glass clip is the presentation layer's problem.
 */
#ifndef TOY_FB_H
#define TOY_FB_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int w, h;
    uint16_t *px; /* w*h, row-major, RGB565 */
} toy_fb_t;

static inline uint16_t toy_rgb565(unsigned r, unsigned g, unsigned b)
{
    return (uint16_t)(((r & 0xf8u) << 8) | ((g & 0xfcu) << 3) | (b >> 3));
}
static inline int toy_cx(const toy_fb_t *fb) { return fb->w / 2; }
static inline int toy_cy(const toy_fb_t *fb) { return fb->h / 2; }
static inline int toy_r(const toy_fb_t *fb) { return (fb->w < fb->h ? fb->w : fb->h) / 2; }

void toy_fb_clear(toy_fb_t *fb, uint16_t color);
void toy_fb_px(toy_fb_t *fb, int x, int y, uint16_t color);
void toy_fb_line(toy_fb_t *fb, int x0, int y0, int x1, int y1, uint16_t color);
void toy_fb_disc(toy_fb_t *fb, int cx, int cy, int r, uint16_t color);
void toy_fb_ring(toy_fb_t *fb, int cx, int cy, int r, uint16_t color);
/* 3x5 micro font, scale >= 1. Returns the x after the last glyph. */
int toy_fb_text(toy_fb_t *fb, int x, int y, int scale, uint16_t color, const char *s);
/* Dim every pixel toward bg by num/256 (trail fade helper). */
void toy_fb_fade(toy_fb_t *fb, uint16_t bg, unsigned num);

#endif
