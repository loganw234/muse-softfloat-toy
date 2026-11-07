/* presenter.c - see presenter.h for the contract and the
 * framebuffer RAM decision. Integer math only. */
#include "presenter.h"

/* The one persistent framebuffer. Static storage so its cost is
 * a single countable .bss line in the linker map (the parcel's
 * RAM evidence quotes this symbol). */
static uint16_t presenter_px[TOY_PRESENTER_FB_PIXELS];

void toy_presenter_init(toy_presenter_t *p, co5300_t *panel)
{
    p->panel = panel;
    p->fb.w = TOY_PRESENTER_FB_W;
    p->fb.h = TOY_PRESENTER_FB_H;
    p->fb.px = presenter_px;
}

toy_fb_t *toy_presenter_fb(toy_presenter_t *p)
{
    return &p->fb;
}

int toy_presenter_src_coord(int dst, int src_size, int dst_size)
{
    /* ((2d+1) * S) / (2D). Max numerator at 466: 931*466 fits
     * comfortably in int; sizes here are <= 466. */
    return ((2 * dst + 1) * src_size) / (2 * dst_size);
}

void toy_presenter_expand_row(const uint16_t *src_row, int src_w,
                              uint16_t *dst_row, int dst_w)
{
    for (int dx = 0; dx < dst_w; dx++)
        dst_row[dx] = src_row[toy_presenter_src_coord(dx, src_w, dst_w)];
}

int toy_presenter_flush(toy_presenter_t *p)
{
#if TOY_PRESENTER_MODE_FULL466
    return co5300_write_pixels(p->panel, 0, 0, CO5300_WIDTH - 1,
                               CO5300_HEIGHT - 1, p->fb.px,
                               TOY_PRESENTER_FB_PIXELS);
#else
    /* One window per glass row, public driver API only. Cost:
     * 466 x (CASET+RASET+RAMWR = 20 1-wire bytes) ~= 3 ms at
     * 25 MHz on top of the ~35 ms pixel stream; the on-device
     * flush_us pacing line is the authority. */
    for (int dy = 0; dy < CO5300_HEIGHT; dy++) {
        int sy = toy_presenter_src_coord(dy, p->fb.h, CO5300_HEIGHT);
        toy_presenter_expand_row(&p->fb.px[(size_t)sy * (size_t)p->fb.w],
                                 p->fb.w, p->row, CO5300_WIDTH);
        int rc = co5300_write_pixels(p->panel, 0, dy, CO5300_WIDTH - 1, dy,
                                     p->row, CO5300_WIDTH);
        if (rc != CO5300_OK)
            return rc;
    }
    return CO5300_OK;
#endif
}

int toy_presenter_blank(toy_presenter_t *p)
{
    return co5300_fill(p->panel, 0, 0, CO5300_WIDTH - 1,
                       CO5300_HEIGHT - 1, 0x0000u);
}
