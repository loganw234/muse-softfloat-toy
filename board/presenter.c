/* presenter.c - see presenter.h for the contract and the
 * framebuffer RAM decision. Integer math only. */
#include "presenter.h"

/* The one persistent framebuffer. Static storage so its cost is
 * a single countable .bss line in the linker map (the parcel's
 * RAM evidence quotes this symbol). */
static uint16_t presenter_px[TOY_PRESENTER_FB_PIXELS];

void toy_presenter_build_map(uint16_t *map, int src_size, int dst_size)
{
    for (int d = 0; d < dst_size; d++)
        map[d] = (uint16_t)toy_presenter_src_coord(d, src_size, dst_size);
}

void toy_presenter_init(toy_presenter_t *p, co5300_t *panel)
{
    p->panel = panel;
    p->fb.w = TOY_PRESENTER_FB_W;
    p->fb.h = TOY_PRESENTER_FB_H;
    p->fb.px = presenter_px;
    /* The only divisions in the presenter run here, once:
     * 466 column + 466 row map entries from the pinned
     * formula. Flush is table lookup only (toy5-pace). */
    toy_presenter_build_map(p->col_src, p->fb.w, CO5300_WIDTH);
    toy_presenter_build_map(p->row_src, p->fb.h, CO5300_HEIGHT);
    p->rotation = 0;
    p->cached_key = -1;
}

toy_fb_t *toy_presenter_fb(toy_presenter_t *p)
{
    return &p->fb;
}

void toy_presenter_set_rotation(toy_presenter_t *p, int quadrant)
{
    p->rotation = toy_orient_norm(quadrant);
    /* The cache resets at every flush start anyway; clear it
     * here too so no reader can mistake a stale key for valid
     * between the set and the next flush. */
    p->cached_key = -1;
}

int toy_presenter_src_coord(int dst, int src_size, int dst_size)
{
    /* ((2d+1) * S) / (2D). Max numerator at 466: 931*466 fits
     * comfortably in int; sizes here are <= 466. */
    return ((2 * dst + 1) * src_size) / (2 * dst_size);
}

void toy_presenter_expand_row(const uint16_t *src_row,
                              const uint16_t *col_src,
                              uint16_t *dst_row, int dst_w)
{
    for (int dx = 0; dx < dst_w; dx++)
        dst_row[dx] = src_row[col_src[dx]];
}

void toy_presenter_rot_src(int rotation, int dx, int dy,
                           const uint16_t *col_src,
                           const uint16_t *row_src, int dst_size,
                           int *sx, int *sy)
{
    int32_t cx, cy;
    toy_orient_inv_point(rotation, dx, dy, dst_size, &cx, &cy);
    *sx = col_src[cx];
    *sy = row_src[cy];
}

int toy_presenter_rot_cache_key(int rotation, int dy,
                                const uint16_t *col_src,
                                const uint16_t *row_src, int dst_size)
{
    switch (toy_orient_norm(rotation)) {
    case 1:  return col_src[dst_size - 1 - dy]; /* source column */
    case 2:  return row_src[dst_size - 1 - dy]; /* source row */
    case 3:  return col_src[dy];                /* source column */
    default: return row_src[dy];                /* source row */
    }
}

void toy_presenter_build_glass_row(const uint16_t *fb_px, int fb_w,
                                   const uint16_t *col_src,
                                   const uint16_t *row_src,
                                   int rotation, int dy,
                                   uint16_t *dst_row, int dst_size)
{
    switch (toy_orient_norm(rotation)) {
    case 1: {
        /* (cx,cy) = (N-1-dy, dx): constant source column,
         * source row walks the row map across dx. */
        int sx = col_src[dst_size - 1 - dy];
        for (int dx = 0; dx < dst_size; dx++)
            dst_row[dx] = fb_px[(size_t)row_src[dx] * (size_t)fb_w +
                                (size_t)sx];
        break;
    }
    case 2: {
        /* (cx,cy) = (N-1-dx, N-1-dy): mapped source row,
         * column map read backwards. */
        const uint16_t *src_row =
            &fb_px[(size_t)row_src[dst_size - 1 - dy] * (size_t)fb_w];
        for (int dx = 0; dx < dst_size; dx++)
            dst_row[dx] = src_row[col_src[dst_size - 1 - dx]];
        break;
    }
    case 3: {
        /* (cx,cy) = (dy, N-1-dx): constant source column,
         * source row walks the row map backwards. */
        int sx = col_src[dy];
        for (int dx = 0; dx < dst_size; dx++)
            dst_row[dx] =
                fb_px[(size_t)row_src[dst_size - 1 - dx] *
                          (size_t)fb_w +
                      (size_t)sx];
        break;
    }
    default:
        /* q=0: the pre-rotation expansion, verbatim. */
        toy_presenter_expand_row(
            &fb_px[(size_t)row_src[dy] * (size_t)fb_w], col_src,
            dst_row, dst_size);
        break;
    }
}

/* Row-built flush: ONE full-glass window and ONE CS-held RAMWR
 * stream per frame, with built rows streamed inside it - the
 * exact wire shape of the glass-proven co5300_fill. (The first
 * shipped version issued one 466x1 write_pixels window per
 * row, 466 streams per frame, and the glass stayed black
 * while every call returned CO5300_OK: see the CS-drain note
 * in display_qspi_pio.c. Do not go back to per-row windows;
 * tools/presenter_test.c pins the single-stream framing.)
 * Used always in 360-upscale mode, and in FULL466 mode
 * whenever rotation != 0 (there the maps are identity, so
 * this is a pure pixel permutation). */
static int presenter_flush_streamed(toy_presenter_t *p)
{
    int rc = co5300_stream_begin(p->panel, 0, 0, CO5300_WIDTH - 1,
                                 CO5300_HEIGHT - 1);
    if (rc != CO5300_OK)
        return rc;
    /* Row cache is per-flush only: the framebuffer may have
     * changed since the last flush, but it cannot change
     * during this one (flush never writes it, single thread),
     * and neither can the rotation, so a repeated effective
     * key may reuse the built row. */
    p->cached_key = -1;
    for (int dy = 0; dy < CO5300_HEIGHT; dy++) {
        int key = toy_presenter_rot_cache_key(p->rotation, dy,
                                              p->col_src, p->row_src,
                                              CO5300_WIDTH);
        if (key != p->cached_key) {
            toy_presenter_build_glass_row(p->fb.px, p->fb.w, p->col_src,
                                          p->row_src, p->rotation,
                                          dy, p->row, CO5300_WIDTH);
            p->cached_key = key;
        }
        rc = co5300_stream_pixels(p->panel, p->row, CO5300_WIDTH);
        if (rc != CO5300_OK)
            break;
    }
    int erc = co5300_stream_end(p->panel);
    return rc != CO5300_OK ? rc : erc;
}

int toy_presenter_flush(toy_presenter_t *p)
{
#if TOY_PRESENTER_MODE_FULL466
    if (p->rotation == 0)
        return co5300_write_pixels(p->panel, 0, 0, CO5300_WIDTH - 1,
                                   CO5300_HEIGHT - 1, p->fb.px,
                                   TOY_PRESENTER_FB_PIXELS);
    return presenter_flush_streamed(p);
#else
    return presenter_flush_streamed(p);
#endif
}

int toy_presenter_blank(toy_presenter_t *p)
{
    return co5300_fill(p->panel, 0, 0, CO5300_WIDTH - 1,
                       CO5300_HEIGHT - 1, 0x0000u);
}
