/* presenter.h - on-glass presenter: cart framebuffer -> CO5300.
 *
 * The presenter owns the ONE persistent toy_fb_t that cartridges
 * render into. Persistence is contractual (emu/main_emu.c frames
 * mode never clears between frames, and core/toy_fb.c's
 * toy_fb_fade is a read-modify-write over the whole fb), so the
 * same pixel storage survives untouched between frames within a
 * cart; the runner clears it only on a cart switch, and flush
 * never modifies it. Measured nuance (toy5-presenter): every
 * current cart's render() in fact clears and redraws - trail
 * carts keep trails in state ring buffers - so persistence is
 * a forward contract, preserved here at zero cost.
 * This layer never writes cart state and never alters a
 * rendered pixel: in full mode the flush is a verbatim copy to
 * the wire; in upscale mode it is a pinned nearest-neighbour
 * expansion (below).
 *
 * Framebuffer RAM decision (toy5-presenter, measured; full
 * numbers in the parcel report):
 *   (a) full 466x466 fb in SRAM: 434,312 B. REJECTED: this same
 *       source built with TOY_PRESENTER_MODE_FULL466=1 does not
 *       link - .data 6,060 B + .bss 518,496 B overshoot the
 *       512 KiB main RAM region (linker: "region RAM overflowed
 *       by 2088 bytes", zero heap free, against the >= 20 KiB
 *       free requirement; core-0 stack is separately reserved
 *       4,096 B in SCRATCH_Y).
 *   (b) 360x360 fb (259,200 B, the size the emulator already
 *       renders, persistence intact) + integer nearest-
 *       neighbour upscale to 466x466 during flush. CHOSEN
 *       (default): .bss 343,384 B; 173,032 B main RAM free
 *       after .data/.bss and the 2,048 B heap reserve.
 *   (c) banded rendering: rejected. Either every band's pixels
 *       persist between frames - storing all bands is exactly
 *       (a) with extra steps - or render() is replayed per
 *       band, which needs a clip/offset window toy_fb does not
 *       have (toy_fb_px clips only against the whole fb) and
 *       would break the persistence contract above plus pixel
 *       identity with the emulator. Core changes for negative
 *       benefit once (b) measured this comfortable.
 *
 * Upscale mapping (mode b), centre-aligned nearest neighbour,
 * integer only. For destination coordinate d in [0,466), source
 * size S=360, destination size D=466:
 *     src(d) = ((2*d + 1) * S) / (2*D)      (integer division)
 * i.e. the source pixel whose centre is nearest the destination
 * pixel's centre. src(0)=0, src(465)=359; the map is monotone,
 * covers every source pixel at least once (S < D), and each
 * source pixel covers 1 or 2 destination pixels. The same map
 * is applied on both axes, so the 360 square fills the 466
 * square exactly - no letterbox, no offset to centre (both
 * squares share the glass centre by construction: src centre
 * 179.5 maps between dst 232/233, the glass centre pair).
 * tools/presenter_test.c pins this mapping pixel-for-pixel.
 *
 * Pacing (toy5-pace): the per-pixel division is gone from the
 * flush path. toy_presenter_init precomputes the 466-entry
 * column map and row map from the pinned formula above (932
 * divisions, once); row expansion and row selection during
 * flush are table lookups. Consecutive glass rows that map to
 * the same source row (466-360 = 106 of them) reuse the last
 * expanded row; the cache is reset at the start of every
 * flush, so it can never serve a stale row across frames.
 */
#ifndef PRESENTER_H
#define PRESENTER_H

#include <stddef.h>
#include <stdint.h>

#include "display_co5300.h"
#include "toy_fb.h"

/* Mode select. Default (b) 360-upscale per the measurement
 * above; build with -DTOY_PRESENTER_MODE_FULL466=1 to reproduce
 * the rejected option (a) map numbers. */
#ifndef TOY_PRESENTER_MODE_FULL466
#define TOY_PRESENTER_MODE_FULL466 0
#endif

#if TOY_PRESENTER_MODE_FULL466
#define TOY_PRESENTER_FB_W CO5300_WIDTH
#define TOY_PRESENTER_FB_H CO5300_HEIGHT
#define TOY_PRESENTER_MODE_NAME "466-full"
#else
#define TOY_PRESENTER_FB_W 360
#define TOY_PRESENTER_FB_H 360
#define TOY_PRESENTER_MODE_NAME "360-upscale"
#endif

#define TOY_PRESENTER_FB_PIXELS \
    ((size_t)TOY_PRESENTER_FB_W * (size_t)TOY_PRESENTER_FB_H)
#define TOY_PRESENTER_FB_BYTES (TOY_PRESENTER_FB_PIXELS * sizeof(uint16_t))

typedef struct {
    co5300_t *panel;
    toy_fb_t fb;
    /* One expanded glass row (upscale flush); unused in full
     * mode but kept so the struct is mode-independent. */
    uint16_t row[CO5300_WIDTH];
    /* Precomputed source maps (toy5-pace): col_src[dx] and
     * row_src[dy] hold toy_presenter_src_coord for every glass
     * coordinate, built once by toy_presenter_init. Identity
     * in full mode. 2 * 466 * 2 = 1,864 B. */
    uint16_t col_src[CO5300_WIDTH];
    uint16_t row_src[CO5300_HEIGHT];
    /* Source row currently expanded in row[], or -1. Reset at
     * the start of every flush; within one flush the
     * framebuffer does not change, so consecutive glass rows
     * sharing a source row reuse row[] verbatim. */
    int cached_sy;
} toy_presenter_t;

/* Bind to an initialised panel and point fb at the presenter's
 * static pixel storage (zeroed by .bss; caller clears). */
void toy_presenter_init(toy_presenter_t *p, co5300_t *panel);

/* The framebuffer carts render into. Persistent across frames;
 * the runner owns clearing policy. */
toy_fb_t *toy_presenter_fb(toy_presenter_t *p);

/* Flush the whole framebuffer to the full glass. CO5300_OK or a
 * driver error. Pixel-exact per the header contract. */
int toy_presenter_flush(toy_presenter_t *p);

/* Solid black over the full glass via the driver's streaming
 * fill (cart-switch blanking; does not touch the framebuffer). */
int toy_presenter_blank(toy_presenter_t *p);

/* Pure mapping helpers (host-testable, no panel involved).
 * Centre-aligned nearest neighbour; when src_size == dst_size
 * this is the identity. Pre: 0 <= dst < dst_size, sizes > 0.
 * This stays the one pinned formula: the precomputed maps are
 * built from it and the host test proves them equal to it. */
int toy_presenter_src_coord(int dst, int src_size, int dst_size);

/* Fill map[0..dst_size) with toy_presenter_src_coord(d,
 * src_size, dst_size). Runs the division once per destination
 * coordinate at init so the flush path never divides. */
void toy_presenter_build_map(uint16_t *map, int src_size, int dst_size);

/* Expand one source row into one destination row (dst_w
 * pixels) by table lookup: dst_row[dx] = src_row[col_src[dx]].
 * No division. col_src comes from toy_presenter_build_map (in
 * practice toy_presenter_t.col_src). */
void toy_presenter_expand_row(const uint16_t *src_row,
                              const uint16_t *col_src,
                              uint16_t *dst_row, int dst_w);

#endif
