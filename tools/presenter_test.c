/* presenter_test.c - host test for the on-glass presenter
 * (board/presenter.*) in its shipped 360-upscale mode.
 *
 * Pins, without any hardware:
 *   1. the centre-aligned nearest-neighbour coordinate map
 *      against an independent floating-point formulation AND
 *      hand-computed spot values (edges, centre pair);
 *   2. crafted-pixel expansion: 1px features at corners, edge
 *      and centre land on exactly their mapped destination
 *      footprint - no more, no fewer - through the real
 *      co5300_stream_* wire path into a capturing sink, with
 *      the flush framing pinned to a single RAMWR stream;
 *   3. persistence: flush never modifies the framebuffer and
 *      toy_fb_fade accumulates across flushes exactly as on a
 *      plain framebuffer; plus a harmonograph stepped+rendered
 *      for 30 frames into the presenter framebuffer (never
 *      cleared, the firmware contract) is pixel-identical every
 *      frame to the same sequence rendered into a plain
 *      reference framebuffer (emu-frames runner shape);
 *   4. end-to-end flush: the reconstructed 466x466 glass image
 *      equals the reference 360 framebuffer sampled through the
 *      independently formulated map, for harmonograph and fig8;
 *   5. the precomputed col/row maps (toy5-pace): every one of
 *      the 466 entries equals both the pinned integer formula
 *      and the independent double formulation, expand_row via
 *      the map is pixel-identical to the direct formula, a
 *      full-frame pattern flushed through the map matches the
 *      independent sampling, and the repeated-row cache never
 *      serves a stale row across flushes.
 *
 * Build (from repo root, Git Bash): gcc -O2 -std=c11 -Wall
 * -Wextra -Werror, includes libs/muse-softfloat/include, core
 * and board; sources are this file, board/presenter.c,
 * board/display_co5300.c, every C file under core and under
 * libs/muse-softfloat/src; output build/presenter_test.
 * (If the link wants toy_clock_*, add emu/clock_emu.c - core
 * references the clock only through runner-owned calls.)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "presenter.h"
#include "toy_cart.h"

static int failures;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
            failures++;                                                      \
        }                                                                    \
    } while (0)

/* Independent map formulation (host-only double math): the
 * source index whose pixel centre is nearest the destination
 * pixel's centre is floor((dst + 0.5) * S / D). */
static int map_ref(int dst, int src, int dstn)
{
    return (int)(((double)dst + 0.5) * (double)src / (double)dstn);
}

static void test_src_coord(void)
{
    CHECK(TOY_PRESENTER_FB_W == 360 && TOY_PRESENTER_FB_H == 360);
    CHECK(TOY_PRESENTER_FB_BYTES == (size_t)360 * 360 * 2);

    /* Identity when sizes match (the full-466 mode's map). */
    for (int d = 0; d < 466; d++)
        CHECK(toy_presenter_src_coord(d, 466, 466) == d);

    /* Hand-computed spot values of ((2d+1)*360)/932. */
    CHECK(toy_presenter_src_coord(0, 360, 466) == 0);
    CHECK(toy_presenter_src_coord(1, 360, 466) == 1);   /* 1080/932 */
    CHECK(toy_presenter_src_coord(2, 360, 466) == 1);   /* 1800/932 */
    CHECK(toy_presenter_src_coord(3, 360, 466) == 2);   /* 2520/932 */
    CHECK(toy_presenter_src_coord(100, 360, 466) == 77); /* 72360/932 */
    CHECK(toy_presenter_src_coord(232, 360, 466) == 179);
    CHECK(toy_presenter_src_coord(233, 360, 466) == 180);
    CHECK(toy_presenter_src_coord(464, 360, 466) == 358);
    CHECK(toy_presenter_src_coord(465, 360, 466) == 359);

    /* Whole-range agreement with the independent formulation,
     * monotonicity, and 1-or-2 coverage of every source pixel. */
    int cover[360] = { 0 };
    int prev = -1;
    for (int d = 0; d < 466; d++) {
        int s = toy_presenter_src_coord(d, 360, 466);
        CHECK(s == map_ref(d, 360, 466));
        CHECK(s >= prev && s >= 0 && s < 360);
        prev = s;
        cover[s]++;
    }
    for (int s = 0; s < 360; s++)
        CHECK(cover[s] == 1 || cover[s] == 2);
}

/* --- capturing co5300 sink (shape mirrors tools/display_test.c) */
#define CAP_PIX ((size_t)466 * 466 * 2)
static uint8_t cap_pix[CAP_PIX];
static size_t cap_len;
static int cap_cs, cap_overflow, cap_windows;

static void cap_cs_fn(void *ctx, int asserted)
{
    (void)ctx;
    cap_cs = asserted ? 1 : 0;
}
static void cap_cmd_fn(void *ctx, const uint8_t *bytes, size_t len)
{
    (void)ctx; (void)bytes; (void)len;
    if (!cap_cs)
        cap_overflow = 1;
    /* Count RAMWR pixel-stream headers (32 00 2c 00). Flushing
     * must be ONE stream per frame (glass-proven fill shape);
     * the original per-row framing issued 466 and the glass
     * stayed black (see presenter.c / display_qspi_pio.c). */
    if (len == 4 && bytes[0] == 0x32 && bytes[2] == 0x2c)
        cap_windows++;
}
static void cap_pix_fn(void *ctx, const uint8_t *bytes, size_t len)
{
    (void)ctx;
    if (!cap_cs || cap_len + len > sizeof cap_pix) {
        cap_overflow = 1;
        return;
    }
    memcpy(cap_pix + cap_len, bytes, len);
    cap_len += len;
}
static void cap_rst_fn(void *ctx, int level) { (void)ctx; (void)level; }
static void cap_dly_fn(void *ctx, uint32_t ms) { (void)ctx; (void)ms; }

static co5300_t panel;
static toy_presenter_t presenter;

static void setup_presenter(void)
{
    co5300_sink_t sink;
    sink.ctx = NULL;
    sink.cs = cap_cs_fn;
    sink.cmd_bytes = cap_cmd_fn;
    sink.pixel_bytes = cap_pix_fn;
    sink.reset_pin = cap_rst_fn;
    sink.delay_ms = cap_dly_fn;
    CHECK(co5300_bind(&panel, &sink) == CO5300_OK);
    toy_presenter_init(&presenter, &panel);
}

/* Glass pixel (x,y) from the captured wire stream. Upscale
 * flush streams expanded rows top-to-bottom inside one
 * full-window RAMWR, so the pixel byte stream is the 466x466
 * image row-major, high byte first (driver contract). */
static uint16_t glass_px(int x, int y)
{
    size_t i = ((size_t)y * 466 + (size_t)x) * 2;
    return (uint16_t)(((unsigned)cap_pix[i] << 8) | cap_pix[i + 1]);
}

static void flush_capture(void)
{
    cap_len = 0;
    cap_overflow = 0;
    cap_windows = 0;
    CHECK(toy_presenter_flush(&presenter) == CO5300_OK);
    CHECK(!cap_overflow);
    CHECK(cap_len == CAP_PIX);
    /* Exactly one RAMWR stream per flush. This is the
     * black-glass regression pin: 466 here means the per-row
     * framing that never landed on glass is back. */
    CHECK(cap_windows == 1);
}

/* Whole-image check: glass == src sampled through map_ref. */
static void check_glass_matches(const toy_fb_t *src)
{
    for (int dy = 0; dy < 466; dy++) {
        int sy = map_ref(dy, src->h, 466);
        for (int dx = 0; dx < 466; dx++) {
            int sx = map_ref(dx, src->w, 466);
            uint16_t want = src->px[(size_t)sy * (size_t)src->w + (size_t)sx];
            if (glass_px(dx, dy) != want) {
                printf("FAIL glass(%d,%d)=%04x want src(%d,%d)=%04x\n",
                       dx, dy, glass_px(dx, dy), sx, sy, want);
                failures++;
                return;
            }
        }
    }
}

static void test_crafted_pixels(void)
{
    toy_fb_t *fb = toy_presenter_fb(&presenter);
    static const struct { int x, y; } spots[] = {
        { 0, 0 }, { 359, 0 }, { 0, 359 }, { 359, 359 }, /* corners */
        { 1, 0 }, { 0, 1 },                              /* 1px in */
        { 179, 179 }, { 180, 180 }, { 179, 180 },        /* centre */
        { 77, 100 },                                     /* generic */
    };
    for (size_t k = 0; k < sizeof spots / sizeof spots[0]; k++) {
        toy_fb_clear(fb, 0x0000u);
        toy_fb_px(fb, spots[k].x, spots[k].y, 0xffffu);
        flush_capture();
        /* White footprint must be exactly the mapped preimage. */
        int whites = 0, expect = 0;
        for (int d = 0; d < 466; d++) {
            if (map_ref(d, 360, 466) == spots[k].x)
                expect++;
        }
        int ecols = expect;
        expect = 0;
        for (int d = 0; d < 466; d++) {
            if (map_ref(d, 360, 466) == spots[k].y)
                expect++;
        }
        expect *= ecols;
        for (int y = 0; y < 466; y++)
            for (int x = 0; x < 466; x++)
                if (glass_px(x, y) == 0xffffu)
                    whites++;
                else
                    CHECK(glass_px(x, y) == 0x0000u);
        CHECK(whites == expect);
        CHECK(whites >= 1 && whites <= 4);
        check_glass_matches(fb);
    }

    /* A 1px checkerboard column pair: alternating source pixels
     * must not blend, smear or drop (nearest, not filtered). */
    toy_fb_clear(fb, 0x0000u);
    for (int x = 0; x < 360; x += 2)
        toy_fb_px(fb, x, 42, 0xf800u);
    flush_capture();
    check_glass_matches(fb);
}

/* FNV-1a 64 over framebuffer bytes, low byte then high byte per
 * pixel - the emu shell tour's fbhash method
 * (emu/main_emu.c shell_fb_fnv), reused so hashes here are
 * comparable with that methodology. */
static uint64_t fb_fnv(const toy_fb_t *fb)
{
    uint64_t h = 1469598103934665603ull;
    size_t n = (size_t)fb->w * (size_t)fb->h;
    for (size_t i = 0; i < n; i++) {
        h ^= (uint64_t)(fb->px[i] & 0xffu); h *= 1099511628211ull;
        h ^= (uint64_t)(fb->px[i] >> 8);    h *= 1099511628211ull;
    }
    return h;
}

/* Emu-frames runner shape (emu/main_emu.c `frames`): reset,
 * step-0 proof setup, then per frame demo steps of
 * feed->proof->step and one render into a NEVER-cleared fb.
 * This test uses feed-less, proof-less carts, where the
 * firmware's live posture (step-0 only, empty live queue) and
 * the emu posture coincide step for step. */
static void test_cart_sequence(const char *name, int frames, int want_trails)
{
    const toy_cart_t *cart = toy_cart_by_name(name);
    CHECK(cart != NULL);
    if (!cart)
        return;
    CHECK(cart->feed == NULL && cart->proof_input_count == 0);

    static unsigned char st_a[TOY_MAX_STATE_SIZE];
    static unsigned char st_b[TOY_MAX_STATE_SIZE];
    toy_fb_t *pfb = toy_presenter_fb(&presenter);
    toy_fb_t ref;
    ref.w = ref.h = 360;
    ref.px = (uint16_t *)calloc((size_t)360 * 360, sizeof(uint16_t));
    CHECK(ref.px != NULL);

    memset(st_a, 0, cart->state_size);
    memset(st_b, 0, cart->state_size);
    cart->reset(st_a);
    cart->reset(st_b);
    uint32_t ca = 0, cb = 0;
    toy_apply_proof_at(cart, st_a, 0, &ca); /* firmware: step-0 only */
    toy_apply_proof_at(cart, st_b, 0, &cb); /* emu frames shape */
    toy_fb_clear(pfb, 0x0000u);             /* cart switch only */
    toy_fb_clear(&ref, 0x0000u);

    long abs_b = 0;
    for (int fr = 0; fr < frames; fr++) {
        for (uint32_t s = 0; s < cart->demo_steps_per_frame; s++)
            cart->step(st_a);
        cart->render(st_a, pfb);
        for (uint32_t s = 0; s < cart->demo_steps_per_frame; s++) {
            abs_b++;
            toy_apply_proof_at(cart, st_b, (uint32_t)abs_b, &cb);
            cart->step(st_b);
        }
        cart->render(st_b, &ref);
        CHECK(toy_hash_state(st_a, cart->state_size) ==
              toy_hash_state(st_b, cart->state_size));
        CHECK(fb_fnv(pfb) == fb_fnv(&ref));
        CHECK(memcmp(pfb->px, ref.px, (size_t)360 * 360 * 2) == 0);
    }

    if (want_trails) {
        /* Age-faded trail intermediates exist in the image:
         * lit pixels well below saturation (harmonograph draws
         * its trail polyline dimmed by ring age). */
        int faded = 0;
        for (size_t i = 0; i < (size_t)360 * 360; i++) {
            unsigned p = ref.px[i];
            unsigned r = (p >> 11) & 31u, g = (p >> 5) & 63u, b = p & 31u;
            if (p && r < 28 && g < 56 && b < 28 &&
                (r || g || b))
                faded++;
        }
        CHECK(faded > 100);
    }

    flush_capture();
    check_glass_matches(&ref);
    free(ref.px);
}

/* Presenter-level persistence: flush must not modify the
 * framebuffer, and the core fade helper (toy_fb_fade, the
 * persistence mechanism the runner contract exists for) must
 * accumulate across flushes exactly as it does on a plain
 * framebuffer. Proven here directly because the current trail
 * carts (harmonograph, lorenz, cloud) happen to redraw their
 * trails from state ring buffers each render - observed while
 * writing this test - so no shipped cart would catch a
 * presenter that silently cleared between frames. */
static void test_persistence(void)
{
    toy_fb_t *fb = toy_presenter_fb(&presenter);
    static uint16_t snapshot[360 * 360];
    toy_fb_clear(fb, 0x0000u);
    toy_fb_disc(fb, 180, 180, 40, 0xffffu);
    memcpy(snapshot, fb->px, sizeof snapshot);
    flush_capture();
    CHECK(memcmp(snapshot, fb->px, sizeof snapshot) == 0);
    check_glass_matches(fb);

    /* Fade three times without clearing; each flush must show
     * the dimmer disc, matching the same fades applied to an
     * independent plain framebuffer. */
    toy_fb_t ref;
    ref.w = ref.h = 360;
    ref.px = snapshot; /* reuse storage as the reference image */
    for (int i = 0; i < 3; i++) {
        toy_fb_fade(fb, 0x0000u, 64);
        toy_fb_fade(&ref, 0x0000u, 64);
        CHECK(memcmp(fb->px, ref.px, sizeof snapshot) == 0);
        flush_capture();
        check_glass_matches(&ref);
    }
    /* The disc is dimmed but still present (not cleared). */
    CHECK(fb->px[(size_t)180 * 360 + 180] != 0x0000u);
    CHECK(fb->px[(size_t)180 * 360 + 180] != 0xffffu);
}

/* Precomputed-map path (toy5-pace). The maps are the flush
 * path's only source of coordinates now, so pin them against
 * BOTH references for every glass coordinate, pin expand_row
 * against the direct formula, and pin the row cache: mutate
 * a source row that covers two glass rows between flushes -
 * a cache that survived across flushes would show the old row
 * twice and fail check_glass_matches. */
static void test_col_map(void)
{
    /* Standalone builder and the presenter's own maps. */
    uint16_t map[466];
    toy_presenter_build_map(map, 360, 466);
    int repeats = 0;
    for (int d = 0; d < 466; d++) {
        int formula = toy_presenter_src_coord(d, 360, 466);
        CHECK(map[d] == formula);
        CHECK(map[d] == map_ref(d, 360, 466));
        CHECK(presenter.col_src[d] == formula);
        CHECK(presenter.row_src[d] == formula);
        if (d > 0 && presenter.row_src[d] == presenter.row_src[d - 1])
            repeats++;
    }
    /* Monotone-surjective 360 -> 466: exactly 106 repeated
     * rows for the flush cache to reuse. */
    CHECK(repeats == 106);
    CHECK(presenter.cached_sy == -1);

    /* expand_row via the map == direct formula, every column. */
    uint16_t src[360], dst[466];
    for (int x = 0; x < 360; x++)
        src[x] = (uint16_t)(x * 37u + 11u);
    toy_presenter_expand_row(src, presenter.col_src, dst, 466);
    for (int dx = 0; dx < 466; dx++) {
        CHECK(dst[dx] == src[toy_presenter_src_coord(dx, 360, 466)]);
        CHECK(dst[dx] == src[map_ref(dx, 360, 466)]);
    }

    /* Full-frame pattern through the real flush, then mutate
     * the source row behind a repeated glass-row pair and
     * flush again: the new content must land on both rows. */
    toy_fb_t *fb = toy_presenter_fb(&presenter);
    for (int y = 0; y < 360; y++)
        for (int x = 0; x < 360; x++)
            fb->px[(size_t)y * 360 + (size_t)x] =
                (uint16_t)((unsigned)(x + 1) * 40503u ^
                           (unsigned)(y + 1) * 257u ^ (unsigned)(x ^ y));
    flush_capture();
    check_glass_matches(fb);

    int rep_sy = -1;
    for (int d = 1; d < 466; d++)
        if (presenter.row_src[d] == presenter.row_src[d - 1]) {
            rep_sy = presenter.row_src[d];
            break;
        }
    CHECK(rep_sy >= 0);
    for (int x = 0; x < 360; x++)
        fb->px[(size_t)rep_sy * 360 + (size_t)x] = 0xffe0u;
    flush_capture();
    check_glass_matches(fb);
}

int main(void)
{
    test_src_coord();
    setup_presenter();
    test_col_map();
    test_crafted_pixels();
    test_persistence();
    test_cart_sequence("harmonograph", 30, 1);
    test_cart_sequence("fig8", 10, 0);
    if (failures) {
        printf("presenter_test: FAIL (%d failures)\n", failures);
        return EXIT_FAILURE;
    }
    printf("presenter_test: PASS\n");
    return EXIT_SUCCESS;
}
