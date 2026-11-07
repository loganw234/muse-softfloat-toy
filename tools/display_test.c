/* display_test.c - host test for the CO5300 framing core.
 *
 * Drives board/display_co5300.c against a capturing fake sink and
 * checks the logical wire stream: init sequence and delays,
 * window offsets/clipping, RGB565 wire byte order, the streamed
 * (co5300_stream_*) single-CS framing and its sequencing/count
 * discipline, framebuffer-free fill chunking, and the 1-wire
 * command byte expansion used by the PIO transport.
 *
 * Build (from repo root):
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard \
 *       tools/display_test.c board/display_co5300.c \
 *       -o build/display_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display_co5300.h"

#define CAP_BYTES (1024u * 1024u)

typedef struct {
    uint8_t cmd[CAP_BYTES];
    size_t cmd_len;
    uint8_t pix[CAP_BYTES];
    size_t pix_len;
    int cs_level;
    unsigned cs_asserts;
    unsigned cs_releases;
    int rst_level;
    unsigned rst_changes;
    uint32_t delays[64];
    unsigned ndelays;
    size_t max_pix_chunk;
    int overflow;
} fake_t;

static fake_t fake;
static co5300_t dev;
static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static void fake_reset(void) { memset(&fake, 0, sizeof fake); }

static void f_cs(void *ctx, int asserted)
{
    fake_t *f = (fake_t *)ctx;
    if (asserted) {
        f->cs_asserts++;
        f->cs_level = 1;
    } else {
        f->cs_releases++;
        f->cs_level = 0;
    }
}

static void f_cmd(void *ctx, const uint8_t *bytes, size_t len)
{
    fake_t *f = (fake_t *)ctx;
    if (!f->cs_level)
        f->overflow = 1; /* bytes outside a CS assertion: bug */
    if (f->cmd_len + len > sizeof f->cmd) {
        f->overflow = 1;
        return;
    }
    memcpy(f->cmd + f->cmd_len, bytes, len);
    f->cmd_len += len;
}

static void f_pix(void *ctx, const uint8_t *bytes, size_t len)
{
    fake_t *f = (fake_t *)ctx;
    if (!f->cs_level)
        f->overflow = 1;
    if (len > f->max_pix_chunk)
        f->max_pix_chunk = len;
    if (f->pix_len + len > sizeof f->pix) {
        f->overflow = 1;
        return;
    }
    memcpy(f->pix + f->pix_len, bytes, len);
    f->pix_len += len;
}

static void f_rst(void *ctx, int level)
{
    fake_t *f = (fake_t *)ctx;
    f->rst_level = level;
    f->rst_changes++;
}

static void f_delay(void *ctx, uint32_t ms)
{
    fake_t *f = (fake_t *)ctx;
    if (f->ndelays < sizeof f->delays / sizeof f->delays[0])
        f->delays[f->ndelays] = ms;
    f->ndelays++;
}

static void bind_fake(void)
{
    co5300_sink_t sink;
    sink.ctx = &fake;
    sink.cs = f_cs;
    sink.cmd_bytes = f_cmd;
    sink.pixel_bytes = f_pix;
    sink.reset_pin = f_rst;
    sink.delay_ms = f_delay;
    CHECK(co5300_bind(&dev, &sink) == CO5300_OK);
}

/* Expected register-write framing: {02, 00, reg, 00, params...}. */
static void expect_cmd_stream(const uint8_t *want, size_t n)
{
    CHECK(fake.cmd_len == n);
    if (fake.cmd_len == n)
        CHECK(memcmp(fake.cmd, want, n) == 0);
}

static void test_bind_rejects(void)
{
    co5300_t d;
    co5300_sink_t s;
    memset(&s, 0, sizeof s);
    CHECK(co5300_bind(&d, &s) == CO5300_ERR_ARG);
    CHECK(co5300_bind(NULL, &s) == CO5300_ERR_ARG);
    CHECK(co5300_init(&d, 0x40) == CO5300_ERR_ARG);
}

static void test_init_sequence(void)
{
    fake_reset();
    bind_fake();
    CHECK(co5300_init(&dev, 0x30) == CO5300_OK);

    /* Reset pulse: high, low, high (3 drives), ends high. */
    CHECK(fake.rst_changes == 3);
    CHECK(fake.rst_level == 1);
    /* Delays: 50/50/300 reset, 120 after SLPOUT, 10 after 53h,
     * 10 after 29h. */
    static const uint32_t want_delays[] = { 50, 50, 300, 120, 10, 10 };
    CHECK(fake.ndelays == 6);
    if (fake.ndelays == 6)
        CHECK(memcmp(fake.delays, want_delays, sizeof want_delays) == 0);

    static const uint8_t want[] = {
        0x02, 0x00, 0x11, 0x00,                   /* SLPOUT */
        0x02, 0x00, 0xc4, 0x00, 0x80,             /* SPI_WRAM on */
        0x02, 0x00, 0x44, 0x00, 0x01, 0xd7,       /* TE scanline 471 */
        0x02, 0x00, 0x35, 0x00, 0x00,             /* TEON M=0 */
        0x02, 0x00, 0x53, 0x00, 0x20,             /* BCTRL on */
        0x02, 0x00, 0x29, 0x00,                   /* DISPON */
        0x02, 0x00, 0x51, 0x00, 0x30,             /* DBV = caller's */
        0x02, 0x00, 0x20, 0x00,                   /* INVOFF */
        0x02, 0x00, 0x36, 0x00, 0x00,             /* MADCTL RGB */
        0x02, 0x00, 0x3a, 0x00, 0x05,             /* COLMOD RGB565 */
    };
    expect_cmd_stream(want, sizeof want);
    CHECK(fake.cs_asserts == 10 && fake.cs_releases == 10);
    CHECK(fake.pix_len == 0);
    CHECK(!fake.overflow);
    CHECK(dev.brightness == 0x30);
}

static void test_brightness_and_onoff(void)
{
    fake_reset();
    bind_fake();
    CHECK(co5300_set_brightness(&dev, 0xab) == CO5300_OK);
    CHECK(co5300_display_off(&dev) == CO5300_OK);
    CHECK(co5300_display_on(&dev) == CO5300_OK);
    static const uint8_t want[] = {
        0x02, 0x00, 0x51, 0x00, 0xab,
        0x02, 0x00, 0x28, 0x00,
        0x02, 0x00, 0x29, 0x00,
    };
    expect_cmd_stream(want, sizeof want);
    CHECK(dev.brightness == 0xab);
}

static void test_window_offsets(void)
{
    fake_reset();
    bind_fake();
    /* Full glass: columns 6..471 in RAM coords, rows 0..465. */
    CHECK(co5300_set_window(&dev, 0, 0, 465, 465) == CO5300_OK);
    static const uint8_t want_full[] = {
        0x02, 0x00, 0x2a, 0x00, 0x00, 0x06, 0x01, 0xd7,
        0x02, 0x00, 0x2b, 0x00, 0x00, 0x00, 0x01, 0xd1,
    };
    expect_cmd_stream(want_full, sizeof want_full);

    fake_reset();
    bind_fake();
    CHECK(co5300_set_window(&dev, 10, 20, 11, 21) == CO5300_OK);
    static const uint8_t want_small[] = {
        0x02, 0x00, 0x2a, 0x00, 0x00, 0x10, 0x00, 0x11, /* 16..17 */
        0x02, 0x00, 0x2b, 0x00, 0x00, 0x14, 0x00, 0x15, /* 20..21 */
    };
    expect_cmd_stream(want_small, sizeof want_small);
}

static void test_window_rejects(void)
{
    fake_reset();
    bind_fake();
    CHECK(co5300_set_window(&dev, -1, 0, 5, 5) == CO5300_ERR_WINDOW);
    CHECK(co5300_set_window(&dev, 0, 0, 466, 5) == CO5300_ERR_WINDOW);
    CHECK(co5300_set_window(&dev, 0, 0, 5, 466) == CO5300_ERR_WINDOW);
    CHECK(co5300_set_window(&dev, 5, 5, 4, 5) == CO5300_ERR_WINDOW);
    CHECK(co5300_set_window(&dev, 5, 5, 5, 4) == CO5300_ERR_WINDOW);
    /* Rejected windows emit nothing at all. */
    CHECK(fake.cmd_len == 0 && fake.pix_len == 0);
    CHECK(fake.cs_asserts == 0);
}

static void test_write_pixels_byte_order(void)
{
    fake_reset();
    bind_fake();
    const uint16_t px[4] = { 0xf800, 0x07e0, 0x001f, 0x1234 };
    CHECK(co5300_write_pixels(&dev, 0, 0, 3, 0, px, 4) == CO5300_OK);
    /* Window first, then the 32h/RAMWR header as cmd bytes. */
    static const uint8_t want_cmd[] = {
        0x02, 0x00, 0x2a, 0x00, 0x00, 0x06, 0x00, 0x09,
        0x02, 0x00, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x32, 0x00, 0x2c, 0x00,
    };
    expect_cmd_stream(want_cmd, sizeof want_cmd);
    /* High byte first per pixel. */
    static const uint8_t want_pix[] = { 0xf8, 0x00, 0x07, 0xe0,
                                        0x00, 0x1f, 0x12, 0x34 };
    CHECK(fake.pix_len == sizeof want_pix);
    if (fake.pix_len == sizeof want_pix)
        CHECK(memcmp(fake.pix, want_pix, sizeof want_pix) == 0);

    /* Count must match the window area exactly. */
    fake_reset();
    bind_fake();
    CHECK(co5300_write_pixels(&dev, 0, 0, 3, 0, px, 3) ==
          CO5300_ERR_COUNT);
    CHECK(fake.cmd_len == 0 && fake.pix_len == 0);
    CHECK(co5300_write_pixels(&dev, 0, 0, 3, 0, NULL, 4) ==
          CO5300_ERR_ARG);
}

/* Streaming API (toy5-presenter black-glass fix): begin+pixels
 * in several calls+end must emit EXACTLY the same logical wire
 * stream as one co5300_write_pixels over the same window and
 * pixels (one CASET, one RASET, one CS-held RAMWR), because
 * that single-stream shape is what the glass-proven fill uses
 * and what the presenter now depends on. */
static void test_stream_matches_write_pixels(void)
{
    static uint8_t ref_cmd[256], ref_pix[256];
    size_t ref_cmd_len, ref_pix_len;
    unsigned ref_asserts, ref_releases;
    const uint16_t px[8] = { 0xf800, 0x07e0, 0x001f, 0x1234,
                             0xffff, 0x0000, 0x8410, 0xffe0 };

    fake_reset();
    bind_fake();
    CHECK(co5300_write_pixels(&dev, 2, 3, 5, 4, px, 8) == CO5300_OK);
    CHECK(fake.cmd_len <= sizeof ref_cmd && fake.pix_len <= sizeof ref_pix);
    memcpy(ref_cmd, fake.cmd, fake.cmd_len);
    memcpy(ref_pix, fake.pix, fake.pix_len);
    ref_cmd_len = fake.cmd_len;
    ref_pix_len = fake.pix_len;
    ref_asserts = fake.cs_asserts;
    ref_releases = fake.cs_releases;
    CHECK(ref_asserts == 3 && ref_releases == 3); /* CASET,RASET,RAMWR */

    fake_reset();
    bind_fake();
    CHECK(co5300_stream_begin(&dev, 2, 3, 5, 4) == CO5300_OK);
    /* Irregular split, including a zero-length call. */
    CHECK(co5300_stream_pixels(&dev, px, 3) == CO5300_OK);
    CHECK(co5300_stream_pixels(&dev, px + 3, 0) == CO5300_OK);
    CHECK(co5300_stream_pixels(&dev, px + 3, 1) == CO5300_OK);
    CHECK(co5300_stream_pixels(&dev, px + 4, 4) == CO5300_OK);
    CHECK(co5300_stream_end(&dev) == CO5300_OK);
    CHECK(fake.cmd_len == ref_cmd_len && fake.pix_len == ref_pix_len);
    if (fake.cmd_len == ref_cmd_len)
        CHECK(memcmp(fake.cmd, ref_cmd, ref_cmd_len) == 0);
    if (fake.pix_len == ref_pix_len)
        CHECK(memcmp(fake.pix, ref_pix, ref_pix_len) == 0);
    CHECK(fake.cs_asserts == ref_asserts);
    CHECK(fake.cs_releases == ref_releases);
    CHECK(!fake.overflow);
}

/* Stream sequencing and count discipline: out-of-sequence and
 * overrunning calls emit nothing and never leave CS dangling,
 * so a presenter bug cannot wedge the panel interface. */
static void test_stream_state_errors(void)
{
    const uint16_t px[4] = { 1, 2, 3, 4 };

    fake_reset();
    bind_fake();
    CHECK(co5300_stream_pixels(&dev, px, 1) == CO5300_ERR_STATE);
    CHECK(co5300_stream_end(&dev) == CO5300_ERR_STATE);
    CHECK(fake.cmd_len == 0 && fake.pix_len == 0);
    CHECK(fake.cs_asserts == 0 && fake.cs_releases == 0);

    /* Bad window rejected before anything is emitted. */
    CHECK(co5300_stream_begin(&dev, 0, 0, 466, 0) == CO5300_ERR_WINDOW);
    CHECK(fake.cmd_len == 0 && fake.cs_asserts == 0);

    CHECK(co5300_stream_begin(&dev, 0, 0, 3, 0) == CO5300_OK);
    /* Double begin and interleaved write_pixels refused. */
    CHECK(co5300_stream_begin(&dev, 0, 0, 3, 0) == CO5300_ERR_STATE);
    CHECK(co5300_write_pixels(&dev, 0, 0, 3, 0, px, 4) ==
          CO5300_ERR_STATE);
    /* Overrun rejected whole: nothing written for that call. */
    CHECK(co5300_stream_pixels(&dev, px, 3) == CO5300_OK);
    size_t pix_before = fake.pix_len;
    CHECK(co5300_stream_pixels(&dev, px, 2) == CO5300_ERR_COUNT);
    CHECK(fake.pix_len == pix_before);
    /* Early end still releases CS and reports the shortfall. */
    CHECK(co5300_stream_end(&dev) == CO5300_ERR_COUNT);
    CHECK(fake.cs_asserts == fake.cs_releases);
    CHECK(fake.cs_level == 0);
    /* Device usable again immediately. */
    CHECK(co5300_fill(&dev, 0, 0, 3, 0, 0xffff) == CO5300_OK);
    CHECK(fake.cs_asserts == fake.cs_releases);
    CHECK(!fake.overflow);
}

static void test_fill_streams_without_framebuffer(void)
{
    fake_reset();
    bind_fake();
    CHECK(co5300_fill(&dev, 0, 0, 465, 465, 0x8410) == CO5300_OK);
    size_t want = (size_t)466 * 466 * 2;
    CHECK(fake.pix_len == want);
    /* Chunked: no single sink call carries the whole frame. */
    CHECK(fake.max_pix_chunk > 0 && fake.max_pix_chunk <= 256);
    int uniform = 1;
    for (size_t i = 0; i + 1 < fake.pix_len; i += 2)
        if (fake.pix[i] != 0x84 || fake.pix[i + 1] != 0x10)
            uniform = 0;
    CHECK(uniform);
    CHECK(!fake.overflow);

    /* Odd-sized window lands exactly. */
    fake_reset();
    bind_fake();
    CHECK(co5300_fill(&dev, 3, 5, 4, 7, 0xffff) == CO5300_OK);
    CHECK(fake.pix_len == (size_t)2 * 3 * 2);
}

/* Independent decode: walk the 4 expanded bytes, take each
 * nibble's bit0 in output order, rebuild the logical byte. */
static uint8_t decode_expanded(const uint8_t out[4])
{
    uint8_t v = 0;
    for (int i = 0; i < 4; i++) {
        v = (uint8_t)((v << 1) | ((out[i] >> 4) & 1u));
        v = (uint8_t)((v << 1) | (out[i] & 1u));
        /* SI1..SI3 must stay low in both nibbles. */
        CHECK((out[i] & 0xe0u) == 0 && (out[i] & 0x0eu) == 0);
    }
    return v;
}

static void test_expand_cmd_byte(void)
{
    for (int b = 0; b < 256; b++) {
        uint8_t out[4];
        co5300_expand_cmd_byte((uint8_t)b, out);
        CHECK(decode_expanded(out) == (uint8_t)b);
    }
}

int main(void)
{
    test_bind_rejects();
    test_init_sequence();
    test_brightness_and_onoff();
    test_window_offsets();
    test_window_rejects();
    test_write_pixels_byte_order();
    test_stream_matches_write_pixels();
    test_stream_state_errors();
    test_fill_streams_without_framebuffer();
    test_expand_cmd_byte();
    if (failures) {
        printf("display_test: FAIL (%d failures)\n", failures);
        return EXIT_FAILURE;
    }
    printf("display_test: PASS\n");
    return EXIT_SUCCESS;
}
