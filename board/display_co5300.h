/* display_co5300.h - CO5300 AMOLED command/framing core (write-only).
 *
 * Platform-independent: this layer turns register writes, address
 * windows and RGB565 pixel streams into the CO5300 Quad-SPI logical
 * byte stream and hands it to a caller-supplied sink. The RP2350
 * PIO transport (display_qspi_pio.c) is one sink; the host test
 * (tools/display_test.c) uses a capturing fake sink. No Pico SDK
 * types appear here.
 *
 * Wire contract (CO5300 datasheet V0.01, section 5.2.1/5.2.2):
 *   - Register/parameter write: instruction 02h, then a 24-bit
 *     address {00h, REG, 00h}, then parameter bytes. All 1-wire
 *     (SIO0 only), MSB first, one CS assertion.
 *   - Pixel write: instruction 32h, address {00h, 2Ch (RAMWR),
 *     00h} 1-wire, then pixel data 4-wire in the same CS assertion.
 *   - RGB565 pixel byte order on the wire: high byte
 *     (RRRRRGGG) first, then low byte (GGGBBBBB). Each byte goes
 *     out high nibble first across SI3..SI0 (transport's job).
 *     Callers pass host-order uint16_t RGB565 (as in core/toy_fb.h);
 *     this layer performs the byte ordering.
 *
 * Panel geometry: the fitted 1.75" module is 466x466 visible
 * (Waveshare product page for RP2350-Touch-AMOLED-1.75). The
 * CO5300 source RAM is wider than the glass; the vendor's board
 * example (examples/C/01_GUI/lib/AMOLED/AMOLED_1in75.c,
 * AMOLED_1IN75_SetWindows) offsets columns by +6 and rows by 0.
 * Those offsets live here as named constants so the display-test
 * border pattern can convict them if they are wrong.
 */
#ifndef DISPLAY_CO5300_H
#define DISPLAY_CO5300_H

#include <stddef.h>
#include <stdint.h>

#define CO5300_WIDTH 466
#define CO5300_HEIGHT 466
#define CO5300_COL_OFFSET 6 /* vendor board example, SetWindows */
#define CO5300_ROW_OFFSET 0 /* vendor board example, SetWindows */

#define CO5300_OK 0
#define CO5300_ERR_ARG (-1)    /* NULL device/sink/pixel pointer */
#define CO5300_ERR_WINDOW (-2) /* window outside the 466x466 glass */
#define CO5300_ERR_COUNT (-3)  /* pixel count != window area */
#define CO5300_ERR_STATE (-4)  /* stream call out of sequence */

/* Transport sink. All callbacks are required.
 *   cs          asserted != 0 pulls CS low, 0 releases it high.
 *               The release call must not raise CS until every
 *               previously handed byte has left the pins.
 *   cmd_bytes   1-wire logical bytes (instructions, addresses,
 *               parameters), MSB-first on SIO0.
 *   pixel_bytes 4-wire logical bytes (pixel data only).
 *   reset_pin   drives the panel RESX pin to level.
 *   delay_ms    blocks for at least the given milliseconds. */
typedef struct {
    void *ctx;
    void (*cs)(void *ctx, int asserted);
    void (*cmd_bytes)(void *ctx, const uint8_t *bytes, size_t len);
    void (*pixel_bytes)(void *ctx, const uint8_t *bytes, size_t len);
    void (*reset_pin)(void *ctx, int level);
    void (*delay_ms)(void *ctx, uint32_t ms);
} co5300_sink_t;

typedef struct {
    co5300_sink_t sink;
    uint8_t brightness; /* last DBV written via 51h */
    /* Open pixel-stream state (co5300_stream_* below). Owned
     * by the driver; callers must not touch these fields. */
    size_t stream_remaining; /* pixels still owed to the window */
    int stream_open;
} co5300_t;

/* Bind a sink. Returns CO5300_ERR_ARG if any callback is NULL. */
int co5300_bind(co5300_t *dev, const co5300_sink_t *sink);

/* Hardware reset pulse + the board's register init sequence.
 * initial_brightness is the raw DBV (00h..FFh) written by the
 * sequence's 51h step; the display ends ON at that brightness
 * with undefined GRAM, so fill immediately after calling this. */
int co5300_init(co5300_t *dev, uint8_t initial_brightness);

int co5300_display_on(co5300_t *dev);  /* 29h DISPON */
int co5300_display_off(co5300_t *dev); /* 28h DISPOFF */
/* Raw display brightness value, 51h DBV: 00h lowest, FFh highest
 * (datasheet 7.5.38). Takes effect because init sets WRCTRLD
 * BCTRL=1 (53h = 20h, datasheet 7.5.40). */
int co5300_set_brightness(co5300_t *dev, uint8_t dbv);

/* Address window, logical glass coordinates, inclusive ends:
 * 0 <= x0 <= x1 < CO5300_WIDTH, same for y. Emits CASET (2Ah)
 * and RASET (2Bh) with the board offsets applied. */
int co5300_set_window(co5300_t *dev, int x0, int y0, int x1, int y1);

/* Set the window, then stream count host-order RGB565 pixels
 * (row-major over the window). count must equal the window area.
 * Conversion happens in a small fixed chunk buffer; no
 * framebuffer is needed beyond the caller's source rows. */
int co5300_write_pixels(co5300_t *dev, int x0, int y0, int x1, int y1,
                        const uint16_t *pixels, size_t count);

/* Streaming pixel write, for callers that produce a window's
 * pixels incrementally (e.g. the presenter expanding one row at
 * a time): begin sets the window and opens ONE RAMWR stream;
 * pixels appends host-order RGB565 pixels in row-major order
 * over any number of calls; end closes the stream. The wire
 * output is byte-identical to one co5300_fill-shaped full-window
 * stream (CASET, RASET, one CS-held RAMWR), which is the shape
 * proven on glass; it must not be emulated with per-row
 * write_pixels calls (the toy5-presenter black-glass bug: 466
 * short streams per frame never landed on glass). The caller
 * owes exactly the window area in pixels across pixels calls:
 * an overrunning call is rejected whole (CO5300_ERR_COUNT,
 * nothing written), and end with pixels still owed closes the
 * stream (CS released) and returns CO5300_ERR_COUNT. Calls out
 * of sequence return CO5300_ERR_STATE and emit nothing. While a
 * stream is open, no other driver call may be made on the
 * device. */
int co5300_stream_begin(co5300_t *dev, int x0, int y0, int x1, int y1);
int co5300_stream_pixels(co5300_t *dev, const uint16_t *pixels,
                         size_t count);
int co5300_stream_end(co5300_t *dev);

/* Set the window, then stream one solid RGB565 colour over it.
 * Streams from a small pattern buffer; a full-screen fill needs
 * no framebuffer. */
int co5300_fill(co5300_t *dev, int x0, int y0, int x1, int y1,
                uint16_t rgb565);

/* Expand one 1-wire logical byte for a quad output stage that
 * emits FIFO bytes high-nibble-first onto pins SI3..SI0 (nibble
 * bit j drives pin SIj): produces 4 bytes whose successive
 * nibbles carry the logical byte MSB-first on SI0, zeros on
 * SI1..SI3. Pure helper shared by the PIO transport and tests. */
static inline void co5300_expand_cmd_byte(uint8_t byte, uint8_t out[4])
{
    for (int k = 0; k < 4; k++) {
        unsigned hi = (byte >> (7 - 2 * k)) & 1u;
        unsigned lo = (byte >> (6 - 2 * k)) & 1u;
        out[k] = (uint8_t)((hi << 4) | lo);
    }
}

#endif
