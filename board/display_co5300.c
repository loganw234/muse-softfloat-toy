/* display_co5300.c - CO5300 framing core. See the header for the
 * wire contract. Every non-obvious constant below is cited to the
 * CO5300 datasheet (V0.01 2023, Chipone) by section/page, or to
 * the Waveshare board example setting it was taken from:
 *   waveshareteam/RP2350-Touch-AMOLED-1.75,
 *   examples/C/01_GUI/lib/AMOLED/AMOLED_1in75.c ("vendor example"
 *   below), consulted for board-specific hardware facts only.
 */
#include "display_co5300.h"

/* QSPI instruction codes (datasheet 5.2.1, p20). */
#define INSTR_CMD_WRITE 0x02u /* 1-wire command/parameter write */
#define INSTR_PIXEL_WRITE_QUAD 0x32u /* 4-wire pixel write, 24-clk addr */

/* DCS / CO5300 registers used here (datasheet ch. 7.5). */
#define REG_SLPOUT 0x11u  /* 7.5.12 Sleep Out */
#define REG_INVOFF 0x20u  /* Display Inversion Off */
#define REG_DISPOFF 0x28u /* Display Off */
#define REG_DISPON 0x29u  /* Display On */
#define REG_CASET 0x2au   /* Column Address Set */
#define REG_RASET 0x2bu   /* Row Address Set */
#define REG_RAMWR 0x2cu   /* Memory Write */
#define REG_TEON 0x35u    /* Tearing Effect Line On */
#define REG_MADCTL 0x36u  /* 7.5.29 Memory Data Access Control */
#define REG_COLMOD 0x3au  /* 7.5.32 Interface Pixel Format */
#define REG_STESL 0x44u   /* 7.5.35 Set Tear Scanline */
#define REG_WRDISBV 0x51u /* 7.5.38 Write Display Brightness */
#define REG_WRCTRLD 0x53u /* 7.5.40 Write CTRL Display */
#define REG_SET_SPI_MODE 0xc4u /* 7.5.57 Set_SPI Mode */

static int dev_ok(const co5300_t *dev)
{
    return dev && dev->sink.cs && dev->sink.cmd_bytes &&
           dev->sink.pixel_bytes && dev->sink.reset_pin &&
           dev->sink.delay_ms;
}

int co5300_bind(co5300_t *dev, const co5300_sink_t *sink)
{
    if (!dev || !sink)
        return CO5300_ERR_ARG;
    dev->sink = *sink;
    dev->brightness = 0;
    return dev_ok(dev) ? CO5300_OK : CO5300_ERR_ARG;
}

/* One register write: {02h, 00h, REG, 00h, params...} in a single
 * CS assertion (datasheet 5.2.2, p21: AD[23:0] = {00h, CMD, 00h}). */
static void send_reg(co5300_t *dev, uint8_t reg, const uint8_t *params,
                     size_t nparams)
{
    uint8_t hdr[4] = { INSTR_CMD_WRITE, 0x00u, reg, 0x00u };
    dev->sink.cs(dev->sink.ctx, 1);
    dev->sink.cmd_bytes(dev->sink.ctx, hdr, sizeof hdr);
    if (nparams)
        dev->sink.cmd_bytes(dev->sink.ctx, params, nparams);
    dev->sink.cs(dev->sink.ctx, 0);
}

/* Open a pixel stream for the already-set window: instruction
 * 32h + RAMWR address 1-wire, pixel bytes 4-wire follow until
 * pixels_end (datasheet 5.2.1/5.2.2). */
static void pixels_begin(co5300_t *dev)
{
    uint8_t hdr[4] = { INSTR_PIXEL_WRITE_QUAD, 0x00u, REG_RAMWR, 0x00u };
    dev->sink.cs(dev->sink.ctx, 1);
    dev->sink.cmd_bytes(dev->sink.ctx, hdr, sizeof hdr);
}

static void pixels_end(co5300_t *dev)
{
    dev->sink.cs(dev->sink.ctx, 0);
}

/* The board's register init sequence, in the vendor example's
 * order (AMOLED_1IN75_InitReg). Each row: register, parameters,
 * count, delay after (ms). The 51h parameter is a placeholder
 * replaced by co5300_init's brightness argument (the vendor
 * example writes A0h here, then FFh from its main; DBV is a free
 * runtime value per datasheet 7.5.38).
 *
 * Row-by-row provenance:
 *   11h SLPOUT, +120 ms: datasheet 7.5.12 (p145) and the p100
 *     power-on table (SLPOUT->DISPON T4 >= 60 ms); 120 ms is the
 *     vendor example's wait and covers both.
 *   C4h = 80h: datasheet 7.5.57 Set_SPI Mode (p216): bit7
 *     SPI_WRAM = 1 enables SPI writes to frame SRAM. Default is
 *     00h (disabled), so this write is load-bearing.
 *   44h = 01D7h: datasheet 7.5.35 STESL (p180), TE scanline
 *     N = 471, just past the 466 active lines; the value is the
 *     vendor example's board setting.
 *   35h = 00h: TEON mode M=0; with N != 0 this selects TE mode 3,
 *     "TE high at N-th line" (datasheet 5.3, p33 table).
 *   53h = 20h: datasheet 7.5.40 WRCTRLD (p186): bit5 BCTRL=1
 *     (DBV brightness active), bit3 DD=0 (no dimming ramp).
 *   29h DISPON, +10 ms: vendor example's placement/wait. Note
 *     the vendor turns the display on before MADCTL/COLMOD;
 *     kept as-is because it is the order proven on this panel.
 *   51h DBV: see above.
 *   20h INVOFF: no colour inversion.
 *   36h = 00h: datasheet 7.5.29 MADCTL: MY=MX=0 (no mirroring),
 *     RGB order bit = 0 (RGB, not BGR).
 *   3Ah = 05h: datasheet 7.5.32 COLMOD (p176): SPI_IFPF_SEL=0,
 *     IFPF=101b -> 16 bit/pixel RGB565 on the SPI interface.
 */
struct init_step {
    uint8_t reg;
    uint8_t params[2];
    uint8_t nparams;
    uint16_t delay_after_ms;
};
static const struct init_step INIT_STEPS[] = {
    { REG_SLPOUT,       { 0x00, 0x00 }, 0, 120 },
    { REG_SET_SPI_MODE, { 0x80, 0x00 }, 1, 0 },
    { REG_STESL,        { 0x01, 0xd7 }, 2, 0 },
    { REG_TEON,         { 0x00, 0x00 }, 1, 0 },
    { REG_WRCTRLD,      { 0x20, 0x00 }, 1, 10 },
    { REG_DISPON,       { 0x00, 0x00 }, 0, 10 },
    { REG_WRDISBV,      { 0x00, 0x00 }, 1, 0 }, /* param = caller's DBV */
    { REG_INVOFF,       { 0x00, 0x00 }, 0, 0 },
    { REG_MADCTL,       { 0x00, 0x00 }, 1, 0 },
    { REG_COLMOD,       { 0x05, 0x00 }, 1, 0 },
};

int co5300_init(co5300_t *dev, uint8_t initial_brightness)
{
    if (!dev_ok(dev))
        return CO5300_ERR_ARG;

    /* Hardware reset, vendor example's AMOLED_1IN75_Reset timing
     * (high 50 ms, low 50 ms, high, 300 ms settle). Datasheet
     * minima are far smaller: reset low pulse >= 10 us and
     * 5 ms after release before commands (6.6, p116), T2/T3
     * 10 ms each (p100); the vendor waits cover all of them,
     * including "SLPOUT cannot be sent for 120 ms" after a
     * reset applied in sleep-out mode (p116 note 5). */
    dev->sink.reset_pin(dev->sink.ctx, 1);
    dev->sink.delay_ms(dev->sink.ctx, 50);
    dev->sink.reset_pin(dev->sink.ctx, 0);
    dev->sink.delay_ms(dev->sink.ctx, 50);
    dev->sink.reset_pin(dev->sink.ctx, 1);
    dev->sink.delay_ms(dev->sink.ctx, 300);

    for (size_t i = 0; i < sizeof INIT_STEPS / sizeof INIT_STEPS[0]; i++) {
        const struct init_step *s = &INIT_STEPS[i];
        uint8_t params[2] = { s->params[0], s->params[1] };
        if (s->reg == REG_WRDISBV)
            params[0] = initial_brightness;
        send_reg(dev, s->reg, params, s->nparams);
        if (s->delay_after_ms)
            dev->sink.delay_ms(dev->sink.ctx, s->delay_after_ms);
    }
    dev->brightness = initial_brightness;
    return CO5300_OK;
}

int co5300_display_on(co5300_t *dev)
{
    if (!dev_ok(dev))
        return CO5300_ERR_ARG;
    send_reg(dev, REG_DISPON, NULL, 0);
    return CO5300_OK;
}

int co5300_display_off(co5300_t *dev)
{
    if (!dev_ok(dev))
        return CO5300_ERR_ARG;
    send_reg(dev, REG_DISPOFF, NULL, 0);
    return CO5300_OK;
}

int co5300_set_brightness(co5300_t *dev, uint8_t dbv)
{
    if (!dev_ok(dev))
        return CO5300_ERR_ARG;
    send_reg(dev, REG_WRDISBV, &dbv, 1);
    dev->brightness = dbv;
    return CO5300_OK;
}

static int window_ok(int x0, int y0, int x1, int y1)
{
    return x0 >= 0 && y0 >= 0 && x1 >= x0 && y1 >= y0 &&
           x1 < CO5300_WIDTH && y1 < CO5300_HEIGHT;
}

int co5300_set_window(co5300_t *dev, int x0, int y0, int x1, int y1)
{
    if (!dev_ok(dev))
        return CO5300_ERR_ARG;
    if (!window_ok(x0, y0, x1, y1))
        return CO5300_ERR_WINDOW;
    /* CASET/RASET take inclusive 16-bit start/end in RAM
     * coordinates, big-endian (datasheet 7.5 CASET/RASET); the
     * board offsets shift glass coords into RAM coords. */
    unsigned xs = (unsigned)x0 + CO5300_COL_OFFSET;
    unsigned xe = (unsigned)x1 + CO5300_COL_OFFSET;
    unsigned ys = (unsigned)y0 + CO5300_ROW_OFFSET;
    unsigned ye = (unsigned)y1 + CO5300_ROW_OFFSET;
    uint8_t col[4] = { (uint8_t)(xs >> 8), (uint8_t)xs,
                       (uint8_t)(xe >> 8), (uint8_t)xe };
    uint8_t row[4] = { (uint8_t)(ys >> 8), (uint8_t)ys,
                       (uint8_t)(ye >> 8), (uint8_t)ye };
    send_reg(dev, REG_CASET, col, sizeof col);
    send_reg(dev, REG_RASET, row, sizeof row);
    return CO5300_OK;
}

int co5300_write_pixels(co5300_t *dev, int x0, int y0, int x1, int y1,
                        const uint16_t *pixels, size_t count)
{
    if (!dev_ok(dev) || !pixels)
        return CO5300_ERR_ARG;
    if (!window_ok(x0, y0, x1, y1))
        return CO5300_ERR_WINDOW;
    size_t area = (size_t)(x1 - x0 + 1) * (size_t)(y1 - y0 + 1);
    if (count != area)
        return CO5300_ERR_COUNT;
    int rc = co5300_set_window(dev, x0, y0, x1, y1);
    if (rc)
        return rc;
    pixels_begin(dev);
    /* Wire order is high byte first; convert in fixed chunks so
     * arbitrarily large windows need no intermediate buffer. */
    uint8_t chunk[128];
    size_t done = 0;
    while (done < count) {
        size_t npx = count - done;
        if (npx > sizeof chunk / 2)
            npx = sizeof chunk / 2;
        for (size_t i = 0; i < npx; i++) {
            chunk[2 * i] = (uint8_t)(pixels[done + i] >> 8);
            chunk[2 * i + 1] = (uint8_t)pixels[done + i];
        }
        dev->sink.pixel_bytes(dev->sink.ctx, chunk, 2 * npx);
        done += npx;
    }
    pixels_end(dev);
    return CO5300_OK;
}

int co5300_fill(co5300_t *dev, int x0, int y0, int x1, int y1,
                uint16_t rgb565)
{
    if (!dev_ok(dev))
        return CO5300_ERR_ARG;
    if (!window_ok(x0, y0, x1, y1))
        return CO5300_ERR_WINDOW;
    int rc = co5300_set_window(dev, x0, y0, x1, y1);
    if (rc)
        return rc;
    size_t total = (size_t)(x1 - x0 + 1) * (size_t)(y1 - y0 + 1) * 2u;
    uint8_t chunk[256];
    for (size_t i = 0; i < sizeof chunk; i += 2) {
        chunk[i] = (uint8_t)(rgb565 >> 8);
        chunk[i + 1] = (uint8_t)rgb565;
    }
    pixels_begin(dev);
    while (total) {
        size_t n = total < sizeof chunk ? total : sizeof chunk;
        dev->sink.pixel_bytes(dev->sink.ctx, chunk, n);
        total -= n;
    }
    pixels_end(dev);
    return CO5300_OK;
}
