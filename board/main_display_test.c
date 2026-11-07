/* main_display_test.c - TOY-DISP D0 "lite-the-panel" firmware.
 *
 * Standalone bring-up for the Waveshare RP2350-Touch-AMOLED-1.75
 * CO5300 panel; separate from the toy_board proof firmware and
 * touches no cartridge code. Sequence over USB CDC serial:
 *   banner -> read-only I2C1 scan -> panel init at conservative
 *   brightness -> TE observation -> diagnostic patterns ->
 *   final low-brightness pattern -> display off, idle.
 * Every phase prints a TOY-DISP-* marker so a black screen can be
 * triaged from the serial log alone (see board/DISPLAY.md).
 */
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/unique_id.h"

#include "display_co5300.h"
#include "display_qspi_pio.h"

#ifndef TOY_GIT_HEAD
#define TOY_GIT_HEAD "unknown"
#endif

/* Shared board I2C: I2C1 on GPIO6 (SDA) / GPIO7 (SCL) per the
 * schematic and vendor DEV_Config.h (DEV_SDA_PIN 6, DEV_SCL_PIN
 * 7, I2C_PORT i2c1 at 400 kHz). Carries the CST9217 touch,
 * QMI8658 IMU, PCF85063 RTC, AXP2101 PMIC and ES8311 codec. */
#define DISP_I2C_PORT i2c1
#define DISP_I2C_SDA 6
#define DISP_I2C_SCL 7

/* Raw DBV brightness steps (51h): boot dim, patterns modest,
 * final hold dimmer. Nothing here sits at full white. */
#define DBV_INIT 0x30u
#define DBV_PATTERN 0x50u
#define DBV_FINAL 0x20u

static co5300_t panel;

/* Read-only bus scan: current-address 1-byte reads with a short
 * timeout, skipping reserved address ranges. Reports every
 * device that ACKs; writes to no register on any device. This
 * runs before panel init so a dead I2C device roster is on the
 * record even if the panel is black. */
static void i2c_scan(void)
{
    i2c_init(DISP_I2C_PORT, 400 * 1000);
    gpio_set_function(DISP_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(DISP_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(DISP_I2C_SDA);
    gpio_pull_up(DISP_I2C_SCL);

    int found = 0;
    for (int addr = 0; addr < 128; addr++) {
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78)
            continue; /* reserved: 0000xxx and 1111xxx */
        uint8_t byte;
        absolute_time_t deadline = make_timeout_time_ms(2);
        int rc = i2c_read_blocking_until(DISP_I2C_PORT, (uint8_t)addr,
                                         &byte, 1, false, deadline);
        if (rc >= 0) {
            printf("TOY-DISP-I2C addr=0x%02x\n", addr);
            found++;
        }
    }
    printf("TOY-DISP-I2C-SCAN-DONE count=%d\n", found);
}

/* TE (GPIO17) is a panel output; observe only. Init programs
 * STESL N=471 with TEON M=0 (TE mode 3), so a live panel pulses
 * TE once per frame. Count edges for a fixed window. */
static void te_observe(void)
{
    gpio_init(DISPLAY_PIN_TE);
    gpio_set_dir(DISPLAY_PIN_TE, GPIO_IN);
    int prev = gpio_get(DISPLAY_PIN_TE) ? 1 : 0;
    unsigned edges = 0;
    absolute_time_t end = make_timeout_time_ms(300);
    while (!time_reached(end)) {
        int now = gpio_get(DISPLAY_PIN_TE) ? 1 : 0;
        if (now != prev) {
            edges++;
            prev = now;
        }
    }
    printf("TOY-DISP-TE edges=%u level=%d (300 ms window)\n", edges, prev);
}

/* Solid full-screen fill with a timing print. */
static void pattern_fill(const char *name, uint16_t rgb565)
{
    printf("TOY-DISP-PATTERN name=%s\n", name);
    uint64_t t0 = time_us_64();
    int rc = co5300_fill(&panel, 0, 0, CO5300_WIDTH - 1,
                         CO5300_HEIGHT - 1, rgb565);
    uint64_t us = time_us_64() - t0;
    printf("TOY-DISP-TIME pattern=%s rc=%d full_frame_us=%llu\n", name,
           rc, (unsigned long long)us);
}

/* Filled rectangle, inclusive ends, no timing line. */
static void rect(int x0, int y0, int x1, int y1, uint16_t rgb565)
{
    int rc = co5300_fill(&panel, x0, y0, x1, y1, rgb565);
    if (rc)
        printf("TOY-DISP-RECT-ERR rc=%d\n", rc);
}

/* TL red, TR green, BL blue, BR yellow: convicts RGB/BGR order
 * and X/Y mirroring in one look. */
static void pattern_quadrants(void)
{
    printf("TOY-DISP-PATTERN name=quadrants\n");
    int mx = CO5300_WIDTH / 2, my = CO5300_HEIGHT / 2;
    rect(0, 0, mx - 1, my - 1, 0xf800u);                 /* TL red */
    rect(mx, 0, CO5300_WIDTH - 1, my - 1, 0x07e0u);      /* TR green */
    rect(0, my, mx - 1, CO5300_HEIGHT - 1, 0x001fu);     /* BL blue */
    rect(mx, my, CO5300_WIDTH - 1, CO5300_HEIGHT - 1,
         0xffe0u);                                       /* BR yellow */
}

/* White border hard against the glass rectangle's edges, plus a
 * cyan border inset by 8 px: convicts column/row offsets and
 * clipping. On round glass the corners fall off the visible
 * disc; the edge midpoints are the telltales. */
static void pattern_border(void)
{
    printf("TOY-DISP-PATTERN name=border\n");
    pattern_fill("border-bg", 0x0000u);
    int w = CO5300_WIDTH - 1, h = CO5300_HEIGHT - 1;
    rect(0, 0, w, 1, 0xffffu);
    rect(0, h - 1, w, h, 0xffffu);
    rect(0, 0, 1, h, 0xffffu);
    rect(w - 1, 0, w, h, 0xffffu);
    rect(8, 8, w - 8, 9, 0x07ffu);
    rect(8, h - 9, w - 8, h - 8, 0x07ffu);
    rect(8, 8, 9, h - 8, 0x07ffu);
    rect(w - 9, 8, w - 8, h - 8, 0x07ffu);
}

static int isqrt_int(int v)
{
    int r = 0;
    while ((r + 1) * (r + 1) <= v)
        r++;
    return r;
}

/* Centred cross through the full width/height plus a centred
 * magenta disc of radius 60: convicts centring and symmetry. */
static void pattern_cross_disc(void)
{
    printf("TOY-DISP-PATTERN name=cross-disc\n");
    pattern_fill("cross-bg", 0x0000u);
    int cx = CO5300_WIDTH / 2, cy = CO5300_HEIGHT / 2;
    rect(cx - 1, 0, cx + 1, CO5300_HEIGHT - 1, 0xffffu);
    rect(0, cy - 1, CO5300_WIDTH - 1, cy + 1, 0xffffu);
    const int r = 60;
    for (int dy = -r; dy <= r; dy++) {
        int dx = isqrt_int(r * r - dy * dy);
        rect(cx - dx, cy + dy, cx + dx, cy + dy, 0xf81fu);
    }
}

int main(void)
{
    stdio_init_all();
    /* Wait for the host to attach before the banner (max ~15 s),
     * same posture as main_board.c. No LED on this board. */
    for (int i = 0; i < 150 && !stdio_usb_connected(); i++)
        sleep_ms(100);
    sleep_ms(200);

    char idstr[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    pico_get_unique_board_id_string(idstr, sizeof idstr);
    printf("TOY-DISP-BOOT git %s serial %s clk_hz %lu\n", TOY_GIT_HEAD,
           idstr, (unsigned long)clock_get_hz(clk_sys));

    printf("TOY-DISP-PHASE i2c-scan (read-only)\n");
    i2c_scan();

    printf("TOY-DISP-PHASE panel-init\n");
    display_qspi_t transport;
    if (display_qspi_init(&transport) != 0) {
        printf("TOY-DISP-ERR transport-init\n");
        for (;;)
            sleep_ms(1000);
    }
    printf("TOY-DISP-QSPI scl_hz=%lu cs=%d scl=%d sio=%d..%d rst=%d te=%d\n",
           (unsigned long)display_qspi_scl_hz(&transport), DISPLAY_PIN_CS,
           DISPLAY_PIN_SCL, DISPLAY_PIN_SIO0, DISPLAY_PIN_SIO0 + 3,
           DISPLAY_PIN_RST, DISPLAY_PIN_TE);
    co5300_sink_t sink;
    display_qspi_sink(&transport, &sink);
    int rc = co5300_bind(&panel, &sink);
    if (rc == CO5300_OK)
        rc = co5300_init(&panel, DBV_INIT);
    printf("TOY-DISP-INIT rc=%d dbv=0x%02x\n", rc, DBV_INIT);
    if (rc != CO5300_OK) {
        for (;;)
            sleep_ms(1000);
    }

    /* GRAM is undefined after init; the first act is black. */
    pattern_fill("black", 0x0000u);
    te_observe();

    co5300_set_brightness(&panel, DBV_PATTERN);
    printf("TOY-DISP-BRIGHTNESS dbv=0x%02x\n", DBV_PATTERN);
    pattern_fill("red", 0xf800u);
    sleep_ms(1200);
    pattern_fill("green", 0x07e0u);
    sleep_ms(1200);
    pattern_fill("blue", 0x001fu);
    sleep_ms(1200);
    pattern_fill("grey", 0x8410u);
    sleep_ms(1200);
    pattern_quadrants();
    sleep_ms(1500);
    pattern_border();
    sleep_ms(1500);
    pattern_cross_disc();
    sleep_ms(1500);

    /* Final diagnostic: border + cross + disc held dim, then off.
     * Total lit time is bounded (~35 s from init). */
    pattern_border();
    {
        int cx = CO5300_WIDTH / 2, cy = CO5300_HEIGHT / 2;
        rect(cx - 1, 0, cx + 1, CO5300_HEIGHT - 1, 0xffffu);
        rect(0, cy - 1, CO5300_WIDTH - 1, cy + 1, 0xffffu);
    }
    co5300_set_brightness(&panel, DBV_FINAL);
    printf("TOY-DISP-FINAL dbv=0x%02x hold_ms=20000\n", DBV_FINAL);
    sleep_ms(20000);

    co5300_display_off(&panel);
    printf("TOY-DISP-OFF done\n");
    printf("TOY-DISP-DONE\n");
    for (;;)
        sleep_ms(1000);
}
