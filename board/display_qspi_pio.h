/* display_qspi_pio.h - RP2350 PIO transport for display_co5300.
 *
 * Waveshare RP2350-Touch-AMOLED-1.75 panel wiring (vendor wiki
 * pinout + schematic nets LCD_CS/LCD_SCL/LCD_SIO0/LCD_SI1..3/
 * LCD_RESET/LCD_TE at connector J3, and the vendor board
 * example's lib/QSPI_PIO/qspi_pio.h pin defines):
 *   CS GPIO10, SCL GPIO11, SIO0 GPIO12, SI1 GPIO13, SI2 GPIO14,
 *   SI3 GPIO15, RESET GPIO16, TE GPIO17.
 * The four data pins are consecutive from GPIO12, which is what a
 * single PIO OUT field needs; the RP2350's hardware SPI cannot do
 * quad output on these pins, so PIO is required.
 *
 * There is no panel power-enable pin (the vendor example's
 * PIN_PWR_EN is -1) and no PMIC setup: see DISPLAY.md for the
 * schematic evidence that the panel runs from VCC3V3.
 */
#ifndef DISPLAY_QSPI_PIO_H
#define DISPLAY_QSPI_PIO_H

#include <stdint.h>

#include "display_co5300.h"

#define DISPLAY_PIN_CS 10
#define DISPLAY_PIN_SCL 11
#define DISPLAY_PIN_SIO0 12 /* SI1..SI3 are SIO0+1 .. SIO0+3 */
#define DISPLAY_PIN_RST 16
#define DISPLAY_PIN_TE 17

/* Target SCL rate: the CO5300 datasheet cap. Datasheet
 * V0.00 2023, section 6.4 AC Timings (p107): TSCYC clock
 * cycle (Write) minimum 20 ns and Note 3 verbatim "The max
 * SCL sequence of 4-wire QSPI transferring RGB888, RGB666 and
 * RGB555 is 50Mhz" (re-verified first-hand in toy5-pace; the
 * 20 ns TSCYC floor is format-independent and gives the same
 * 50 MHz ceiling for the RGB565 used here). D0 ran a
 * 25,000,000 target; toy5-pace raises the target to the cap.
 *
 * Divider derivation (integer divider only,
 * sm_config_set_clkdiv_int_frac with frac 0; the PIO program
 * spends 2 state-machine cycles per SCL period):
 * SCL = clk_sys / (2 * div), div rounded up. At the stock
 * 150 MHz sysclk: div = ceil(150e6 / (2 * 50e6)) = 2, so the
 * ACTUAL rate is 37.5 MHz - the nearest below the cap. div 1
 * would give 75 MHz (13.3 ns period, under the 20 ns TSCYC
 * minimum). A fractional 1.5 divider would average 50 MHz
 * but the jittered SM clock emits individual SCL cycles
 * shorter than 20 ns, so it is deliberately not used. At
 * 37.5 MHz the period is 26.7 ns and each half-period is
 * 13.3 ns, above the TSCH/TSCL 6.5 ns minima (same table).
 * display_qspi_scl_hz() reports the actual post-divider
 * rate. Shared by toy_product and toy_display_test: the
 * display test simply runs at the faster rate. */
#define DISPLAY_QSPI_SCL_HZ 50000000u

typedef struct display_qspi display_qspi_t;
struct display_qspi {
    void *pio; /* PIO instance (hardware/pio.h PIO), kept opaque here */
    int sm;
    uint32_t scl_hz; /* actual SCL rate after divider rounding */
};

/* Claim a PIO state machine, drive pins, start the transmitter.
 * CS ends high (deselected), RESX high. Returns 0, or -1 if no
 * state machine is free. Does not reset or initialise the panel;
 * that is co5300_init's job over the sink. */
int display_qspi_init(display_qspi_t *t);

/* Fill in a co5300_sink_t that drives this transport. */
void display_qspi_sink(display_qspi_t *t, co5300_sink_t *sink);

uint32_t display_qspi_scl_hz(const display_qspi_t *t);

#endif
