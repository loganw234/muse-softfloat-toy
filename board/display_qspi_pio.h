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

/* Target SCL rate. CO5300 datasheet 6.4.1 (p107) Note 3 caps
 * 4-wire QSPI pixel transfers at 50 MHz; D0 runs at half that. */
#define DISPLAY_QSPI_SCL_HZ 25000000u

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
