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

#include <stddef.h>
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

/* Word packing for the PIO TX feed (toy5-flush). The state
 * machine shifts its OSR left, MSB first, with autopull at 32
 * bits, so one 32-bit FIFO word carries FOUR wire bytes -
 * bytes[0] in bits 31..24 down to bytes[3] in bits 7..0 - and
 * the nibble sequence on the pins (each byte high nibble
 * first) is exactly the sequence the pre-packing feed produced
 * with one byte per word in bits 31..24 and autopull at 8.
 * The PIO program itself is unchanged.
 *
 * The packer sits below the co5300_sink_t byte seam: cmd and
 * pixel byte calls of ANY length feed through it in order,
 * and a partial word (1..3 bytes) is carried across calls,
 * because chunk boundaries are invisible on the wire. What is
 * still pending when CS is released is emitted by the
 * transport as one final short word (its bytes in the top of
 * pack.word, zeros below, consumed bits = 8 * pack.len) - see
 * flush_pack_tail in display_qspi_pio.c. Pure and
 * host-compilable: tools/display_qspi_pack_test.c pins the
 * exact word sequences, including every tail length. */
typedef struct {
    uint32_t word; /* pending bytes, MSB-first, zeros below */
    uint8_t len;   /* pending byte count, 0..3 */
} display_qspi_pack_t;

static inline void display_qspi_pack_reset(display_qspi_pack_t *p)
{
    p->word = 0;
    p->len = 0;
}

/* Pack exactly four wire bytes into one FIFO word. */
static inline uint32_t display_qspi_pack_word4(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

/* Append one byte. Returns 1 (and writes *word_out, resetting
 * the packer) when that byte completes a 4-byte word. */
static inline int display_qspi_pack_push(display_qspi_pack_t *p,
                                         uint8_t byte,
                                         uint32_t *word_out)
{
    p->word |= (uint32_t)byte << (24u - 8u * p->len);
    p->len++;
    if (p->len < 4)
        return 0;
    *word_out = p->word;
    display_qspi_pack_reset(p);
    return 1;
}

/* Bits of pack.word that are real wire bytes, for the final
 * short-word flush: 0, 8, 16 or 24. */
static inline uint32_t
display_qspi_pack_tail_bits(const display_qspi_pack_t *p)
{
    return (uint32_t)p->len * 8u;
}

typedef void (*display_qspi_put_word_fn)(void *ctx, uint32_t word);

/* Feed len wire bytes through the packer, emitting each
 * completed word via put_word. With no partial word pending
 * the body packs four bytes at a time; a pending partial is
 * topped up byte-wise first, and a 1..3 byte remainder is left
 * pending for the next call (or the CS-release tail flush).
 * This is the exact algorithm the PIO sink runs; the host
 * test drives it with a capturing put_word. */
static inline void display_qspi_pack_feed(display_qspi_pack_t *p,
                                          const uint8_t *bytes,
                                          size_t len,
                                          display_qspi_put_word_fn put_word,
                                          void *ctx)
{
    size_t i = 0;
    while (i < len) {
        if (p->len == 0 && len - i >= 4) {
            put_word(ctx, display_qspi_pack_word4(bytes + i));
            i += 4;
        } else {
            uint32_t word;
            if (display_qspi_pack_push(p, bytes[i], &word))
                put_word(ctx, word);
            i++;
        }
    }
}

typedef struct display_qspi display_qspi_t;
struct display_qspi {
    void *pio; /* PIO instance (hardware/pio.h PIO), kept opaque here */
    int sm;
    uint32_t scl_hz; /* actual SCL rate after divider rounding */
    display_qspi_pack_t pack; /* wire bytes not yet a full FIFO word */
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
