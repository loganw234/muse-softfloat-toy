/* display_qspi_pio.c - RP2350 PIO transport for the CO5300 sink.
 *
 * One PIO state machine (display_qspi.pio) streams nibbles to
 * SI3..SI0 with SCL as a side-set pin. Command bytes are expanded
 * with co5300_expand_cmd_byte so they leave 1-wire on SIO0; pixel
 * bytes go out as-is (two nibbles each, high first). CPU feeds the
 * FIFO with blocking puts of WORD-PACKED data (toy5-flush: four
 * wire bytes per 32-bit FIFO word, autopull at 32 bits; the pure
 * packer lives in display_qspi_pio.h and is host-tested by
 * tools/display_qspi_pack_test.c). The nibble stream on the pins
 * is bit-identical to the previous one-byte-per-word feed - the
 * PIO program did not change, only the pull threshold and the
 * feed loop. At the actual 37.5 MHz SCL (divider derivation in
 * display_qspi_pio.h) a 1-wire command byte lasts ~213 ns (eight
 * SCL cycles) and a pixel byte ~53 ns, and if the CPU ever falls
 * behind the machine stretches SCL high at a byte boundary,
 * which the panel tolerates. A full frame is 434,312 pixel bytes:
 * 108,578 FIFO writes packed, down from 434,312. No DMA: the
 * cycle estimate in the toy5-flush parcel report puts the packed
 * CPU feed inside ~1.3x of the 23.2 ms wire floor, so the extra
 * machinery buys nothing (and this transport keeps exactly one
 * writer - the foreground - touching the FIFO, which is the
 * posture wait_idle's drain argument depends on).
 *
 * Integer divider math only: this target keeps the RP2350
 * no-hardware-float policy (see board/CMakeLists.txt), so the
 * clock divider uses sm_config_set_clkdiv_int_frac, never the
 * float form.
 */
#include "display_qspi_pio.h"

#include "hardware/address_mapped.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/regs/pio.h"
#include "pico/stdlib.h"

#include "display_qspi.pio.h" /* pioasm output via pico_generate_pio_header */

static PIO tpio(const display_qspi_t *t) { return (PIO)t->pio; }

int display_qspi_init(display_qspi_t *t)
{
    t->pio = pio0;
    t->sm = pio_claim_unused_sm(pio0, false);
    if (t->sm < 0)
        return -1;
    uint sm = (uint)t->sm;

    /* CS and RESX are plain GPIOs, idle high (CSX is low-active,
     * datasheet 4.2). RESX starts high; co5300_init pulses it. */
    gpio_init(DISPLAY_PIN_CS);
    gpio_put(DISPLAY_PIN_CS, 1);
    gpio_set_dir(DISPLAY_PIN_CS, GPIO_OUT);
    gpio_init(DISPLAY_PIN_RST);
    gpio_put(DISPLAY_PIN_RST, 1);
    gpio_set_dir(DISPLAY_PIN_RST, GPIO_OUT);

    uint offset = pio_add_program(pio0, &display_qspi_tx_program);
    pio_sm_config c = display_qspi_tx_program_get_default_config(offset);

    /* SCL: side-set pin. Data: OUT field over SIO0..SI3. */
    pio_gpio_init(pio0, DISPLAY_PIN_SCL);
    for (uint i = 0; i < 4; i++)
        pio_gpio_init(pio0, DISPLAY_PIN_SIO0 + i);
    pio_sm_set_consecutive_pindirs(pio0, sm, DISPLAY_PIN_SCL, 1, true);
    pio_sm_set_consecutive_pindirs(pio0, sm, DISPLAY_PIN_SIO0, 4, true);
    sm_config_set_sideset_pins(&c, DISPLAY_PIN_SCL);
    sm_config_set_out_pins(&c, DISPLAY_PIN_SIO0, 4);
    /* Shift left (MSB of the FIFO word first), autopull every 32
     * bits: one FIFO word = four wire bytes, first in bits
     * 31..24 (toy5-flush word packing; threshold 32 is the
     * SDK's own encoding - the 5-bit PULL_THRESH field stores 0,
     * which the hardware reads as 32, same as the default
     * config's sm_config_set_out_shift(..., 32) call). */
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    /* SCL = clk_sys / (2 cycles * divider). Round the divider up
     * so the rate never exceeds the target. */
    uint32_t sys_hz = clock_get_hz(clk_sys);
    uint32_t div = (sys_hz + 2u * DISPLAY_QSPI_SCL_HZ - 1u) /
                   (2u * DISPLAY_QSPI_SCL_HZ);
    if (div == 0)
        div = 1;
    sm_config_set_clkdiv_int_frac(&c, (uint16_t)div, 0);
    t->scl_hz = sys_hz / (2u * div);

    display_qspi_pack_reset(&t->pack);
    pio_sm_init(pio0, sm, offset, &c);
    pio_sm_set_enabled(pio0, sm, true);
    return 0;
}

uint32_t display_qspi_scl_hz(const display_qspi_t *t)
{
    return t->scl_hz;
}

/* Wait until every queued byte has left the pins. FIFO empty
 * alone is not enough: the last word may still be shifting out
 * of the OSR (up to one packed word, four wire bytes), and
 * raising CS inside that window truncates the final bytes on
 * the wire.
 *
 * The TX-stall debug flag marks the state machine stalling on
 * the empty FIFO, which happens only after the final byte's
 * sampling edges are done - but it is a STICKY event flag, and
 * the machine also stalls mid-stream whenever the CPU falls
 * behind the FIFO for a moment (in the presenter flush that is
 * routine: the sink is refilled in fixed conversion chunks, so
 * an underrun stall happens inside almost every stream).
 * Testing the sticky flag without clearing it first therefore
 * passes instantly on a stale event and CS rises while the
 * last byte is still on the pins. Against a short command
 * stream that truncates its final parameter byte - for RASET,
 * the row-window end - and the panel can silently drop the
 * window, so pixel streams land nowhere (toy5-presenter
 * black-glass bug; the conversion-free fill path stays ahead
 * of the FIFO and never armed the stale flag, which is why
 * toy_display_test never saw it).
 *
 * Correct order: wait for FIFO empty (now at most one packed
 * word is in flight), CLEAR the sticky flag, then wait for a
 * fresh stall event. If the machine had already stalled
 * before the clear, the word is long gone but the flag might
 * not re-set until the next stall, so a short backstop
 * timeout - far longer than one packed word (four pixel bytes
 * or one 1-wire command byte, ~213 ns each) takes to shift at
 * any divider used here - ends the wait in that case. Word
 * packing (toy5-flush) changes nothing in this logic: FIFO
 * empty + a fresh stall still means OSR drained and the last
 * sampling edges done, at any pull threshold. */
static void wait_idle(display_qspi_t *t)
{
    PIO pio = tpio(t);
    uint sm = (uint)t->sm;
    uint32_t stall_bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!pio_sm_is_tx_fifo_empty(pio, sm))
        tight_loop_contents();
    pio->fdebug = stall_bit;
    uint64_t deadline = time_us_64() + 4;
    while (!(pio->fdebug & stall_bit) && time_us_64() < deadline)
        tight_loop_contents();
    pio->fdebug = stall_bit;
}

/* One packed word into the TX FIFO, blocking on space. The
 * pack_feed callback (display_qspi_pio.h). */
static void put_word(void *ctx, uint32_t word)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    pio_sm_put_blocking(tpio(t), (uint)t->sm, word);
}

/* Emit a partial word (1..3 wire bytes) still pending in the
 * packer at CS release, then leave the machine drained.
 *
 * With the standing 32-bit pull threshold the OSR would shift
 * all 32 bits - the zero pad bytes below the real ones would
 * clock out as extra nibbles the old byte feed never sent. So
 * for this one word the pull threshold is retargeted to
 * exactly the pending bit count: the OSR loads the word,
 * shifts only the real bytes, then stalls on the next pull
 * exactly as at any stream end. Both threshold writes happen
 * while wait_idle has left the machine stalled on an empty
 * FIFO with the OSR drained, so the new threshold can only
 * affect the next pull - never a word mid-shift. The field is
 * the PIO_SM0_* one: PULL_THRESH sits at the same bit
 * positions in every SM's SHIFTCTRL, and 0 restores the
 * 32-bit encoding set at init.
 *
 * Reachability: every command stream packs exactly (each
 * 1-wire logical byte expands to 4 wire bytes = 1 word), and
 * a full frame is 434,312 pixel bytes = 108,578 words, so the
 * presenter flush and the full-screen fill never enter here.
 * Odd-area windows do (any window whose 2*area byte count is
 * 2 mod 4, e.g. the display test's disc rows) - rare, short,
 * and now still bit-identical. */
static void flush_pack_tail(display_qspi_t *t)
{
    wait_idle(t);
    if (t->pack.len == 0)
        return;
    PIO pio = tpio(t);
    uint sm = (uint)t->sm;
    uint32_t word = t->pack.word;
    uint32_t bits = display_qspi_pack_tail_bits(&t->pack);
    hw_write_masked(&pio->sm[sm].shiftctrl,
                    bits << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB,
                    PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);
    pio_sm_put_blocking(pio, sm, word);
    wait_idle(t);
    hw_write_masked(&pio->sm[sm].shiftctrl, 0,
                    PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);
    display_qspi_pack_reset(&t->pack);
}

static void sink_cs(void *ctx, int asserted)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    if (asserted) {
        /* The packer is empty here by construction: the
         * previous release flushed its tail, and init reset
         * it before the first assertion. */
        gpio_put(DISPLAY_PIN_CS, 0);
    } else {
        flush_pack_tail(t);
        gpio_put(DISPLAY_PIN_CS, 1);
    }
}

static void sink_cmd_bytes(void *ctx, const uint8_t *bytes, size_t len)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    for (size_t i = 0; i < len; i++) {
        uint8_t expanded[4];
        co5300_expand_cmd_byte(bytes[i], expanded);
        /* 4 expanded wire bytes = exactly one packed word
         * (any cross-call partial still composes correctly
         * through the shared packer). */
        display_qspi_pack_feed(&t->pack, expanded, sizeof expanded,
                               put_word, t);
    }
}

static void sink_pixel_bytes(void *ctx, const uint8_t *bytes, size_t len)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    display_qspi_pack_feed(&t->pack, bytes, len, put_word, t);
}

static void sink_reset_pin(void *ctx, int level)
{
    (void)ctx;
    gpio_put(DISPLAY_PIN_RST, level ? 1 : 0);
}

static void sink_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    sleep_ms(ms);
}

void display_qspi_sink(display_qspi_t *t, co5300_sink_t *sink)
{
    sink->ctx = t;
    sink->cs = sink_cs;
    sink->cmd_bytes = sink_cmd_bytes;
    sink->pixel_bytes = sink_pixel_bytes;
    sink->reset_pin = sink_reset_pin;
    sink->delay_ms = sink_delay_ms;
}
