/* display_qspi_pio.c - RP2350 PIO transport for the CO5300 sink.
 *
 * One PIO state machine (display_qspi.pio) streams nibbles to
 * SI3..SI0 with SCL as a side-set pin. Command bytes are expanded
 * with co5300_expand_cmd_byte so they leave 1-wire on SIO0; pixel
 * bytes go out as-is (two nibbles each, high first). CPU feeds the
 * FIFO with blocking puts: at 25 MHz SCL a byte lasts 320 ns, and
 * if the CPU ever falls behind the machine stretches SCL high at
 * a byte boundary, which the panel tolerates. DMA is a later
 * presenter optimisation, deliberately not D0 scope.
 *
 * Integer divider math only: this target keeps the RP2350
 * no-hardware-float policy (see board/CMakeLists.txt), so the
 * clock divider uses sm_config_set_clkdiv_int_frac, never the
 * float form.
 */
#include "display_qspi_pio.h"

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
    /* Shift left (MSB of the FIFO word first), autopull every 8
     * bits: one FIFO word = one wire byte in bits 31..24. */
    sm_config_set_out_shift(&c, false, true, 8);
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

    pio_sm_init(pio0, sm, offset, &c);
    pio_sm_set_enabled(pio0, sm, true);
    return 0;
}

uint32_t display_qspi_scl_hz(const display_qspi_t *t)
{
    return t->scl_hz;
}

/* Wait until every queued byte has left the pins: FIFO empty,
 * then the state machine's TX-stall debug flag shows it tried to
 * pull from the empty FIFO (i.e. the last nibble is out). Clear
 * the flag so the next transfer's wait sees a fresh event. The
 * caller always queues at least one byte before waiting. */
static void wait_idle(display_qspi_t *t)
{
    PIO pio = tpio(t);
    uint sm = (uint)t->sm;
    uint32_t stall_bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!pio_sm_is_tx_fifo_empty(pio, sm))
        tight_loop_contents();
    while (!(pio->fdebug & stall_bit))
        tight_loop_contents();
    pio->fdebug = stall_bit;
}

static void sink_cs(void *ctx, int asserted)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    if (asserted) {
        gpio_put(DISPLAY_PIN_CS, 0);
    } else {
        wait_idle(t);
        gpio_put(DISPLAY_PIN_CS, 1);
    }
}

static void sink_cmd_bytes(void *ctx, const uint8_t *bytes, size_t len)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    PIO pio = tpio(t);
    uint sm = (uint)t->sm;
    for (size_t i = 0; i < len; i++) {
        uint8_t expanded[4];
        co5300_expand_cmd_byte(bytes[i], expanded);
        for (int k = 0; k < 4; k++)
            pio_sm_put_blocking(pio, sm, (uint32_t)expanded[k] << 24);
    }
}

static void sink_pixel_bytes(void *ctx, const uint8_t *bytes, size_t len)
{
    display_qspi_t *t = (display_qspi_t *)ctx;
    PIO pio = tpio(t);
    uint sm = (uint)t->sm;
    for (size_t i = 0; i < len; i++)
        pio_sm_put_blocking(pio, sm, (uint32_t)bytes[i] << 24);
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
