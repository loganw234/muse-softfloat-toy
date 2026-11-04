/* main_board.c - core-only board proof (no display yet).
 *
 * Runs the cartridges headless and streams state hashes over USB
 * CDC in the same line format as emu's `run` mode. The gate is
 * host == board, exactly, at every reported step.
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "toy_cart.h"

#ifndef TOY_GIT_HEAD
#define TOY_GIT_HEAD "unknown"
#endif

static _Alignas(8) unsigned char state_buf[TOY_MAX_STATE_SIZE];

static void run_cart(const toy_cart_t *cart, long total, long every)
{
    memset(state_buf, 0, cart->state_size);
    cart->reset(state_buf);
    printf("TOY-RUN cart=%s state_size=%u hash=fnv1a64\n",
           cart->name, (unsigned)cart->state_size);
    for (long s = 1; s <= total; s++) {
        cart->step(state_buf);
        if (every > 0 && (s % every) == 0) {
            printf("step %ld hash %016llx\n", s,
                   (unsigned long long)toy_hash_state(state_buf, cart->state_size));
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
        }
    }
    printf("TOY-DONE steps %ld hash %016llx\n", total,
           (unsigned long long)toy_hash_state(state_buf, cart->state_size));
}

int main(void)
{
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    /* Wait for the host to attach before the banner (max ~15 s),
     * so the capture never races the first lines. */
    for (int i = 0; i < 150 && !stdio_usb_connected(); i++) {
        gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
        sleep_ms(100);
    }
    sleep_ms(200);
    char idstr[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    pico_get_unique_board_id_string(idstr, sizeof idstr);
    printf("TOY-BOARD git %s serial %s clk_hz %lu\n",
           TOY_GIT_HEAD, idstr, (unsigned long)clock_get_hz(clk_sys));
    for (int i = 0; i < toy_cart_count; i++) {
        const toy_cart_t *cart = toy_carts[i];
        if (cart->state_size > sizeof state_buf) {
            printf("TOY-SKIP cart=%s state_size=%u exceeds %u\n",
                   cart->name, (unsigned)cart->state_size,
                   (unsigned)sizeof state_buf);
            continue;
        }
        run_cart(cart, (long)cart->proof_steps,
                 (long)cart->proof_report_every);
    }
    printf("TOY-ALL-DONE\n");
    for (;;) { gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN); sleep_ms(500); }
}
