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

/* Input seam: the proof loop replays each cart's compiled-in
 * proof_inputs script at its recorded steps (step 0 after reset,
 * then before each numbered step) through the shared helpers in
 * toy_cart.h, exactly as the host emu does; the TOY-RUN line gains
 * inputs=/input_hash= only for scripted carts. The live event
 * queue exists here but stays empty: this proof firmware has no
 * touch/IMU/input source on current hardware, so nothing ever
 * enqueues. Its (empty) drain runs each step to keep the seam
 * exercised; a future live firmware enqueues from drivers and
 * drains through the same call. */
static toy_queue_t board_live_queue;

/* Signal seam (SIG-1): the proof loop instantiates each feed
 * cart's synthetic descriptor spec ON DEVICE (toy_signal.c is
 * in the board build for exactly this) and feeds exactly
 * samples_per_step frames before apply()/step(), feed -> apply
 * -> step, byte-for-byte the host emu's order and TOY-RUN
 * format. This is the strongest leg: it proves the RP2040
 * build of the integer generator matches the host sample for
 * sample. Source creation (toy_sig_proof_source_init) is the
 * one point a future live firmware swaps for a driver-backed
 * source; no sensor is fitted or read by this proof. */
static toy_sig_source_t board_sig_src;
static int16_t board_sig_frame[TOY_SIG_MAX_FRAME_VALUES];

static void run_cart(const toy_cart_t *cart, long total, long every)
{
    int sig_created = toy_sig_proof_source_init(cart, &board_sig_src);
    if (sig_created < 0) {
        printf("TOY-SIG-ERR cart=%s stage=init\n", cart->name);
        return;
    }
    memset(state_buf, 0, cart->state_size);
    cart->reset(state_buf);
    toy_queue_clear(&board_live_queue);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, state_buf, 0, &cur); /* step-0 setup; no feed */
    printf("TOY-RUN cart=%s state_size=%u hash=fnv1a64",
           cart->name, (unsigned)cart->state_size);
    if (cart->proof_input_count)
        printf(" inputs=%u input_hash=%016llx",
               (unsigned)cart->proof_input_count,
               (unsigned long long)toy_hash_inputs(cart->proof_inputs,
                                                  cart->proof_input_count));
    if (sig_created)
        printf(" sig=%s device=%u scenario=%u seed=0x%llx rate=%u per_step=%u stream=%016llx",
               toy_sig_kind_name(board_sig_src.info.kind),
               (unsigned)board_sig_src.info.device,
               (unsigned)board_sig_src.info.scenario_id,
               (unsigned long long)board_sig_src.info.seed,
               (unsigned)board_sig_src.info.sample_rate_hz,
               (unsigned)cart->sig_spec.samples_per_step,
               (unsigned long long)board_sig_src.info.stream_id);
    printf("\n");
    for (long s = 1; s <= total; s++) {
        int frc = toy_feed_step(cart, state_buf,
                                sig_created ? &board_sig_src : 0,
                                board_sig_frame,
                                TOY_SIG_MAX_FRAME_VALUES);
        if (frc) { /* proof: fail loud, never zero-fill */
            printf("TOY-SIG-ERR cart=%s stage=feed step=%ld code=%d\n",
                   cart->name, s, frc);
            return;
        }
        uint32_t before = cur;
        toy_apply_proof_at(cart, state_buf, (uint32_t)s, &cur);
        /* Empty on current hardware; live cap shares the step cap. */
        toy_apply_queue(cart, state_buf, &board_live_queue,
                        TOY_MAX_EVENTS_PER_STEP - (cur - before));
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
        if (toy_cart_check_input(cart) || toy_cart_check_signal(cart)) {
            printf("TOY-BAD-CART cart=%s\n", cart->name);
            continue;
        }
        run_cart(cart, (long)cart->proof_steps,
                 (long)cart->proof_report_every);
    }
    printf("TOY-ALL-DONE\n");
    for (;;) { gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN); sleep_ms(500); }
}
