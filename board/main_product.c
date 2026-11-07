/* main_product.c - first on-glass product firmware: presenter +
 * attract rotation (toy5-presenter).
 *
 * Boots the CO5300 panel (board/display_co5300.* over the PIO
 * QSPI transport, glass-proven by toy_display_test), then cycles
 * every registered cartridge forever, TOY_PROD_CART_SECONDS of
 * wall clock each, in registry order. This is the LIVE runner
 * posture, not a proof:
 *   - feed carts run on their synthetic on-device source
 *     (toy_sig_proof_source_init + toy_feed_step), the same
 *     source main_board.c instantiates, boundary order
 *     feed -> apply -> step exactly as there;
 *   - the apply slot drains a live queue that stays empty in
 *     this parcel (no touch/IMU/RTC/shell yet; the sensors
 *     parcel attaches sources later). Proof scripts are NOT
 *     replayed: no scripted gestures run unattended. The one
 *     exception is step-0 setup, mirrored from emu frames mode
 *     (emu/main_emu.c applies toy_apply_proof_at(..., 0, ...)
 *     right after reset): step-0 entries are parameter setup
 *     (julia's iter pin, solclock's calendar pin), and applying
 *     them is what makes those carts look right unattended.
 *     Entries past step 0 are never applied here.
 *   - the framebuffer persists across frames within a cart
 *     (trail carts fade; see presenter.h) and is cleared, with
 *     a black fill to glass, on every cart switch.
 * Serial (USB CDC) is output-only: TOY-PROD-* markers with
 * integer prints only. No AXP writes, GPIO25 untouched.
 */
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/unique_id.h"

#include "display_co5300.h"
#include "display_qspi_pio.h"
#include "presenter.h"
#include "toy_cart.h"

#ifndef TOY_GIT_HEAD
#define TOY_GIT_HEAD "unknown"
#endif

/* Wall-clock seconds per cart in the attract rotation. */
#define TOY_PROD_CART_SECONDS 20u

/* Product brightness: constant raw DBV (51h), same modest level
 * toy_display_test used for its patterns. Panel stays on. */
#define TOY_PROD_DBV 0x50u

/* Linker-script symbols for the boot memory evidence line
 * (pico_standard_link script_includes define all of these). */
extern char __data_start__, __data_end__;
extern char __bss_start__, __bss_end__;
extern char __end__, __HeapLimit;         /* heap start / RAM end */
extern char __StackTop, __StackBottom;    /* SCRATCH_Y stack */

static _Alignas(8) unsigned char state_buf[TOY_MAX_STATE_SIZE];
static toy_presenter_t presenter;
static co5300_t panel;

/* Live input seam: empty until a driver parcel enqueues (same
 * posture and comment as main_board.c's board_live_queue). */
static toy_queue_t prod_live_queue;

/* Signal seam: synthetic on-device source + frame, as in
 * main_board.c. Source creation (toy_sig_proof_source_init) is
 * the one point a future live source swaps in. */
static toy_sig_source_t prod_sig_src;
static int16_t prod_sig_frame[TOY_SIG_MAX_FRAME_VALUES];

/* Run one cart for its attract slot. Returns when the slot's
 * wall clock expires or the signal feed fails (loud, with a
 * marker; the rotation moves on rather than wedge the toy). */
static void run_cart_attract(const toy_cart_t *cart)
{
    int sig_created = toy_sig_proof_source_init(cart, &prod_sig_src);
    if (sig_created < 0) {
        printf("TOY-PROD-SIG-ERR cart=%s stage=init\n", cart->name);
        return;
    }
    memset(state_buf, 0, cart->state_size);
    cart->reset(state_buf);
    toy_queue_clear(&prod_live_queue);
    /* Step-0 parameter setup only (see file header). The cursor
     * is then dropped: later proof entries never apply. */
    uint32_t cur = 0;
    toy_apply_proof_at(cart, state_buf, 0, &cur);

    toy_fb_t *fb = toy_presenter_fb(&presenter);
    toy_fb_clear(fb, 0x0000u);
    (void)toy_presenter_blank(&presenter); /* black between carts */

    printf("TOY-PROD-CART cart=%s title=%s steps_per_frame=%lu seconds=%lu\n",
           cart->name, cart->title,
           (unsigned long)cart->demo_steps_per_frame,
           (unsigned long)TOY_PROD_CART_SECONDS);

    uint64_t slot_start = time_us_64();
    uint64_t slot_end = slot_start +
                        (uint64_t)TOY_PROD_CART_SECONDS * 1000000ull;
    uint64_t pace_mark = slot_start + 1000000ull;
    uint64_t sim_us = 0, render_us = 0, flush_us = 0;
    unsigned long frames = 0, seconds = 0;

    while (time_us_64() < slot_end) {
        uint64_t t0 = time_us_64();
        for (uint32_t s = 0; s < cart->demo_steps_per_frame; s++) {
            int frc = toy_feed_step(cart, state_buf,
                                    sig_created ? &prod_sig_src : 0,
                                    prod_sig_frame,
                                    TOY_SIG_MAX_FRAME_VALUES);
            if (frc) {
                printf("TOY-PROD-SIG-ERR cart=%s stage=feed code=%d\n",
                       cart->name, frc);
                return;
            }
            /* Empty on current hardware; shared step cap, as in
             * main_board.c (proof cursor is absent live). */
            toy_apply_queue(cart, state_buf, &prod_live_queue,
                            TOY_MAX_EVENTS_PER_STEP);
            cart->step(state_buf);
        }
        uint64_t t1 = time_us_64();
        cart->render(state_buf, fb);
        uint64_t t2 = time_us_64();
        int rc = toy_presenter_flush(&presenter);
        uint64_t t3 = time_us_64();
        if (rc != CO5300_OK) {
            printf("TOY-PROD-FLUSH-ERR cart=%s rc=%d\n", cart->name, rc);
            return;
        }
        sim_us += t1 - t0;
        render_us += t2 - t1;
        flush_us += t3 - t2;
        frames++;

        if (t3 >= pace_mark) {
            seconds++;
            printf("TOY-PROD-PACE cart=%s sim_us=%llu render_us=%llu "
                   "flush_us=%llu frames=%lu seconds=%lu\n",
                   cart->name, (unsigned long long)sim_us,
                   (unsigned long long)render_us,
                   (unsigned long long)flush_us, frames, seconds);
            sim_us = render_us = flush_us = 0;
            frames = 0;
            pace_mark = t3 + 1000000ull;
        }
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
    printf("TOY-PROD-BOOT git %s serial %s clk_hz %lu\n", TOY_GIT_HEAD,
           idstr, (unsigned long)clock_get_hz(clk_sys));
    printf("TOY-PROD-PRESENTER mode=%s fb=%dx%d fb_bytes=%lu carts=%d "
           "cart_seconds=%lu dbv=0x%02x\n",
           TOY_PRESENTER_MODE_NAME, TOY_PRESENTER_FB_W, TOY_PRESENTER_FB_H,
           (unsigned long)TOY_PRESENTER_FB_BYTES, toy_cart_count,
           (unsigned long)TOY_PROD_CART_SECONDS, TOY_PROD_DBV);
    /* Free-SRAM evidence from the linked layout itself: main RAM
     * is [0x20000000, __HeapLimit); __end__ is the first byte past
     * .data/.bss/.heap-reserve, so heap_free is what option (a)
     * in presenter.h was judged on. The core-0 stack lives in
     * SCRATCH_Y, reserved by crt0's .stack spacer
     * (PICO_STACK_SIZE, set for this target in CMakeLists.txt). */
    printf("TOY-PROD-MEM data_bytes=%lu bss_bytes=%lu heap_free_bytes=%lu "
           "stack_reserved_bytes=%lu state_buf_bytes=%lu\n",
           (unsigned long)(&__data_end__ - &__data_start__),
           (unsigned long)(&__bss_end__ - &__bss_start__),
           (unsigned long)(&__HeapLimit - &__end__),
           (unsigned long)(&__StackTop - &__StackBottom),
           (unsigned long)sizeof state_buf);

    printf("TOY-PROD-PHASE panel-init\n");
    display_qspi_t transport;
    if (display_qspi_init(&transport) != 0) {
        printf("TOY-PROD-ERR transport-init\n");
        for (;;)
            sleep_ms(1000);
    }
    printf("TOY-PROD-QSPI scl_hz=%lu\n",
           (unsigned long)display_qspi_scl_hz(&transport));
    co5300_sink_t sink;
    display_qspi_sink(&transport, &sink);
    int rc = co5300_bind(&panel, &sink);
    if (rc == CO5300_OK)
        rc = co5300_init(&panel, (uint8_t)TOY_PROD_DBV);
    printf("TOY-PROD-INIT rc=%d dbv=0x%02x\n", rc, TOY_PROD_DBV);
    if (rc != CO5300_OK) {
        for (;;)
            sleep_ms(1000);
    }
    toy_presenter_init(&presenter, &panel);
    /* GRAM is undefined after init; first act is black. */
    (void)toy_presenter_blank(&presenter);

    printf("TOY-PROD-ATTRACT carts=%d seconds_each=%lu\n", toy_cart_count,
           (unsigned long)TOY_PROD_CART_SECONDS);
    for (;;) {
        for (int i = 0; i < toy_cart_count; i++) {
            const toy_cart_t *cart = toy_carts[i];
            if (cart->state_size > sizeof state_buf) {
                printf("TOY-PROD-SKIP cart=%s state_size=%lu exceeds %lu\n",
                       cart->name, (unsigned long)cart->state_size,
                       (unsigned long)sizeof state_buf);
                continue;
            }
            if (toy_cart_check_input(cart) || toy_cart_check_signal(cart)) {
                printf("TOY-PROD-BAD-CART cart=%s\n", cart->name);
                continue;
            }
            if (!cart->render || cart->demo_steps_per_frame == 0) {
                printf("TOY-PROD-SKIP cart=%s no-render-or-pacing\n",
                       cart->name);
                continue;
            }
            run_cart_attract(cart);
        }
    }
}
