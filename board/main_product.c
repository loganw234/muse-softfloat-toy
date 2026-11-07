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
 *     a black fill to glass, on every cart switch;
 *   - pacing is FRAME-RATE FIRST (toy5-pace, owner decision):
 *     each presented frame steps the sim at least once, then
 *     keeps stepping until demo_steps_per_frame steps ran or
 *     the TOY_PROD_SIM_BUDGET_US budget is spent. On heavy
 *     carts the sim therefore runs slower than wall clock
 *     instead of the frame rate collapsing. State after N
 *     steps is untouched (feed/apply/step order and step-0
 *     setup unchanged); only how many steps happen per
 *     presented frame varies, which is runner-layer pacing,
 *     not cart behaviour. TOY-PROD-PACE reports steps= per
 *     window so the sim-speed ratio is
 *     steps / (frames * demo_steps_per_frame).
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

/* Per-frame sim time budget (toy5-pace, frame-rate first): a
 * frame always runs at least one step, then stops stepping
 * once this many microseconds of sim work have elapsed, even
 * if demo_steps_per_frame has not been reached. A single step
 * longer than the budget still completes and the frame
 * presents after it (steps=frames in PACE marks that regime).
 * Cheap carts finish all their demo steps well inside the
 * budget and pace exactly as before. */
#ifndef TOY_PROD_SIM_BUDGET_US
#define TOY_PROD_SIM_BUDGET_US 30000u
#endif

/* Product brightness: constant raw DBV (51h), same modest level
 * toy_display_test used for its patterns. Panel stays on. */
#define TOY_PROD_DBV 0x50u

/* Boot glass self-test (toy5-presenter black-glass diagnosis):
 * compile with -DTOY_PROD_SELFTEST=ON (CMake option, default
 * OFF; see CMakeLists.txt). Three ~1.5 s solid-colour stages
 * run after panel init, before attract, each bracketing one
 * candidate fault domain; see prod_selftest() below. */
#ifndef TOY_PROD_SELFTEST
#define TOY_PROD_SELFTEST 0
#endif

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

    printf("TOY-PROD-CART cart=%s title=%s steps_per_frame=%lu "
           "seconds=%lu sim_budget_us=%lu\n",
           cart->name, cart->title,
           (unsigned long)cart->demo_steps_per_frame,
           (unsigned long)TOY_PROD_CART_SECONDS,
           (unsigned long)TOY_PROD_SIM_BUDGET_US);

    uint64_t slot_start = time_us_64();
    uint64_t slot_end = slot_start +
                        (uint64_t)TOY_PROD_CART_SECONDS * 1000000ull;
    uint64_t pace_mark = slot_start + 1000000ull;
    uint64_t sim_us = 0, render_us = 0, flush_us = 0;
    unsigned long frames = 0, seconds = 0, steps_window = 0;

    while (time_us_64() < slot_end) {
        uint64_t t0 = time_us_64();
        /* Time-boxed stepper (toy5-pace): at least one step,
         * then step until demo_steps_per_frame ran or the sim
         * budget deadline passed. The time check runs once
         * per step, after the step, and only when further
         * steps remain - the step sequence itself (feed ->
         * apply -> step) is unchanged per step. */
        uint64_t sim_deadline = t0 + TOY_PROD_SIM_BUDGET_US;
        uint32_t steps_done = 0;
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
            steps_done++;
            if (steps_done < cart->demo_steps_per_frame &&
                time_us_64() >= sim_deadline)
                break;
        }
        steps_window += steps_done;
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
            /* steps= is the window's total sim steps: against
             * frames * demo_steps_per_frame it gives the
             * frame-rate-first sim-speed ratio (1.0 = budget
             * never bit; steps == frames = one step per frame,
             * the single-step-over-budget regime). */
            printf("TOY-PROD-PACE cart=%s sim_us=%llu render_us=%llu "
                   "flush_us=%llu frames=%lu steps=%lu seconds=%lu\n",
                   cart->name, (unsigned long long)sim_us,
                   (unsigned long long)render_us,
                   (unsigned long long)flush_us, frames, steps_window,
                   seconds);
            sim_us = render_us = flush_us = 0;
            frames = 0;
            steps_window = 0;
            pace_mark = t3 + 1000000ull;
        }
    }
}

#if TOY_PROD_SELFTEST
/* The pre-fix per-row flush, kept only for this self-test: one
 * 466x1 co5300_write_pixels window per glass row (466 CASET+
 * RASET+RAMWR streams per frame). Deliberately not in
 * presenter.c; production flush streams rows inside one CS. */
static int selftest_legacy_perrow_flush(void)
{
    toy_fb_t *fb = toy_presenter_fb(&presenter);
    for (int dy = 0; dy < CO5300_HEIGHT; dy++) {
        /* Precomputed maps (toy5-pace); pixel-identical to the
         * formula path this used before, host-pinned - the
         * per-row WINDOW framing stays the variable under test. */
        int sy = presenter.row_src[dy];
        toy_presenter_expand_row(&fb->px[(size_t)sy * (size_t)fb->w],
                                 presenter.col_src, presenter.row,
                                 CO5300_WIDTH);
        int rc = co5300_write_pixels(&panel, 0, dy, CO5300_WIDTH - 1, dy,
                                     presenter.row, CO5300_WIDTH);
        if (rc != CO5300_OK)
            return rc;
    }
    return CO5300_OK;
}

/* Three stages, ~1.5 s each, TOY-PROD-SELFTEST markers with rc.
 * What each stage discriminates (stated before any flash):
 *   fill-red: co5300_fill RED over the full glass. The exact
 *     path toy_display_test proved on this panel. RED visible
 *     => panel init + transport + fill are healthy inside the
 *     product binary. Black => the fault is in product-context
 *     bring-up, not the presenter; stop and diff against
 *     toy_display_test instead.
 *   presenter-green: fb cleared GREEN with toy_fb_clear, then
 *     the production toy_presenter_flush (single streamed
 *     window). No cart code involved. GREEN visible => fb
 *     wiring, row expansion and the streamed flush are proven
 *     in this binary; if attract then stays black, the fault
 *     moves to carts/runner. Black while red showed => the
 *     presenter/flush path is still broken on device despite
 *     host tests passing.
 *   legacy-perrow-blue: fb cleared BLUE, flushed with the
 *     pre-fix per-row algorithm above (but with the fixed PIO
 *     CS drain). BLUE visible => the CS-drain fix alone
 *     rehabilitates per-row windows, convicting the stale
 *     TXSTALL truncation as the root cause. Black/garbled
 *     while green showed => per-row CASET/RASET+RAMWR streams
 *     are themselves unsafe on this panel at this rate, and
 *     the single-stream presenter is the load-bearing fix.
 * Ends with the fb and glass black, ready for attract. */
static void prod_selftest(void)
{
    toy_fb_t *fb = toy_presenter_fb(&presenter);
    int rc;

    printf("TOY-PROD-SELFTEST stage=fill-red begin\n");
    rc = co5300_fill(&panel, 0, 0, CO5300_WIDTH - 1,
                     CO5300_HEIGHT - 1, 0xf800u);
    printf("TOY-PROD-SELFTEST stage=fill-red rc=%d hold_ms=1500\n", rc);
    sleep_ms(1500);

    toy_fb_clear(fb, 0x07e0u);
    printf("TOY-PROD-SELFTEST stage=presenter-green begin\n");
    rc = toy_presenter_flush(&presenter);
    printf("TOY-PROD-SELFTEST stage=presenter-green rc=%d hold_ms=1500\n",
           rc);
    sleep_ms(1500);

    toy_fb_clear(fb, 0x001fu);
    printf("TOY-PROD-SELFTEST stage=legacy-perrow-blue begin\n");
    rc = selftest_legacy_perrow_flush();
    printf("TOY-PROD-SELFTEST stage=legacy-perrow-blue rc=%d "
           "hold_ms=1500\n", rc);
    sleep_ms(1500);

    toy_fb_clear(fb, 0x0000u);
    (void)toy_presenter_blank(&presenter);
    printf("TOY-PROD-SELFTEST done\n");
}
#endif /* TOY_PROD_SELFTEST */

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
#if TOY_PROD_SELFTEST
    prod_selftest();
#endif

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
