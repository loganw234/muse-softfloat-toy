/* main_product.c - product firmware: the shell runner on glass
 * (toy5-shell). Replaces the toy5-presenter attract rotation
 * outright (it is not retained behind an option: two main
 * loops would share nothing but boot, and the shell IS the
 * product posture now; the attract build stays reachable in
 * git history at the toy5-pace merge).
 *
 * What runs here, in the toy_shell.h runner contract order:
 *   boot: CO5300 panel over the PIO QSPI transport (presenter
 *   360-upscale, toy5-pace flush), then the shared I2C1 bus
 *   (GPIO6/7, 400 kHz): PCF85063 RTC bound as the runner
 *   clock (board/clock_rtc.c), CST9217 touch (INT GPIO22 /
 *   RST GPIO23), QMI8658 IMU + a 1 s still-desk gyro bias
 *   calibration attempt. Each sensor failure prints a loud
 *   TOY-PROD-*-ERR line and the toy continues without that
 *   sense organ. The shell opens in GALLERY over toy_carts.
 *   loop, every frame: service touch FIRST, before step/
 *   render/flush, so this pass's events drive this frame's
 *   shell/step decisions. Touch INT (GPIO22) is latched by a
 *   falling-edge IRQ (vendor flow is one report per edge;
 *   touchfix2 - a level sample once per ~55 ms frame missed
 *   pulses that rose during render+flush) and merged with
 *   the live level into the same cst9217_should_read drain
 *   policy; IMU joins in the same producer phase. Events
 *   become toy_event_t via the pure seams
 *   in board/product_input.h; feed EVERY event to
 *   toy_shell_event first and act on its mask (this file's
 *   prod_dispatch mirrors emu/main_emu.c shell_do_actions);
 *   RUNNING steps the cart with the toy5-pace time-boxed
 *   stepper (feed -> apply live queue -> step) and renders
 *   the cart; every other screen
 *   renders the shell with toy_clock_now() and never steps.
 *   The presenter flushes every frame in all screens, then
 *   toy_shell_tick() runs the idle clockface.
 *
 * Deliberate deviation from the emu tour's frame stepper:
 * proof-script entries past step 0 are NOT replayed on
 * device (the attract firmware's live posture, retained).
 * Step 0 still applies proof parameter setup then the
 * shell's param_queue, exactly as the tour's start path;
 * from step 1 on, the live queue is the only input source,
 * so on-glass behaviour is the owner's, not a recording's.
 *
 * Gestures (board/PRODUCT.md): tap/drag = pointer; in
 * GALLERY a single tap on a cart row launches it (runner
 * confirms the selecting tap on its UP edge, product_input
 * tap-launch gate; keyboard flow stays select+enter);
 * device tilt = TILT; turning the whole device = DIAL;
 * long-press (one finger held STILL >=1000 ms; a drag never
 * exits) = KEY_ESC exit/back; two-finger tap while RUNNING
 * = KEY_X settings. Every DOWN acted on prints one
 * TOY-PROD-TOUCHDBG line (raw/glass/milli/screen/action) so
 * the touch mirror direction is decidable from serial alone.
 * Serial (USB CDC) is output-only TOY-PROD-* markers,
 * integer prints. No AXP writes, GPIO25 untouched, no
 * core/cart changes.
 *
 * Shared-bus recovery (busfix): touch + IMU + RTC all ride
 * I2C1, and one transfer that dies mid-byte can leave a
 * slave holding SDA low, after which every address fails
 * forever while the frame loop runs on (live-observed:
 * TOUCH-ERR/IMU-ERR counts in the thousands, gallery still
 * idling to clockface). prod_bus_recover() below is the
 * self-heal: after TOY_PROD_BUS_ERR_THRESHOLD consecutive
 * transport errors (pure counter in product_input.h;
 * CST9217_NO_REPORT is a successful transfer and resets
 * it), the main loop resets the bus (deinit, up to 9 SCL
 * clocks with SDA released, STOP, re-init at 400 kHz) and
 * re-probes every device that was healthy at boot,
 * printing TOY-PROD-BUS-RECOVER lines (with the SDA/SCL
 * levels found and clocks needed, so the next wedge
 * convicts its own mechanism from serial). All I2C happens
 * in this main loop only: the GPIO22 handler touches the
 * volatile latch flag + edge counter and nothing else, so
 * transfers are single-flight by construction.
 *
 * Live feed (toy5-livefeed): the four IMU6 feed carts
 * (gpend/seismo/attitude/heartbeat) start on the real
 * QMI8658 when the IMU is up. This file is the live runner
 * core's contract always named (toy_feed_step is
 * proof-strict; a live runner "feeds gaps itself",
 * core/toy_cart.h): prod_imu_poll's one read per frame -
 * the same read tilt/dial already took, same back-off,
 * same bus-health notes, no extra I2C - also notes the raw
 * counts into board/live_feed.h, whose latest-sample hold
 * fills each step's frame and whose LIVE info stamps the
 * cart's hashed provenance through the existing
 * toy_sig_label_feed path. On-glass the label reads LIVE;
 * serial gets a TOY-PROD-FEED line at cart start (src=live,
 * or src=synthetic-fallback when the IMU is down and the
 * cart keeps its synthetic scenario) and a 1 Hz telemetry
 * line while a live cart runs (fresh vs repeated samples,
 * read errors, max held-sample age, achieved fresh rate vs
 * the cart's spec rate, time-warp). On a read error the
 * last good sample is held and counted; motion is never
 * fabricated, and before the first good sample the cart is
 * not stepped at all (its NO SIG rendering shows). The proof
 * firmware (main_board.c) does not include live_feed.h and
 * stays synthetic-only.
 *
 * Flash checkpoints (toy5-flashprod): while a cart is
 * RUNNING, its state is checkpointed to the internal-flash
 * A/B slots (board/ckpt_flash.c over core/toy_ckpt_slot.h,
 * TOYCKPT1 images) every TOY_PROD_CKPT_SECONDS of wall time
 * and once on clean exit-to-gallery, always between frames
 * (after the presenter flush, before the next step; the
 * exit save runs at frame top before that frame steps).
 * Each save serialises into a static SRAM image buffer and
 * goes through toy_ckpt_flash_write_next, so the previous
 * checkpoint is never the slot being erased and a power
 * cut mid-save leaves it newest-valid. Policy - cadence,
 * skip-live, boot-resume decision - is the pure seam in
 * board/product_ckpt.h (tools/prod_ckpt_test.c): LIVE feed
 * runs skip saving (a live sample position is not a
 * function of a step count and core refuses to rebuild
 * it; one TOY-PROD-CKPT skip-live line per run), synthetic
 * feed runs save and resume by seeking the respawned
 * source to step_count * samples_per_step. At boot, after
 * shell init, the newest valid slot for a still-registered
 * cart auto-starts that cart from the checkpoint bytes
 * (TOY-PROD-RESUME); any validation failure or unregistered
 * cart name lands in the gallery with a loud line. A fresh
 * (non-resume) start erases both slots first: step_count
 * restarts at 0 on a relaunch and is meaningless across
 * runs/carts, so slot selection must only ever compare
 * images of the one current run (the toy_ckpt_slot.h
 * caveat, decided here). Single-core only, exactly as
 * ckpt_flash.h states: PICO_FLASH_ASSUME_CORE1_SAFE, and a
 * future core-1 product must revisit. The proof firmware
 * gains none of this (no product_ckpt include, no calls).
 *
 * Gravity-quadrant rotation (toy5-rotate): the same
 * once-per-frame IMU read also feeds the pure orientation
 * tracker (board/orientation.h: hysteresis margin + 500 ms
 * dwell, near-flat keeps the last quadrant, boot quadrant
 * 0). On a quadrant change the runner sets the presenter's
 * sampling rotation, resets the tilt gate (carts learn the
 * new canonical down on the next sample) and prints
 * TOY-PROD-ORIENT quadrant=<n> - change only, nothing else
 * on serial. Touch and TILT production use the _rot seams
 * with the current quadrant identically on shell screens
 * and RUNNING; the live-feed hold is noted rotated too, so
 * feed carts' gravity agrees with the picture; DIAL
 * (screen-normal) is deliberately unchanged. Rotation
 * lives only in presentation + input production: cart
 * state, shell state, checkpoints and proof hashes never
 * see it, and the presenter is not linked by toy_board.
 */
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/unique_id.h"

#include "ckpt_flash.h"
#include "clock_civil.h"
#include "clock_rtc.h"
#include "display_co5300.h"
#include "display_qspi_pio.h"
#include "imu_qmi8658.h"
#include "live_feed.h"
#include "orientation.h"
#include "presenter.h"
#include "product_ckpt.h"
#include "product_input.h"
#include "rtc_pcf85063.h"
#include "touch_cst9217.h"
#include "toy_cart.h"
#include "toy_ckpt.h"
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"
#define TOY_SHELL_ALLOW 1
#include "toy_shell.h"

#ifndef TOY_GIT_HEAD
#define TOY_GIT_HEAD "unknown"
#endif

/* Per-frame sim time budget (toy5-pace, frame-rate first):
 * unchanged. A frame always runs at least one step, then
 * stops stepping once this many microseconds of sim work
 * elapsed, even if demo_steps_per_frame was not reached. */
#ifndef TOY_PROD_SIM_BUDGET_US
#define TOY_PROD_SIM_BUDGET_US 30000u
#endif

/* Product brightness: constant raw DBV (51h), the modest
 * level toy_display_test and the attract build used. */
#define TOY_PROD_DBV 0x50u

/* Boot glass self-test (diagnostic, CMake option default
 * OFF; retained from the presenter parcel verbatim). */
#ifndef TOY_PROD_SELFTEST
#define TOY_PROD_SELFTEST 0
#endif

/* Shared I2C1 + touch side pins (board/SENSORS.md). */
#define PIN_I2C_SDA 6u
#define PIN_I2C_SCL 7u
#define PIN_TOUCH_INT 22u
#define PIN_TOUCH_RST 23u

/* Touch read cadence cap (~50 Hz, as main_sensor_test) and
 * the ungated-probe period for a silent INT line. */
#define TOY_PROD_TOUCH_READ_MS 20u
#define TOY_PROD_TOUCH_PROBE_MS 1000u
/* IMU error back-off: a dead part costs one frame stall per
 * retry, not one per frame. */
#define TOY_PROD_IMU_RETRY_MS 500u
/* Dial integration step cap: a long frame (heavy cart,
 * settings pause) must not integrate one stale sample over
 * its whole length. */
#define TOY_PROD_DIAL_MAX_DT_MS 250u
/* Boot calibration offer: 50 samples, 20 ms apart (~1 s). */
#define TOY_PROD_CAL_SAMPLES 50u
#define TOY_PROD_CAL_PERIOD_MS 20u

extern char __data_start__, __data_end__;
extern char __bss_start__, __bss_end__;
extern char __end__, __HeapLimit;
extern char __StackTop, __StackBottom;

static _Alignas(8) unsigned char state_buf[TOY_MAX_STATE_SIZE];
/* Flash checkpoint image buffer (toy5-flashprod): one
 * TOYCKPT1 image at format cap (64 + 65,536 = 65,600 B;
 * the flash slot holding it is 69,632 B of erased sectors,
 * ckpt_flash.h). Static in SRAM because the write path
 * programs from it while XIP is down - never heap (could
 * be carved per-save and fragment the ~170 KB heap) and
 * never an XIP pointer (ckpt_flash rejects those). */
static _Alignas(8) unsigned char ckpt_buf[TOY_CKPT_MAX_IMAGE_SIZE];
static toy_presenter_t presenter;
static co5300_t panel;
static toy_shell_t shell;

/* Running cart (shell's RUNNING screen), or NULL. */
static const toy_cart_t *run_cart;
/* Steps since reset (or since the resumed checkpoint's
 * reset) for the running cart: the checkpoint header's
 * step_count and, for synthetic feed carts, the seek base
 * that respawns the sample source (product_ckpt.h). */
static uint64_t run_step_count;
static toy_prod_ckpt_timer_t ckpt_timer;
static int ckpt_skip_live_noted;
/* Boot-resume stash: set by prod_ckpt_try_resume() after
 * the image is loaded into state_buf and any synthetic
 * feed source is built + seeked; consumed by the
 * START_CART its synthetic ENTER produces. */
static int ckpt_resume_pending;
static const toy_cart_t *ckpt_resume_cart;
static uint32_t ckpt_resume_slot;
static uint64_t ckpt_resume_step;
static toy_queue_t prod_live_queue;
static toy_sig_source_t prod_sig_src;
static int prod_sig_created;
static int16_t prod_sig_frame[TOY_SIG_MAX_FRAME_VALUES];
/* Live feed (toy5-livefeed): the hold is noted by every
 * successful prod_imu_poll read, running cart or not, so a
 * cart launched later starts from a real fresh reading.
 * prod_feed_mode is the per-start source selection
 * (live_feed.h); prod_live_info is built once per live
 * start. feed_last_us anchors the 1 Hz telemetry rate. */
static toy_live_feed_t prod_live_feed;
static toy_sig_info_t prod_live_info;
static int prod_feed_mode = TOY_LIVE_FEED_NONE;
static uint64_t feed_last_us;

/* PACE window (RUNNING only), reset at each cart start. */
static uint64_t pace_mark_us, pace_sim_us, pace_render_us, pace_flush_us;
static unsigned long pace_frames, pace_steps, pace_seconds;

/* Sensors: ok flags degrade per-device, never wedge. */
static cst9217_config_t touch_cfg;
static bool touch_ok, imu_ok;
static toy_prod_gesture_t gesture;
static toy_orient_t orient; /* gravity quadrant (toy5-rotate) */
static toy_prod_tilt_gate_t tilt_gate;
static toy_prod_dial_t dial;
static toy_prod_tap_launch_t tap_launch;
/* IRQ latch (touchfix2): the GPIO22 falling-edge handler
 * only sets this flag and counts the edge - no I2C, no
 * gesture work in IRQ context. The frame loop merges it
 * with the live INT level (toy_prod_touch_pending_merge)
 * so a pulse that fell and rose during render+flush still
 * triggers exactly one drain read at the next pass. */
static volatile bool touch_irq_pending;
static volatile unsigned long touch_irq_edges;
/* Last real DOWN's controller coords per contact, so the
 * synthesised tap-launch DOWN can log the same TOUCHDBG
 * fields as the real one that armed it. */
static int32_t dbg_raw_x[TOY_PROD_MAX_CONTACTS];
static int32_t dbg_raw_y[TOY_PROD_MAX_CONTACTS];

static void prod_touch_irq(uint gpio, uint32_t events)
{
    if (gpio == PIN_TOUCH_INT && (events & GPIO_IRQ_EDGE_FALL)) {
        touch_irq_pending = true;
        touch_irq_edges++;
    }
}
static uint64_t next_touch_read_ms, next_touch_probe_ms, next_imu_ms;
static uint64_t last_imu_ms;
static bool have_imu_ms;
static unsigned touch_err_count, imu_err_count;
static uint8_t prev_screen = 0xFFu;
/* Shared-bus health (busfix): consecutive transport errors
 * across touch + IMU drive prod_bus_recover(). The expect_*
 * flags record which devices were healthy at boot and are
 * therefore owed a re-probe; a device absent at boot is
 * never reprobed (and can never keep the recovery retry
 * loop alive). */
static toy_prod_bus_health_t bus_health;
static bool rtc_ok, expect_rtc, expect_touch, expect_imu;

static toy_i2c_t *bus(void) { return (toy_i2c_t *)i2c1; }

static const char *screen_name(uint8_t s)
{
    switch (s) {
    case TOY_SHELL_GALLERY: return "gallery";
    case TOY_SHELL_RUNNING: return "running";
    case TOY_SHELL_SETTINGS: return "settings";
    case TOY_SHELL_CLOCKSET: return "clockset";
    case TOY_SHELL_CLOCKFACE: return "clockface";
    default: return "?";
    }
}

static void note_screen(void)
{
    uint8_t s = toy_shell_screen(&shell);
    if (s != prev_screen) {
        printf("TOY-PROD-EVENT screen=%s cart=%s\n", screen_name(s),
               toy_shell_current(&shell) ? toy_shell_current(&shell)->name
                                          : "-");
        prev_screen = s;
    }
}

/* --- cart lifecycle (mirrors emu shell_do_actions) -------------- */

/* One flash checkpoint of the running cart (toy5-flashprod).
 * Callers place this strictly between frames: the cadence
 * call sits after the presenter flush, the exit call at
 * frame top before anything steps. Serialise in SRAM, then
 * toy_ckpt_flash_write_next erases+programs the slot that
 * is NOT the newest valid one and read-back verifies, so a
 * power cut at any point leaves the previous checkpoint
 * selectable (product_ckpt.h / ckpt_flash.h).
 *
 * Interrupt audit for the XIP stall: no extra masking is
 * needed here, and none is done. The erase/program callback
 * runs inside flash_safe_execute, whose SDK default helper
 * (pico_flash flash.c, default_enter_safe_zone_timeout_ms)
 * save_and_disable_interrupts() on THIS core for the
 * callback's whole duration and restores after; with
 * PICO_FLASH_ASSUME_CORE1_SAFE=1 on core 0 that IRQ-only
 * path is the whole protocol (core 1 never runs). The
 * touch GPIO22 handler (prod_touch_irq) is an ordinary
 * flash-resident function - it must never execute while
 * XIP is down, and under the global mask it cannot: a
 * falling edge during the stall only latches the NVIC
 * pending bit (and the GPIO bank's edge state), delivered
 * as one IRQ after restore, which this runner's
 * latch-merged drain already treats as one report. Same
 * for the USB IRQ. Nothing else in the product takes
 * interrupts or runs DMA between frames (display QSPI PIO
 * is polled inside flush; I2C is polled), so the callback
 * window is the only flash-down window and it is masked.
 * A failed save is loud, non-fatal, and re-spaces the next
 * attempt by a full interval (timer note in every path). */
static void prod_ckpt_save(const char *why)
{
    if (!run_cart)
        return;
    if (toy_prod_ckpt_save_decision(prod_feed_mode) ==
        TOY_PROD_CKPT_SKIP_LIVE) {
        if (!ckpt_skip_live_noted) {
            printf("TOY-PROD-CKPT skip-live cart=%s why=%s "
                   "(LIVE feed: sample position not "
                   "reconstructible from a step count)\n",
                   run_cart->name, why);
            ckpt_skip_live_noted = 1;
        }
        toy_prod_ckpt_note(&ckpt_timer, toy_clock_mono_ms());
        return;
    }
    size_t len = 0;
    toy_ckpt_err_t serr = toy_ckpt_serialize(run_cart, state_buf,
                                             run_step_count, ckpt_buf,
                                             sizeof ckpt_buf, &len);
    if (serr != TOY_CKPT_OK) {
        printf("TOY-PROD-CKPT-ERR cart=%s step=%llu "
               "stage=serialize err=%d (%s) why=%s\n",
               run_cart->name, (unsigned long long)run_step_count,
               (int)serr, toy_ckpt_strerror(serr), why);
        toy_prod_ckpt_note(&ckpt_timer, toy_clock_mono_ms());
        return;
    }
    uint32_t slot = 0;
    uint64_t t0 = time_us_64();
    toy_ckpt_flash_err_t ferr =
        toy_ckpt_flash_write_next(ckpt_buf, len, &slot);
    uint64_t ms = (time_us_64() - t0) / 1000ull;
    toy_prod_ckpt_note(&ckpt_timer, toy_clock_mono_ms());
    if (ferr != TOY_CKPT_FLASH_OK) {
        printf("TOY-PROD-CKPT-ERR cart=%s step=%llu stage=write "
               "err=%d safe_rc=%d ms=%llu why=%s\n",
               run_cart->name, (unsigned long long)run_step_count,
               (int)ferr, toy_ckpt_flash_last_safe_rc(),
               (unsigned long long)ms, why);
        return;
    }
    printf("TOY-PROD-CKPT saved cart=%s step=%llu slot=%lu "
           "bytes=%lu ms=%llu why=%s\n",
           run_cart->name, (unsigned long long)run_step_count,
           (unsigned long)slot, (unsigned long)len,
           (unsigned long long)ms, why);
}

/* Fresh start: retire the previous run's checkpoints.
 * step_count restarts at 0 here, and slot selection orders
 * by step_count, so leaving the old run's images in place
 * would let a stale high-count image outrank this run's
 * saves - and its slot would pin the rotation (the
 * toy_ckpt_slot.h caveat). Erasing both slots once per
 * human launch keeps every scan single-run; the cost is
 * one erase of 34 sectors at a moment the owner just
 * tapped (wear accounting in board/PRODUCT.md). Resume
 * starts never come through here. */
static void prod_ckpt_clear_fresh(const char *cart_name)
{
    uint64_t t0 = time_us_64();
    toy_ckpt_flash_err_t err = toy_ckpt_flash_erase_all();
    uint64_t ms = (time_us_64() - t0) / 1000ull;
    if (err != TOY_CKPT_FLASH_OK)
        printf("TOY-PROD-CKPT-ERR cart=%s stage=clear err=%d "
               "safe_rc=%d ms=%llu why=fresh-start (continuing; "
               "a stale resume image may survive)\n",
               cart_name, (int)err, toy_ckpt_flash_last_safe_rc(),
               (unsigned long long)ms);
    else
        printf("TOY-PROD-CKPT cleared cart=%s ms=%llu "
               "why=fresh-start\n",
               cart_name, (unsigned long long)ms);
}

static void prod_stop_cart(const char *why)
{
    if (run_cart)
        printf("TOY-PROD-EVENT exit cart=%s why=%s\n", run_cart->name, why);
    run_cart = NULL;
    run_step_count = 0;
    toy_prod_ckpt_disarm(&ckpt_timer);
    prod_sig_created = 0;
    prod_feed_mode = TOY_LIVE_FEED_NONE;
    toy_queue_clear(&prod_live_queue);
}

/* Resume start (toy5-flashprod): prod_ckpt_try_resume()
 * already loaded the checkpoint bytes into state_buf,
 * built + seeked any synthetic feed source, and set
 * prod_feed_mode / prod_sig_created. State bytes are the
 * state - no reset(), no proof step-0, no shell param
 * drain: parameters ride inside the state where carts
 * keep them, exactly as the emu loader treats them. */
static void prod_start_resumed(const toy_cart_t *cart)
{
    ckpt_resume_pending = 0;
    run_cart = cart;
    run_step_count = ckpt_resume_step;
    toy_queue_clear(&prod_live_queue);
    toy_fb_clear(toy_presenter_fb(&presenter), 0x0000u);
    pace_mark_us = time_us_64() + 1000000ull;
    pace_sim_us = pace_render_us = pace_flush_us = 0;
    pace_frames = pace_steps = pace_seconds = 0;
    ckpt_skip_live_noted = 0;
    toy_prod_ckpt_arm(&ckpt_timer, toy_clock_mono_ms());
    printf("TOY-PROD-RESUME cart=%s step=%llu slot=%lu\n", cart->name,
           (unsigned long long)ckpt_resume_step,
           (unsigned long)ckpt_resume_slot);
    if (cart->feed)
        printf("TOY-PROD-FEED cart=%s src=synthetic-resume step=%llu "
               "seek_samples=%llu spec_hz=%lu sps=%lu\n",
               cart->name, (unsigned long long)ckpt_resume_step,
               (unsigned long long)ckpt_resume_step *
                   cart->sig_spec.samples_per_step,
               (unsigned long)cart->sig_spec.sample_rate_hz,
               (unsigned long)cart->sig_spec.samples_per_step);
    printf("TOY-PROD-EVENT start cart=%s resumed=1 hash=%016llx\n",
           cart->name,
           (unsigned long long)toy_hash_state(state_buf,
                                              cart->state_size));
    printf("TOY-PROD-CART cart=%s title=%s steps_per_frame=%lu "
           "sim_budget_us=%lu\n",
           cart->name, cart->title,
           (unsigned long)cart->demo_steps_per_frame,
           (unsigned long)TOY_PROD_SIM_BUDGET_US);
}

static void prod_start_cart(void)
{
    const toy_cart_t *cart = toy_shell_current(&shell);
    if (!cart) {
        printf("TOY-PROD-EVENT start-failed no-current-cart\n");
        toy_shell_enter(&shell, TOY_SHELL_GALLERY);
        return;
    }
    if (ckpt_resume_pending && cart == ckpt_resume_cart) {
        prod_start_resumed(cart);
        return;
    }
    ckpt_resume_pending = 0; /* any other start drops the stash */
    if (cart->state_size > sizeof state_buf || !cart->render ||
        cart->demo_steps_per_frame == 0 || toy_cart_check_input(cart) ||
        toy_cart_check_signal(cart)) {
        printf("TOY-PROD-BAD-CART cart=%s (staying in gallery)\n",
               cart->name);
        toy_shell_enter(&shell, TOY_SHELL_GALLERY);
        return;
    }
    /* Source selection (toy5-livefeed, live_feed.h): an
     * IMU6 feed cart goes LIVE on the real QMI8658 when the
     * IMU is up; otherwise it keeps its synthetic proof
     * scenario, said loudly here and on glass (the cart's
     * label renders SYNTHETIC, never a silent pretend-live).
     * Selection is start-time only - see live_feed.h. */
    prod_feed_mode = toy_live_feed_select(cart->feed != 0,
                                          cart->sig_spec.device,
                                          imu_ok ? 1 : 0);
    int sig_created = 0;
    if (prod_feed_mode == TOY_LIVE_FEED_LIVE) {
        toy_live_feed_info(&cart->sig_spec, &prod_live_info);
        toy_live_feed_reset_run(&prod_live_feed);
        feed_last_us = time_us_64();
        printf("TOY-PROD-FEED cart=%s src=live odr_hz=%u "
               "spec_hz=%lu sps=%lu stream=%016llx\n",
               cart->name, QMI8658_ODR_HZ,
               (unsigned long)cart->sig_spec.sample_rate_hz,
               (unsigned long)cart->sig_spec.samples_per_step,
               (unsigned long long)prod_live_info.stream_id);
    } else {
        sig_created = toy_sig_proof_source_init(cart, &prod_sig_src);
        if (sig_created < 0) {
            printf("TOY-PROD-SIG-ERR cart=%s stage=init\n", cart->name);
            toy_shell_enter(&shell, TOY_SHELL_GALLERY);
            return;
        }
        if (prod_feed_mode == TOY_LIVE_FEED_SYNTHETIC)
            printf("TOY-PROD-FEED cart=%s src=synthetic-fallback "
                   "reason=%s spec_hz=%lu sps=%lu\n",
                   cart->name,
                   cart->sig_spec.device == TOY_SIG_DEVICE_IMU6
                       ? "imu-down"
                       : "no-live-device",
                   (unsigned long)cart->sig_spec.sample_rate_hz,
                   (unsigned long)cart->sig_spec.samples_per_step);
    }
    prod_sig_created = sig_created;
    /* Fresh run (toy5-flashprod): retire the previous run's
     * checkpoints before this one's step 0, and arm the
     * cadence from the start moment. */
    prod_ckpt_clear_fresh(cart->name);
    run_step_count = 0;
    ckpt_skip_live_noted = 0;
    toy_prod_ckpt_arm(&ckpt_timer, toy_clock_mono_ms());
    memset(state_buf, 0, cart->state_size);
    run_cart = cart;
    toy_queue_clear(&prod_live_queue);
    cart->reset(state_buf);
    /* Step-0 slot, tour order: proof parameter setup first,
     * then the shell's user params drain over it. The proof
     * cursor stops at step 0 for good (file header). */
    uint32_t cur = 0;
    toy_apply_proof_at(cart, state_buf, 0, &cur);
    uint32_t applied = toy_apply_queue(cart, state_buf,
                                       &shell.param_queue,
                                       TOY_MAX_EVENTS_PER_STEP);
    toy_fb_clear(toy_presenter_fb(&presenter), 0x0000u);
    pace_mark_us = time_us_64() + 1000000ull;
    pace_sim_us = pace_render_us = pace_flush_us = 0;
    pace_frames = pace_steps = pace_seconds = 0;
    printf("TOY-PROD-EVENT start cart=%s params=%lu hash=%016llx\n",
           cart->name, (unsigned long)applied,
           (unsigned long long)toy_hash_state(state_buf,
                                              cart->state_size));
    printf("TOY-PROD-CART cart=%s title=%s steps_per_frame=%lu "
           "sim_budget_us=%lu\n",
           cart->name, cart->title,
           (unsigned long)cart->demo_steps_per_frame,
           (unsigned long)TOY_PROD_SIM_BUDGET_US);
}

/* CLOCK_COMMIT: shell gives hh/mm/tz; the date is ours (the
 * RTC's own calendar date - from now() when valid, else the
 * raw fields when only the OS flag withholds trust, else a
 * loudly-named 2000-01-01 fallback so the commit still sets
 * a real, owner-visible time instead of silently doing
 * nothing on a never-set chip). */
static void prod_clock_commit(void)
{
    toy_time_t now = toy_clock_now();
    int64_t base_local = 0;
    const char *src = "fallback-2000-01-01";
    if (now.valid) {
        base_local = toy_clock_local_seconds(now);
        src = "rtc";
    } else if (toy_clock_rtc_local_epoch_raw(&base_local)) {
        src = "rtc-fields-os-set";
    } else {
        base_local = toy_days_from_civil(2000, 1, 1) * 86400;
    }
    int64_t utc = toy_clock_commit_utc(base_local, shell.clock_hh,
                                       shell.clock_mm,
                                       shell.clock_tz_offset_min);
    bool ok = toy_clock_set(utc, shell.clock_tz_offset_min);
    printf("TOY-PROD-EVENT clock-commit local=%02ld:%02ld tz=%ld "
           "utc=%lld date_src=%s ok=%d\n",
           (long)shell.clock_hh, (long)shell.clock_mm,
           (long)shell.clock_tz_offset_min, (long long)utc, src,
           ok ? 1 : 0);
}

/* The one event funnel: shell first, act on the mask, in the
 * emu tour's action order. PARAM_QUEUED outside a start
 * moves into the live queue (FIFO with later live events,
 * shared per-step cap at the next boundary - the tour's
 * choice, and what pauses-in-settings requires). */
static uint32_t prod_dispatch(const toy_event_t *ev)
{
    uint32_t act = toy_shell_event(&shell, ev);
    if (act & TOY_SH_ACT_START_CART)
        prod_start_cart();
    if ((act & TOY_SH_ACT_PARAM_QUEUED) &&
        !(act & TOY_SH_ACT_START_CART)) {
        unsigned n = 0;
        while (shell.param_queue.len) {
            toy_queue_push(&prod_live_queue, &shell.param_queue.ev[0]);
            for (uint32_t i = 1; i < shell.param_queue.len; i++)
                shell.param_queue.ev[i - 1] = shell.param_queue.ev[i];
            shell.param_queue.len--;
            n++;
        }
        printf("TOY-PROD-EVENT params-queued n=%u\n", n);
    }
    if (act & TOY_SH_ACT_EXIT_TO_GALLERY) {
        /* Clean exit (toy5-flashprod): one final checkpoint
         * while run_cart + state are still live. Runs at
         * frame top, after last frame's flush and before
         * this frame steps - between frames, like cadence. */
        prod_ckpt_save("exit");
        prod_stop_cart("shell-exit");
    }
    if (act & TOY_SH_ACT_CLOCK_COMMIT)
        prod_clock_commit();
    if (act & TOY_SH_ACT_FORWARD_TO_CART)
        toy_queue_push(&prod_live_queue, ev);
    note_screen();
    return act;
}

/* Boot resume (toy5-flashprod), once after shell init:
 * scan both flash slots, decide over the live registry
 * (product_ckpt.h), and - only after every failure-prone
 * step has succeeded - launch the resumed cart by driving
 * the shell through its own event API (DOWN to the cart's
 * gallery row, then ENTER), so focus/param defaults/
 * running_valid come from the shell's start path, never
 * from runner field-pokes. The START_CART that ENTER
 * produces finds the resume stash in prod_start_cart and
 * adopts the pre-loaded state instead of resetting.
 * Failure anywhere leaves the shell in GALLERY untouched:
 * read_slot re-validates, toy_ckpt_load re-validates again
 * and leaves state_buf untouched on error, and the feed
 * source is built + seeked before the load. */
static void prod_ckpt_try_resume(void)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout) {
        printf("TOY-PROD-RESUME-ERR layout (gallery)\n");
        return;
    }
    printf("TOY-PROD-CKPT layout slots=%lu slot_bytes=%lu "
           "base=0x%lx flash_bytes=%lu cadence_s=%lu\n",
           (unsigned long)layout->slot_count,
           (unsigned long)layout->slot_bytes,
           (unsigned long)layout->base_offset,
           (unsigned long)layout->flash_bytes,
           (unsigned long)TOY_PROD_CKPT_SECONDS);
    toy_prod_ckpt_slot_view_t views[TOY_CKPT_SLOT_MAX_COUNT];
    memset(views, 0, sizeof views);
    for (uint32_t i = 0; i < layout->slot_count; i++) {
        const unsigned char *xip = toy_ckpt_flash_slot_xip(i);
        if (!xip)
            continue;
        toy_ckpt_info_t info;
        if (toy_ckpt_parse_header(xip, layout->slot_bytes, &info) ==
            TOY_CKPT_OK) {
            views[i].header_ok = 1;
            memcpy(views[i].cart_name, info.cart_name,
                   sizeof views[i].cart_name);
            views[i].step_count = info.step_count;
        }
        toy_ckpt_slot_candidate_t cand;
        views[i].valid =
            toy_ckpt_slot_probe(xip, layout->slot_bytes, NULL, &cand,
                                NULL) == TOY_CKPT_OK;
    }
    toy_prod_ckpt_resume_t dec;
    toy_prod_ckpt_resume_decide(views, layout->slot_count, toy_carts,
                                toy_cart_count, &dec);
    if (dec.action == TOY_PROD_RESUME_NONE) {
        printf("TOY-PROD-RESUME none (gallery)\n");
        return;
    }
    if (dec.action == TOY_PROD_RESUME_UNKNOWN_CART) {
        printf("TOY-PROD-RESUME-ERR cart=%s slot=%d "
               "not-in-registry (gallery)\n",
               views[dec.slot].cart_name, dec.slot);
        return;
    }
    if (dec.action != TOY_PROD_RESUME_CART) {
        printf("TOY-PROD-RESUME-ERR no-valid-checkpoint (gallery)\n");
        return;
    }
    const toy_cart_t *cart = toy_carts[dec.cart_index];
    uint32_t slot = (uint32_t)dec.slot;
    if (cart->state_size > sizeof state_buf || !cart->render ||
        cart->demo_steps_per_frame == 0 || toy_cart_check_input(cart) ||
        toy_cart_check_signal(cart)) {
        printf("TOY-PROD-RESUME-ERR cart=%s bad-cart (gallery)\n",
               cart->name);
        return;
    }
    size_t len = 0;
    toy_ckpt_info_t info;
    toy_ckpt_flash_err_t ferr = toy_ckpt_flash_read_slot(
        slot, cart, ckpt_buf, sizeof ckpt_buf, &len, &info);
    if (ferr != TOY_CKPT_FLASH_OK) {
        printf("TOY-PROD-RESUME-ERR cart=%s slot=%lu stage=read "
               "err=%d (gallery)\n",
               cart->name, (unsigned long)slot, (int)ferr);
        return;
    }
    /* Synthetic feed: respawn + seek BEFORE adopting the
     * state. The resumed run stays on the synthetic source
     * for its whole life (the checkpoint's provenance),
     * whatever the IMU is doing this boot. */
    int sig_created = 0;
    int mode = TOY_LIVE_FEED_NONE;
    if (cart->feed) {
        if (cart->sig_spec.proof_kind != TOY_SIG_KIND_SYNTHETIC) {
            printf("TOY-PROD-RESUME-ERR cart=%s stage=feed-kind "
                   "(gallery)\n", cart->name);
            return;
        }
        sig_created = toy_sig_proof_source_init(cart, &prod_sig_src);
        if (sig_created != 1) {
            printf("TOY-PROD-RESUME-ERR cart=%s stage=sig-init "
                   "(gallery)\n", cart->name);
            return;
        }
        if (cart->sig_spec.samples_per_step != 0u &&
            info.step_count >
                UINT64_MAX / cart->sig_spec.samples_per_step) {
            printf("TOY-PROD-RESUME-ERR cart=%s stage=seek-range "
                   "(gallery)\n", cart->name);
            return;
        }
        uint64_t pos = info.step_count *
                       (uint64_t)cart->sig_spec.samples_per_step;
        if (toy_sig_seek_samples(&prod_sig_src, pos) != 0) {
            printf("TOY-PROD-RESUME-ERR cart=%s stage=seek "
                   "samples=%llu (gallery)\n",
                   cart->name, (unsigned long long)pos);
            return;
        }
        mode = TOY_LIVE_FEED_SYNTHETIC;
    }
    const toy_cart_t *loaded = NULL;
    uint64_t loaded_step = 0;
    toy_ckpt_err_t lerr = toy_ckpt_load(ckpt_buf, len, cart, state_buf,
                                        sizeof state_buf, &loaded,
                                        &loaded_step);
    if (lerr != TOY_CKPT_OK || loaded != cart ||
        loaded_step != info.step_count) {
        printf("TOY-PROD-RESUME-ERR cart=%s slot=%lu stage=load "
               "err=%d (%s) (gallery)\n",
               cart->name, (unsigned long)slot, (int)lerr,
               toy_ckpt_strerror(lerr));
        return;
    }
    prod_feed_mode = mode;
    prod_sig_created = sig_created;
    ckpt_resume_cart = cart;
    ckpt_resume_slot = slot;
    ckpt_resume_step = loaded_step;
    ckpt_resume_pending = 1;
    toy_event_t ev;
    memset(&ev, 0, sizeof ev);
    ev.type = TOY_EV_KEY_DOWN;
    ev.id = TOY_KEY_DOWN;
    for (int i = 0; i < dec.cart_index; i++)
        prod_dispatch(&ev);
    ev.id = TOY_KEY_ENTER;
    prod_dispatch(&ev);
    if (run_cart != cart) {
        ckpt_resume_pending = 0;
        run_cart = NULL;
        run_step_count = 0;
        toy_prod_ckpt_disarm(&ckpt_timer);
        prod_feed_mode = TOY_LIVE_FEED_NONE;
        prod_sig_created = 0;
        toy_shell_enter(&shell, TOY_SHELL_GALLERY);
        note_screen();
        printf("TOY-PROD-RESUME-ERR cart=%s stage=launch (gallery)\n",
               cart->name);
    }
}

/* One TOUCHDBG line per DOWN the runner acts on (real or
 * the synthesised tap-launch confirm; never MOVE/UP): the
 * controller-raw point, its post-mirror glass px, the milli
 * coords the shell actually received, the screen it landed
 * on, and the action mask the shell returned. Comparing raw
 * against milli across known finger positions convicts or
 * clears TOY_PROD_TOUCH_MIRROR in one owner session, and
 * irq_edges vs DOWN count shows whether reports still die
 * between edge and drain read. */
static void prod_touchdbg(const char *src, const toy_event_t *ev,
                          int32_t raw_x, int32_t raw_y, uint8_t screen,
                          uint32_t act)
{
    printf("TOY-PROD-TOUCHDBG src=%s id=%lu raw=(%ld,%ld) "
           "glass=(%ld,%ld) milli=(%ld,%ld) screen=%s act=0x%lx "
           "mirror=%d quadrant=%d irq_edges=%lu\n",
           src, (unsigned long)ev->id, (long)raw_x, (long)raw_y,
           (long)toy_prod_touch_axis(raw_x),
           (long)toy_prod_touch_axis(raw_y), (long)ev->x, (long)ev->y,
           screen_name(screen), (unsigned long)act,
           TOY_PROD_TOUCH_MIRROR, orient.quadrant, touch_irq_edges);
}

/* --- input producers -------------------------------------------- */

/* IRQ-latched touch service, run at the top of every frame
 * (before step/render/flush, so this pass's events drive
 * this frame). The drain policy is the sensor-test one
 * unchanged - cst9217_should_read(pending, any-active,
 * 1 Hz probe), reads capped ~50 Hz - except `pending` is
 * now the IRQ latch merged with the live INT level instead
 * of a once-per-frame level sample (touchfix2: pulses that
 * fell and rose during the previous frame's render+flush
 * were invisible to the level sample and waited for the
 * 1 Hz probe). The latch is cleared only when a read
 * actually happens, and before the read, so edges landing
 * during the I2C transfer re-latch for the next pass.
 * NO_REPORT (all-FF window) is benign and never a release;
 * a non-empty frame with no accepted contact is the release
 * frame. Decoded points become events only through the
 * gesture seam; DOWNs additionally feed TOUCHDBG and the
 * gallery tap-launch gate (UP edge confirms a selecting
 * tap with one synthesised DOWN, GALLERY cart rows only). */
static void prod_touch_poll(uint64_t now_ms)
{
    if (!touch_ok)
        return;
    bool pending = toy_prod_touch_pending_merge(
        touch_irq_pending, cst9217_int_asserted(&touch_cfg));
    if (now_ms < next_touch_read_ms ||
        !cst9217_should_read(pending,
                             toy_prod_gesture_any_active(&gesture),
                             now_ms >= next_touch_probe_ms))
        return;
    next_touch_read_ms = now_ms + TOY_PROD_TOUCH_READ_MS;
    next_touch_probe_ms = now_ms + TOY_PROD_TOUCH_PROBE_MS;
    touch_irq_pending = false;
    /* INT-triggered reads get the named settle (busfix): an
     * edge that landed just before this frame-top poll has
     * had ~0 ms for the chip to finish latching its report.
     * An edge a full frame old needs nothing; 1 ms either
     * way is one short sleep before one transfer. */
    if (pending)
        sleep_ms(TOY_PROD_TOUCH_INT_SETTLE_MS);
    cst9217_report_t rep;
    int rc = cst9217_read_report(&touch_cfg, &rep, NULL);
    if (rc == CST9217_NO_REPORT) {
        /* Successful transfer (window reads all-FF): the
         * bus is alive, classified apart from NACKs, never
         * an error - and it clears the wedge counter. */
        toy_prod_bus_note_ok(&bus_health);
        return;
    }
    if (rc != CST9217_OK) {
        touch_err_count++;
        toy_prod_bus_note_error(&bus_health);
        if (touch_err_count == 1u || (touch_err_count % 100u) == 0u)
            printf("TOY-PROD-TOUCH-ERR rc=%d count=%u\n", rc,
                   touch_err_count);
        return;
    }
    toy_prod_bus_note_ok(&bus_health);
    if (rep.empty)
        return;
    bool present[TOY_PROD_MAX_CONTACTS] = { false, false };
    int32_t px[TOY_PROD_MAX_CONTACTS] = { 0, 0 };
    int32_t py[TOY_PROD_MAX_CONTACTS] = { 0, 0 };
    if (rep.valid) {
        for (unsigned i = 0; i < rep.num_points; i++) {
            unsigned id = rep.points[i].track_id;
            if (id < TOY_PROD_MAX_CONTACTS) {
                present[id] = true;
                px[id] = rep.points[i].x;
                py[id] = rep.points[i].y;
            }
        }
    }
    toy_event_t out[6];
    size_t n = toy_prod_gesture_report_rot(
        &gesture, now_ms, toy_shell_screen(&shell) == TOY_SHELL_RUNNING,
        present, px, py, out, 6u, orient.quadrant);
    for (size_t i = 0; i < n; i++) {
        const toy_event_t *ev = &out[i];
        uint8_t pre_screen = toy_shell_screen(&shell);
        uint32_t act = prod_dispatch(ev);
        if (ev->type == TOY_EV_POINTER_DOWN &&
            ev->id < TOY_PROD_MAX_CONTACTS) {
            dbg_raw_x[ev->id] = px[ev->id];
            dbg_raw_y[ev->id] = py[ev->id];
            prod_touchdbg("real", ev, px[ev->id], py[ev->id],
                          pre_screen, act);
            (void)toy_prod_tap_launch_note_down(
                &tap_launch, now_ms, pre_screen == TOY_SHELL_GALLERY,
                (act & TOY_SH_ACT_START_CART) != 0, shell.scroll,
                shell.cart_count, shell.selected, ev);
        } else if (ev->type == TOY_EV_POINTER_MOVE) {
            toy_prod_tap_launch_note_move(&tap_launch, ev);
        } else if (ev->type == TOY_EV_POINTER_UP &&
                   ev->id < TOY_PROD_MAX_CONTACTS) {
            toy_event_t synth;
            if (toy_prod_tap_launch_take_up(
                    &tap_launch, now_ms,
                    toy_shell_screen(&shell) == TOY_SHELL_GALLERY,
                    shell.scroll, shell.cart_count, shell.selected,
                    ev, &synth)) {
                uint8_t synth_screen = toy_shell_screen(&shell);
                uint32_t synth_act = prod_dispatch(&synth);
                prod_touchdbg("synth-tap", &synth, dbg_raw_x[ev->id],
                              dbg_raw_y[ev->id], synth_screen,
                              synth_act);
            }
        }
    }
}

static void prod_gesture_poll(uint64_t now_ms)
{
    if (!touch_ok)
        return;
    toy_event_t out[1];
    size_t n = toy_prod_gesture_poll(&gesture, now_ms, out, 1u);
    for (size_t i = 0; i < n; i++)
        prod_dispatch(&out[i]);
}

/* One IMU sample per frame: tilt through the 33 ms / 20 mg
 * gate, dial through the bias-corrected accumulator with
 * real dt (capped), and (toy5-livefeed) the same reading's
 * raw counts noted into the live-feed hold - one I2C read
 * serves all three, so the feed inherits this cadence, the
 * error back-off below and the shared bus-health notes
 * exactly, and adds no transfers of its own. Read errors
 * back off and count (the hold keeps the last good sample
 * and counts the error too). The same sample also drives
 * the gravity-quadrant tracker (toy5-rotate) BEFORE the
 * feed/tilt below consume it, so a quadrant change applies
 * to this very sample and to the frame about to flush. */
static void prod_imu_poll(uint64_t now_ms)
{
    if (!imu_ok || now_ms < next_imu_ms)
        return;
    qmi8658_milli_t m;
    qmi8658_raw_t raw;
    if (qmi8658_read_milli(bus(), &m, &raw) != 0) {
        toy_live_feed_note_error(&prod_live_feed);
        imu_err_count++;
        toy_prod_bus_note_error(&bus_health);
        if (imu_err_count == 1u || (imu_err_count % 100u) == 0u)
            printf("TOY-PROD-IMU-ERR count=%u\n", imu_err_count);
        next_imu_ms = now_ms + TOY_PROD_IMU_RETRY_MS;
        have_imu_ms = false;
        return;
    }
    toy_prod_bus_note_ok(&bus_health);
    uint32_t dt_ms = 0;
    if (have_imu_ms && now_ms > last_imu_ms) {
        uint64_t d = now_ms - last_imu_ms;
        dt_ms = d > TOY_PROD_DIAL_MAX_DT_MS ? TOY_PROD_DIAL_MAX_DT_MS
                                            : (uint32_t)d;
    }
    last_imu_ms = now_ms;
    have_imu_ms = true;
    if (toy_orient_update(&orient, m.ax_mg, m.ay_mg, now_ms)) {
        toy_presenter_set_rotation(&presenter, orient.quadrant);
        /* Next sample always sends a TILT in the new canonical
         * frame, even if it lands within 20 mg of the last
         * sent vector from the old quadrant. */
        toy_prod_tilt_gate_init(&tilt_gate);
        printf("TOY-PROD-ORIENT quadrant=%d\n", orient.quadrant);
    }
    {
        const int16_t raw6[6] = { raw.ax, raw.ay, raw.az,
                                  raw.gx, raw.gy, raw.gz };
        toy_live_feed_note_sample_rot(&prod_live_feed, raw6, now_ms,
                                      orient.quadrant);
    }
    toy_event_t ev;
    if (toy_prod_tilt_gate_poll_rot(&tilt_gate, now_ms, m.ax_mg, m.ay_mg,
                                    &ev, orient.quadrant))
        prod_dispatch(&ev);
    if (toy_prod_dial_feed(&dial, m.gz_mdps, dt_ms, &ev))
        prod_dispatch(&ev);
}

/* --- shared-bus recovery (busfix) -------------------------------- */

/* One documented I2C bus reset + re-probe of every device
 * that was healthy at boot. Runs only from the main loop
 * (all I2C is single-flight there; the GPIO22 IRQ is
 * disabled across the attempt so no latch can queue a read
 * onto mid-recovery pins). Sequence (NXP UM10204 bus-reset
 * shape): i2c_deinit; sample SDA/SCL as pulled-up inputs
 * (the wedge's fingerprints - SDA low convicts a slave
 * left mid-byte by an aborted transfer); drive SCL for up
 * to 9 clocks until SDA releases (a slave interrupted
 * mid-byte lets go within 9); force a STOP (SDA low while
 * SCL low, SCL high, SDA released); re-init at 400 kHz;
 * re-probe RTC (init + read), touch (verified normal-mode
 * handshake, escalating once to a GPIO23 reset pulse +
 * handshake), IMU (probe + full config, dial bias kept -
 * no 1 s still recalibration of a device in a hand).
 * Gesture/tap state resets so a contact interrupted by
 * the wedge cannot stick. The consecutive-error counter
 * resets only if a probe read succeeded (pure seam). */
static void prod_bus_recover(void)
{
    printf("TOY-PROD-BUS-RECOVER begin n=%u consec=%u "
           "touch_err=%u imu_err=%u\n",
           bus_health.recoveries + 1u, bus_health.consec_err,
           touch_err_count, imu_err_count);
    if (expect_touch)
        gpio_set_irq_enabled(PIN_TOUCH_INT, GPIO_IRQ_EDGE_FALL,
                             false);
    i2c_deinit(i2c1);
    gpio_init(PIN_I2C_SDA);
    gpio_init(PIN_I2C_SCL);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    gpio_set_dir(PIN_I2C_SCL, GPIO_IN);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    sleep_us(10);
    int sda_was = gpio_get(PIN_I2C_SDA) ? 1 : 0;
    int scl_was = gpio_get(PIN_I2C_SCL) ? 1 : 0;
    gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
    gpio_put(PIN_I2C_SCL, 1);
    unsigned clocks = 0;
    while (!gpio_get(PIN_I2C_SDA) && clocks < 9u) {
        gpio_put(PIN_I2C_SCL, 0);
        sleep_us(5);
        gpio_put(PIN_I2C_SCL, 1);
        sleep_us(5);
        clocks++;
    }
    /* STOP: SDA low -> (SCL high) -> SDA released high. */
    gpio_put(PIN_I2C_SCL, 0);
    gpio_set_dir(PIN_I2C_SDA, GPIO_OUT);
    gpio_put(PIN_I2C_SDA, 0);
    sleep_us(5);
    gpio_put(PIN_I2C_SCL, 1);
    sleep_us(5);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN); /* pull-up: SDA rises */
    sleep_us(10);
    /* Same bring-up order as boot. */
    i2c_init(i2c1, 400u * 1000u);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    bool probe_any = false;
    if (expect_rtc) {
        rtc_ok = (toy_clock_rtc_reprobe() == 0);
        probe_any = probe_any || rtc_ok;
    }
    if (expect_touch) {
        int rc = cst9217_enter_normal(&touch_cfg);
        if (rc != CST9217_OK &&
            cst9217_reset(&touch_cfg) == CST9217_OK)
            rc = cst9217_enter_normal(&touch_cfg);
        touch_ok = (rc == CST9217_OK);
        probe_any = probe_any || touch_ok;
    }
    if (expect_imu) {
        uint8_t who = 0, rev = 0;
        int rc = qmi8658_probe(bus(), &who, &rev);
        if (rc == 0)
            rc = qmi8658_init(bus());
        imu_ok = (rc == 0);
        probe_any = probe_any || imu_ok;
    }
    bool device_down = (expect_rtc && !rtc_ok) ||
                       (expect_touch && !touch_ok) ||
                       (expect_imu && !imu_ok);
    toy_prod_bus_note_recovery(&bus_health, toy_clock_mono_ms(),
                               probe_any, device_down);
    /* Re-sync input state: a pre-wedge press must not stick,
     * and the next polls should run promptly, not wait out
     * stale deadlines. */
    toy_prod_gesture_init(&gesture);
    toy_prod_tap_launch_init(&tap_launch);
    have_imu_ms = false;
    next_touch_read_ms = 0;
    next_touch_probe_ms = 0;
    next_imu_ms = 0;
    touch_irq_pending = false;
    if (expect_touch)
        gpio_set_irq_enabled_with_callback(PIN_TOUCH_INT,
                                           GPIO_IRQ_EDGE_FALL,
                                           true, &prod_touch_irq);
    printf("TOY-PROD-BUS-RECOVER done n=%u ok=%d sda=%d scl=%d "
           "clocks=%u touch=%d imu=%d rtc=%d "
           "(expected t=%d i=%d r=%d)\n",
           bus_health.recoveries, probe_any ? 1 : 0, sda_was,
           scl_was, clocks, touch_ok ? 1 : 0, imu_ok ? 1 : 0,
           rtc_ok ? 1 : 0, expect_touch ? 1 : 0,
           expect_imu ? 1 : 0, expect_rtc ? 1 : 0);
}

/* --- RUNNING frame ---------------------------------------------- */

/* Time-boxed stepper (toy5-pace, unchanged arithmetic): at
 * least one step, then until demo_steps_per_frame ran or the
 * budget deadline passed; feed -> apply -> step per step,
 * live queue under the shared cap. Returns steps run, or -1
 * on a signal-feed failure (caller bails to the gallery). */
static int prod_step_frame(void)
{
    const toy_cart_t *cart = run_cart;
    uint64_t deadline = time_us_64() + TOY_PROD_SIM_BUDGET_US;
    uint32_t steps_done = 0;
    for (uint32_t s = 0; s < cart->demo_steps_per_frame; s++) {
        if (prod_feed_mode == TOY_LIVE_FEED_LIVE) {
            /* Live runner path (toy5-livefeed): fill from
             * the hold and call feed() directly - the
             * proof-strict toy_feed_step stays synthetic-
             * only. No good sample yet: step nothing this
             * frame, the cart renders NO SIG; never feed
             * fabricated counts. */
            uint32_t flags = 0;
            uint32_t got = toy_live_feed_fill(
                &prod_live_feed, prod_sig_frame,
                cart->sig_spec.samples_per_step,
                toy_clock_mono_ms(), &flags);
            if (got == 0)
                break;
            cart->feed(state_buf, prod_sig_frame, got,
                       prod_live_info.channels, &prod_live_info,
                       flags);
            toy_apply_queue(cart, state_buf, &prod_live_queue,
                            TOY_MAX_EVENTS_PER_STEP);
            cart->step(state_buf);
            run_step_count++;
            steps_done++;
            if (steps_done < cart->demo_steps_per_frame &&
                time_us_64() >= deadline)
                break;
            continue;
        }
        int frc = toy_feed_step(cart, state_buf,
                                prod_sig_created ? &prod_sig_src : 0,
                                prod_sig_frame,
                                TOY_SIG_MAX_FRAME_VALUES);
        if (frc) {
            printf("TOY-PROD-SIG-ERR cart=%s stage=feed code=%d\n",
                   cart->name, frc);
            return -1;
        }
        toy_apply_queue(cart, state_buf, &prod_live_queue,
                        TOY_MAX_EVENTS_PER_STEP);
        cart->step(state_buf);
        run_step_count++;
        steps_done++;
        if (steps_done < cart->demo_steps_per_frame &&
            time_us_64() >= deadline)
            break;
    }
    return (int)steps_done;
}

static void prod_pace_account(uint64_t sim_us, uint64_t render_us,
                              uint64_t flush_us, unsigned steps)
{
    pace_sim_us += sim_us;
    pace_render_us += render_us;
    pace_flush_us += flush_us;
    pace_frames++;
    pace_steps += steps;
    uint64_t now = time_us_64();
    if (run_cart && now >= pace_mark_us) {
        pace_seconds++;
        printf("TOY-PROD-PACE cart=%s sim_us=%llu render_us=%llu "
               "flush_us=%llu frames=%lu steps=%lu seconds=%lu\n",
               run_cart->name, (unsigned long long)pace_sim_us,
               (unsigned long long)pace_render_us,
               (unsigned long long)pace_flush_us, pace_frames, pace_steps,
               pace_seconds);
        if (prod_feed_mode == TOY_LIVE_FEED_LIVE) {
            /* 1 Hz live-feed telemetry (toy5-livefeed):
             * fresh/rep = samples fed this window that were
             * a new reading / a hold repeat; err = IMU read
             * errors (held, never fabricated); max_age_ms =
             * oldest held reading actually fed; fresh_hz_x10
             * and cons_hz_x10 are the achieved new-reading
             * rate and total consumption rate (samples fed
             * per wall second, x10) over this window's real
             * elapsed time; warp_x100 = cons/spec_hz x100
             * (100 = cart time runs at wall rate). */
            uint64_t el = now > feed_last_us ? now - feed_last_us : 0;
            uint64_t fresh_x10 = el ? (uint64_t)prod_live_feed.fresh_w *
                                          10000000ull / el
                                    : 0;
            uint64_t cons_x10 =
                el ? (uint64_t)(prod_live_feed.fresh_w +
                                prod_live_feed.rep_w) *
                         10000000ull / el
                   : 0;
            uint64_t warp_x100 =
                run_cart->sig_spec.sample_rate_hz
                    ? cons_x10 * 10ull /
                          run_cart->sig_spec.sample_rate_hz
                    : 0;
            printf("TOY-PROD-FEED cart=%s src=live fresh=%lu rep=%lu "
                   "err=%lu max_age_ms=%lu fresh_hz_x10=%llu "
                   "cons_hz_x10=%llu spec_hz=%lu warp_x100=%llu\n",
                   run_cart->name,
                   (unsigned long)prod_live_feed.fresh_w,
                   (unsigned long)prod_live_feed.rep_w,
                   (unsigned long)prod_live_feed.err_w,
                   (unsigned long)prod_live_feed.max_age_w,
                   (unsigned long long)fresh_x10,
                   (unsigned long long)cons_x10,
                   (unsigned long)run_cart->sig_spec.sample_rate_hz,
                   (unsigned long long)warp_x100);
            toy_live_feed_reset_window(&prod_live_feed);
            feed_last_us = now;
        }
        pace_sim_us = pace_render_us = pace_flush_us = 0;
        pace_frames = 0;
        pace_steps = 0;
        pace_mark_us = now + 1000000ull;
    }
}

#if TOY_PROD_SELFTEST
/* Retained verbatim from the presenter parcel; see history
 * for what each stage discriminates. */
static int selftest_legacy_perrow_flush(void)
{
    toy_fb_t *fb = toy_presenter_fb(&presenter);
    for (int dy = 0; dy < CO5300_HEIGHT; dy++) {
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

/* --- boot -------------------------------------------------------- */

static const char *tz_source_name(int src)
{
    switch (src) {
    case TOY_CLOCK_TZ_RAM_BYTE: return "rtc-ram";
    case TOY_CLOCK_TZ_SESSION: return "session-unpersisted";
    default: return "default";
    }
}

static void boot_rtc(void)
{
    int rc = toy_clock_rtc_bind(bus());
    if (rc != 0) {
        printf("TOY-PROD-RTC-ERR rc=%d (clock unset; shell continues)\n",
               rc);
        rtc_ok = false;
        expect_rtc = false;
        return;
    }
    rtc_ok = true;
    expect_rtc = true; /* healthy at boot: owed bus re-probes */
    toy_time_t now = toy_clock_now();
    if (now.valid) {
        unsigned m = toy_clock_minutes_of_day(now);
        printf("TOY-PROD-RTC ok valid=1 utc=%lld tz=%ld tz_src=%s "
               "local=%02u:%02u\n",
               (long long)now.utc_seconds, (long)now.tz_offset_min,
               tz_source_name(toy_clock_rtc_tz_source()), m / 60u, m % 60u);
    } else {
        printf("TOY-PROD-RTC ok valid=0 tz=%ld tz_src=%s "
               "(unset/stopped: set it from SET CLOCK)\n",
               (long)now.tz_offset_min,
               tz_source_name(toy_clock_rtc_tz_source()));
    }
}

static void boot_touch(void)
{
    touch_cfg.i2c = bus();
    touch_cfg.rst_gpio = PIN_TOUCH_RST;
    touch_cfg.int_gpio = PIN_TOUCH_INT;
    int rc = cst9217_reset(&touch_cfg);
    cst9217_info_t info;
    memset(&info, 0, sizeof info);
    if (rc == CST9217_OK)
        rc = cst9217_read_info(&touch_cfg, &info);
    int rc2 = (rc == CST9217_OK) ? cst9217_enter_normal(&touch_cfg)
                                 : rc;
    if (rc2 != CST9217_OK) {
        printf("TOY-PROD-TOUCH-ERR rc=%d (no touch; dial/tilt only)\n",
               rc2);
        touch_ok = false;
        expect_touch = false;
        return;
    }
    touch_ok = true;
    expect_touch = true;
    /* Latch INT edges from here on (pin already configured
     * input+pull-up by cst9217_reset). Clearing the latch
     * before enabling drops any boot transient; real edges
     * after this point are the owner's. */
    touch_irq_pending = false;
    touch_irq_edges = 0;
    gpio_set_irq_enabled_with_callback(PIN_TOUCH_INT,
                                       GPIO_IRQ_EDGE_FALL, true,
                                       &prod_touch_irq);
    printf("TOY-PROD-TOUCH ok chip=0x%04x res=%ux%u mirror=%d "
           "int_gpio=%u rst_gpio=%u irq=edge-fall-latched\n",
           info.chip_type, info.resolution_x, info.resolution_y,
           TOY_PROD_TOUCH_MIRROR, PIN_TOUCH_INT, PIN_TOUCH_RST);
}

static void boot_imu_and_calibrate(void)
{
    uint8_t who = 0, rev = 0;
    int rc = qmi8658_probe(bus(), &who, &rev);
    if (rc == 0)
        rc = qmi8658_init(bus());
    if (rc != 0) {
        printf("TOY-PROD-IMU-ERR rc=%d who=0x%02x (no tilt/dial)\n", rc,
               who);
        imu_ok = false;
        expect_imu = false;
        toy_prod_dial_init(&dial, 0);
        return;
    }
    imu_ok = true;
    expect_imu = true;
    printf("TOY-PROD-IMU ok who=0x%02x rev=0x%02x odr_hz=%u\n", who, rev,
           QMI8658_ODR_HZ);
    /* ~1 s still-desk offer. Whatever the board is doing
     * (held, moving), the pure gate decides; a rejection is
     * a printed skip with bias 0, never a guess. */
    toy_prod_cal_t cal;
    toy_prod_cal_init(&cal);
    unsigned got = 0;
    for (unsigned i = 0; i < TOY_PROD_CAL_SAMPLES; i++) {
        qmi8658_milli_t m;
        if (qmi8658_read_milli(bus(), &m, NULL) == 0) {
            toy_prod_cal_add(&cal, m.ax_mg, m.ay_mg, m.az_mg, m.gz_mdps);
            got++;
        }
        sleep_ms(TOY_PROD_CAL_PERIOD_MS);
    }
    int32_t bias = 0;
    if (toy_prod_cal_finish(&cal, &bias)) {
        printf("TOY-PROD-CAL ok bias_mdps=%ld samples=%u\n", (long)bias,
               got);
    } else {
        bias = 0;
        printf("TOY-PROD-CAL skipped samples=%u (not still at 1g; "
               "dial bias=0)\n", got);
    }
    toy_prod_dial_init(&dial, bias);
}

int main(void)
{
    stdio_init_all();
    for (int i = 0; i < 150 && !stdio_usb_connected(); i++)
        sleep_ms(100);
    sleep_ms(200);

    char idstr[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    pico_get_unique_board_id_string(idstr, sizeof idstr);
    printf("TOY-PROD-BOOT git %s serial %s clk_hz %lu\n", TOY_GIT_HEAD,
           idstr, (unsigned long)clock_get_hz(clk_sys));
    printf("TOY-PROD-PRESENTER mode=%s fb=%dx%d fb_bytes=%lu carts=%d "
           "dbv=0x%02x shell=1\n",
           TOY_PRESENTER_MODE_NAME, TOY_PRESENTER_FB_W, TOY_PRESENTER_FB_H,
           (unsigned long)TOY_PRESENTER_FB_BYTES, toy_cart_count,
           TOY_PROD_DBV);
    printf("TOY-PROD-MEM data_bytes=%lu bss_bytes=%lu heap_free_bytes=%lu "
           "stack_reserved_bytes=%lu state_buf_bytes=%lu "
           "ckpt_buf_bytes=%lu\n",
           (unsigned long)(&__data_end__ - &__data_start__),
           (unsigned long)(&__bss_end__ - &__bss_start__),
           (unsigned long)(&__HeapLimit - &__end__),
           (unsigned long)(&__StackTop - &__StackBottom),
           (unsigned long)sizeof state_buf,
           (unsigned long)sizeof ckpt_buf);

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
    (void)toy_presenter_blank(&presenter); /* GRAM undefined: black first */
#if TOY_PROD_SELFTEST
    prod_selftest();
#endif

    printf("TOY-PROD-PHASE sensors i2c1 sda=%u scl=%u hz=400000\n",
           PIN_I2C_SDA, PIN_I2C_SCL);
    i2c_init(i2c1, 400u * 1000u);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    boot_rtc();
    boot_touch();
    boot_imu_and_calibrate();

    toy_live_feed_init(&prod_live_feed);
    toy_prod_bus_init(&bus_health);
    toy_orient_init(&orient); /* boot quadrant 0 (toy5-rotate) */
    toy_prod_gesture_init(&gesture);
    toy_prod_tap_launch_init(&tap_launch);
    toy_prod_tilt_gate_init(&tilt_gate);
    toy_queue_clear(&prod_live_queue);
    toy_prod_ckpt_timer_init(&ckpt_timer);
    toy_shell_init(&shell, toy_carts, toy_cart_count);
    prev_screen = 0xFFu;
    note_screen();
    printf("TOY-PROD-SHELL carts=%d touch=%d imu=%d (gallery first; "
           "controls in board/PRODUCT.md)\n",
           toy_cart_count, touch_ok ? 1 : 0, imu_ok ? 1 : 0);
    /* Desk-toy resume (toy5-flashprod): a valid newest
     * checkpoint auto-starts its cart from where power
     * left it; anything else stays in the gallery above. */
    prod_ckpt_try_resume();

    toy_fb_t *fb = toy_presenter_fb(&presenter);
    for (;;) {
        uint64_t now_ms = toy_clock_mono_ms();
        prod_touch_poll(now_ms);
        prod_gesture_poll(now_ms);
        prod_imu_poll(now_ms);
        /* Wedge self-heal (busfix): owed after a run of
         * consecutive transport errors, before this frame's
         * render asks the (possibly dead) RTC for the time.
         * Costs one long frame (~150 ms: bus reset + IMU
         * settle) and only ever runs on a sick bus. */
        if (toy_prod_bus_should_recover(&bus_health, now_ms))
            prod_bus_recover();

        uint64_t t0 = time_us_64();
        bool cart_frame = false;
        unsigned steps = 0;
        if (toy_shell_screen(&shell) == TOY_SHELL_RUNNING && run_cart) {
            int ran = prod_step_frame();
            if (ran < 0) {
                prod_stop_cart("sig-feed-error");
                toy_shell_enter(&shell, TOY_SHELL_GALLERY);
                note_screen();
            } else {
                steps = (unsigned)ran;
                cart_frame = true;
            }
        }
        uint64_t t1 = time_us_64();
        if (cart_frame)
            run_cart->render(state_buf, fb);
        else
            toy_shell_render(&shell, fb, toy_clock_now());
        uint64_t t2 = time_us_64();
        rc = toy_presenter_flush(&presenter);
        uint64_t t3 = time_us_64();
        if (rc != CO5300_OK)
            printf("TOY-PROD-FLUSH-ERR rc=%d screen=%s\n", rc,
                   screen_name(toy_shell_screen(&shell)));
        if (cart_frame)
            prod_pace_account(t1 - t0, t2 - t1, t3 - t2, steps);
        /* Cadence checkpoint (toy5-flashprod): strictly
         * after this frame's flush and before the next
         * frame's step, so the measured XIP stall lands
         * between frames, never inside step/render/flush. */
        if (cart_frame &&
            toy_prod_ckpt_due(&ckpt_timer, toy_clock_mono_ms(),
                              TOY_PROD_CKPT_INTERVAL_MS))
            prod_ckpt_save("cadence");

        (void)toy_shell_tick(&shell);
        note_screen();
    }
}

