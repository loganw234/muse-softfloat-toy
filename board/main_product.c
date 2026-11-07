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
 *   stepper (feed -> apply live queue -> step, synthetic
 *   on-device signal source as before - live IMU feed is a
 *   later parcel) and renders the cart; every other screen
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
 */
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/unique_id.h"

#include "clock_civil.h"
#include "clock_rtc.h"
#include "display_co5300.h"
#include "display_qspi_pio.h"
#include "imu_qmi8658.h"
#include "presenter.h"
#include "product_input.h"
#include "rtc_pcf85063.h"
#include "touch_cst9217.h"
#include "toy_cart.h"
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
static toy_presenter_t presenter;
static co5300_t panel;
static toy_shell_t shell;

/* Running cart (shell's RUNNING screen), or NULL. */
static const toy_cart_t *run_cart;
static toy_queue_t prod_live_queue;
static toy_sig_source_t prod_sig_src;
static int prod_sig_created;
static int16_t prod_sig_frame[TOY_SIG_MAX_FRAME_VALUES];

/* PACE window (RUNNING only), reset at each cart start. */
static uint64_t pace_mark_us, pace_sim_us, pace_render_us, pace_flush_us;
static unsigned long pace_frames, pace_steps, pace_seconds;

/* Sensors: ok flags degrade per-device, never wedge. */
static cst9217_config_t touch_cfg;
static bool touch_ok, imu_ok;
static toy_prod_gesture_t gesture;
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

static void prod_stop_cart(const char *why)
{
    if (run_cart)
        printf("TOY-PROD-EVENT exit cart=%s why=%s\n", run_cart->name, why);
    run_cart = NULL;
    prod_sig_created = 0;
    toy_queue_clear(&prod_live_queue);
}

static void prod_start_cart(void)
{
    const toy_cart_t *cart = toy_shell_current(&shell);
    if (!cart) {
        printf("TOY-PROD-EVENT start-failed no-current-cart\n");
        toy_shell_enter(&shell, TOY_SHELL_GALLERY);
        return;
    }
    if (cart->state_size > sizeof state_buf || !cart->render ||
        cart->demo_steps_per_frame == 0 || toy_cart_check_input(cart) ||
        toy_cart_check_signal(cart)) {
        printf("TOY-PROD-BAD-CART cart=%s (staying in gallery)\n",
               cart->name);
        toy_shell_enter(&shell, TOY_SHELL_GALLERY);
        return;
    }
    int sig_created = toy_sig_proof_source_init(cart, &prod_sig_src);
    if (sig_created < 0) {
        printf("TOY-PROD-SIG-ERR cart=%s stage=init\n", cart->name);
        toy_shell_enter(&shell, TOY_SHELL_GALLERY);
        return;
    }
    prod_sig_created = sig_created;
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
    if (act & TOY_SH_ACT_EXIT_TO_GALLERY)
        prod_stop_cart("shell-exit");
    if (act & TOY_SH_ACT_CLOCK_COMMIT)
        prod_clock_commit();
    if (act & TOY_SH_ACT_FORWARD_TO_CART)
        toy_queue_push(&prod_live_queue, ev);
    note_screen();
    return act;
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
           "mirror=%d irq_edges=%lu\n",
           src, (unsigned long)ev->id, (long)raw_x, (long)raw_y,
           (long)toy_prod_touch_axis(raw_x),
           (long)toy_prod_touch_axis(raw_y), (long)ev->x, (long)ev->y,
           screen_name(screen), (unsigned long)act,
           TOY_PROD_TOUCH_MIRROR, touch_irq_edges);
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
    cst9217_report_t rep;
    int rc = cst9217_read_report(&touch_cfg, &rep, NULL);
    if (rc == CST9217_NO_REPORT)
        return;
    if (rc != CST9217_OK) {
        touch_err_count++;
        if (touch_err_count == 1u || (touch_err_count % 100u) == 0u)
            printf("TOY-PROD-TOUCH-ERR rc=%d count=%u\n", rc,
                   touch_err_count);
        return;
    }
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
    size_t n = toy_prod_gesture_report(
        &gesture, now_ms, toy_shell_screen(&shell) == TOY_SHELL_RUNNING,
        present, px, py, out, 6u);
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
 * real dt (capped). Read errors back off and count. */
static void prod_imu_poll(uint64_t now_ms)
{
    if (!imu_ok || now_ms < next_imu_ms)
        return;
    qmi8658_milli_t m;
    if (qmi8658_read_milli(bus(), &m, NULL) != 0) {
        imu_err_count++;
        if (imu_err_count == 1u || (imu_err_count % 100u) == 0u)
            printf("TOY-PROD-IMU-ERR count=%u\n", imu_err_count);
        next_imu_ms = now_ms + TOY_PROD_IMU_RETRY_MS;
        have_imu_ms = false;
        return;
    }
    uint32_t dt_ms = 0;
    if (have_imu_ms && now_ms > last_imu_ms) {
        uint64_t d = now_ms - last_imu_ms;
        dt_ms = d > TOY_PROD_DIAL_MAX_DT_MS ? TOY_PROD_DIAL_MAX_DT_MS
                                            : (uint32_t)d;
    }
    last_imu_ms = now_ms;
    have_imu_ms = true;
    toy_event_t ev;
    if (toy_prod_tilt_gate_poll(&tilt_gate, now_ms, m.ax_mg, m.ay_mg, &ev))
        prod_dispatch(&ev);
    if (toy_prod_dial_feed(&dial, m.gz_mdps, dt_ms, &ev))
        prod_dispatch(&ev);
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
        return;
    }
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
        return;
    }
    touch_ok = true;
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
        toy_prod_dial_init(&dial, 0);
        return;
    }
    imu_ok = true;
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

    toy_prod_gesture_init(&gesture);
    toy_prod_tap_launch_init(&tap_launch);
    toy_prod_tilt_gate_init(&tilt_gate);
    toy_queue_clear(&prod_live_queue);
    toy_shell_init(&shell, toy_carts, toy_cart_count);
    prev_screen = 0xFFu;
    note_screen();
    printf("TOY-PROD-SHELL carts=%d touch=%d imu=%d (gallery first; "
           "controls in board/PRODUCT.md)\n",
           toy_cart_count, touch_ok ? 1 : 0, imu_ok ? 1 : 0);

    toy_fb_t *fb = toy_presenter_fb(&presenter);
    for (;;) {
        uint64_t now_ms = toy_clock_mono_ms();
        prod_touch_poll(now_ms);
        prod_gesture_poll(now_ms);
        prod_imu_poll(now_ms);

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

        (void)toy_shell_tick(&shell);
        note_screen();
    }
}

