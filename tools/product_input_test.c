/* product_input_test.c - standalone host test for the
 * toy5-shell product pure seams: board/clock_civil.h (civil
 * <-> epoch, CLOCK_COMMIT combiner, tz RAM-byte encoding)
 * and board/product_input.h (touch mapping, gesture state
 * machine, tilt gate, dial accumulator, still calibration).
 *
 * Deliberately NOT part of tools/run.sh (mirroring
 * clock_test.c / sensor_drivers_test.c): build and run with
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard -Icore \
 *       tools/product_input_test.c -o build/product_input_test
 * Exit status 0 iff every check passes.
 *
 * Oracles are independent of the code under test where it
 * matters: epoch anchors are the well-known constants
 * (1970-01-01 = 0, 2000-01-01 = 946684800, 2026-11-07
 * 12:00 UTC by hand day count: 56*365+14 leaps to 2026-01-01
 * = 20454, +310 days to Nov 7 = day 20764, +noon),
 * leap behaviour is the Gregorian rule stated in
 * cart_solclock.c, and milli expectations are computed from
 * toy_px_to_milli's own contract by hand (centre 233 on a
 * 466 glass: px 0 -> -1000, 465 -> (465-233)*1000/233 = 996).
 */
#include <stdio.h>
#include <string.h>

#include "clock_civil.h"
#include "product_input.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, what)                                              \
    do {                                                               \
        checks++;                                                      \
        if (!(cond)) {                                                 \
            failures++;                                                \
            printf("  FAIL %s (line %d): %s\n", __func__, __LINE__,    \
                   what);                                              \
        }                                                              \
    } while (0)

/* --- clock_civil ------------------------------------------------ */

static void test_epoch_anchors(void)
{
    toy_civil_t c;
    c = (toy_civil_t){ 1970, 1, 1, 0, 0, 0 };
    CHECK(toy_civil_to_epoch(&c) == 0, "epoch itself = 0");
    c = (toy_civil_t){ 2000, 1, 1, 0, 0, 0 };
    CHECK(toy_civil_to_epoch(&c) == 946684800ll,
          "2000-01-01 = 946684800 (solclock day 0, epoch day 10957)");
    CHECK(toy_days_from_civil(2000, 1, 1) == 10957, "day anchor 10957");
    toy_epoch_to_civil(946684800ll, &c);
    CHECK(c.year == 2000 && c.month == 1 && c.day == 1 && c.hour == 0 &&
              c.minute == 0 && c.second == 0,
          "946684800 -> 2000-01-01 00:00:00");
    toy_epoch_to_civil(0, &c);
    CHECK(c.year == 1970 && c.month == 1 && c.day == 1, "0 -> epoch date");
    toy_epoch_to_civil(-1, &c);
    CHECK(c.year == 1969 && c.month == 12 && c.day == 31 &&
              c.hour == 23 && c.minute == 59 && c.second == 59,
          "-1 floors to 1969-12-31 23:59:59");
    /* The live RTC's era: set 2026-11-07 by the sensors parcel. */
    c = (toy_civil_t){ 2026, 11, 7, 12, 0, 0 };
    CHECK(toy_civil_to_epoch(&c) == 20764ll * 86400ll + 43200ll,
          "2026-11-07 12:00 = day 20764 + noon");
    CHECK(toy_civil_weekday(1970, 1, 1) == 4, "epoch was Thursday (4)");
    CHECK(toy_civil_weekday(2000, 1, 1) == 6, "2000-01-01 was Saturday");
    CHECK(toy_civil_weekday(2026, 11, 7) == 6, "2026-11-07 is Saturday");
}

static void test_leap_years(void)
{
    CHECK(toy_civil_is_leap(2000), "2000 leap (div by 400)");
    CHECK(!toy_civil_is_leap(1900), "1900 not leap (div by 100)");
    CHECK(!toy_civil_is_leap(2100), "2100 not leap (div by 100)");
    CHECK(toy_civil_is_leap(2028), "2028 leap");
    CHECK(!toy_civil_is_leap(2026), "2026 not leap");
    /* Feb 29 exists exactly in leap years, and Mar 1 follows
     * Feb 28/29 with no gap in day counts either way. */
    CHECK(toy_days_from_civil(2000, 3, 1) -
                  toy_days_from_civil(2000, 2, 28) == 2,
          "2000 has Feb 29 between Feb 28 and Mar 1");
    CHECK(toy_days_from_civil(2100, 3, 1) -
                  toy_days_from_civil(2100, 2, 28) == 1,
          "2100 has no Feb 29");
    CHECK(toy_days_from_civil(2028, 3, 1) -
                  toy_days_from_civil(2028, 2, 28) == 2,
          "2028 has Feb 29");
    toy_civil_t feb29 = { 2100, 2, 29, 0, 0, 0 };
    CHECK(!toy_civil_valid(&feb29), "2100-02-29 invalid");
    feb29.year = 2000;
    CHECK(toy_civil_valid(&feb29), "2000-02-29 valid");
    toy_civil_t apr31 = { 2026, 4, 31, 0, 0, 0 };
    CHECK(!toy_civil_valid(&apr31), "Apr 31 invalid");
    toy_civil_t bad_h = { 2026, 1, 1, 24, 0, 0 };
    CHECK(!toy_civil_valid(&bad_h), "hour 24 invalid");
    /* Year lengths via Jan-1 deltas. */
    CHECK(toy_days_from_civil(2001, 1, 1) -
                  toy_days_from_civil(2000, 1, 1) == 366,
          "year 2000 has 366 days");
    CHECK(toy_days_from_civil(2101, 1, 1) -
                  toy_days_from_civil(2100, 1, 1) == 365,
          "year 2100 has 365 days");
}

static void test_round_trips(void)
{
    /* Every day 1999-12-01..2100-03-01 round-trips and steps
     * by exactly 86400 s at midnight (the RTC century, plus
     * the 2100 non-leap boundary). */
    int64_t d0 = toy_days_from_civil(1999, 12, 1);
    int64_t d1 = toy_days_from_civil(2100, 3, 1);
    int bad = 0;
    for (int64_t d = d0; d <= d1; d++) {
        int32_t y, m, dd;
        toy_civil_from_days(d, &y, &m, &dd);
        if (toy_days_from_civil(y, m, dd) != d)
            bad++;
        toy_civil_t c = { y, m, dd, 23, 59, 58 };
        if (!toy_civil_valid(&c))
            bad++;
        toy_civil_t back;
        toy_epoch_to_civil(toy_civil_to_epoch(&c), &back);
        if (memcmp(&c, &back, sizeof c) != 0)
            bad++;
    }
    CHECK(bad == 0, "daily round-trips 1999-12..2100-03 all exact");
    /* Pre-epoch span (floor semantics). */
    bad = 0;
    for (int64_t s = -86400ll * 400; s < 0; s += 3607) {
        toy_civil_t c;
        toy_epoch_to_civil(s, &c);
        if (toy_civil_to_epoch(&c) != s)
            bad++;
    }
    CHECK(bad == 0, "pre-epoch sampled round-trips exact");
}

static void test_commit_and_tz(void)
{
    /* Base: local 2026-11-07 09:15:42 at UTC-7. Edit to
     * 18:30 at UTC+1: same local date, seconds kept,
     * utc = local - 60 min. */
    toy_civil_t base = { 2026, 11, 7, 9, 15, 42 };
    int64_t base_local = toy_civil_to_epoch(&base);
    int64_t utc = toy_clock_commit_utc(base_local, 18, 30, 60);
    toy_civil_t want_local = { 2026, 11, 7, 18, 30, 42 };
    CHECK(utc == toy_civil_to_epoch(&want_local) - 3600,
          "commit keeps local date + seconds, applies new tz");
    /* Same hh:mm, owner tz: utc = local - tz, seconds kept. */
    utc = toy_clock_commit_utc(base_local, 9, 15, -420);
    CHECK(utc == base_local - (-420ll) * 60,
          "commit utc = local - tz (seconds preserved)");

    int32_t tz = 0;
    CHECK(toy_tz_ram_encode(-720) == 1, "tz -720 -> byte 1");
    CHECK(toy_tz_ram_encode(840) == 105, "tz +840 -> byte 105");
    CHECK(toy_tz_ram_encode(-420) == 21, "tz -420 (owner) -> byte 21");
    CHECK(toy_tz_ram_encode(0) == 49, "tz 0 -> byte 49");
    CHECK(toy_tz_ram_encode(330) == 71, "tz +330 -> byte 71");
    CHECK(toy_tz_ram_encode(331) == -1, "odd-minute tz unpersistable");
    CHECK(toy_tz_ram_encode(900) == -1, "out-of-range tz unpersistable");
    CHECK(toy_tz_ram_encode(-721) == -1, "below-range tz unpersistable");
    CHECK(!toy_tz_ram_decode(0, &tz), "byte 0 = never stored");
    CHECK(!toy_tz_ram_decode(106, &tz), "byte 106 foreign, rejected");
    CHECK(!toy_tz_ram_decode(255, &tz), "byte 255 rejected");
    int bad = 0;
    for (int32_t v = TOY_TZ_RAM_MIN; v <= TOY_TZ_RAM_MAX;
         v += TOY_TZ_RAM_STEP) {
        int enc = toy_tz_ram_encode(v);
        int32_t dec = 0;
        if (enc < 0 || !toy_tz_ram_decode((uint8_t)enc, &dec) || dec != v)
            bad++;
    }
    CHECK(bad == 0, "whole 15-min tz domain round-trips the RAM byte");
}

/* --- touch mapping ---------------------------------------------- */

static void test_touch_mapping(void)
{
    /* Mirror ON (default): controller 0 -> glass 465 -> milli
     * (465-233)*1000/233 = 995.7 -> 996; controller 465 ->
     * glass 0 -> -1000; controller 233 -> glass 232 ->
     * (232-233)*1000/233 = -4. */
    CHECK(toy_prod_touch_axis(0) == 465, "mirror: ctrl 0 -> glass 465");
    CHECK(toy_prod_touch_axis(465) == 0, "mirror: ctrl 465 -> glass 0");
    CHECK(toy_prod_touch_axis(233) == 232, "mirror: ctrl 233 -> 232");
    CHECK(toy_prod_touch_axis(999) == 0, "stray 999 clamps then mirrors");
    CHECK(toy_prod_touch_axis(-5) == 465, "negative clamps then mirrors");
    toy_event_t ev;
    toy_prod_pointer_event(&ev, TOY_EV_POINTER_DOWN, 0, 0, 465);
    CHECK(ev.type == TOY_EV_POINTER_DOWN && ev.id == 0,
          "pointer event type/id");
    CHECK(ev.x == 996 && ev.y == -1000,
          "mirrored corner milli (996,-1000)");
    toy_prod_pointer_event(&ev, TOY_EV_POINTER_UP, 1, 233, 233);
    CHECK(ev.x == -4 && ev.y == -4, "centre-ish milli (-4,-4), id kept");
    CHECK(ev.id == 1, "contact id 1 carried");
}

/* --- gestures --------------------------------------------------- */

static size_t report(toy_prod_gesture_t *g, uint64_t ms, bool running,
                     bool p0, int32_t x0, int32_t y0,
                     bool p1, int32_t x1, int32_t y1,
                     toy_event_t *out)
{
    bool present[2] = { p0, p1 };
    int32_t px[2] = { x0, x1 };
    int32_t py[2] = { y0, y1 };
    return toy_prod_gesture_report(g, ms, running, present, px, py,
                                   out, 6);
}

static void test_gesture_basic(void)
{
    toy_prod_gesture_t g;
    toy_prod_gesture_init(&g);
    toy_event_t out[6];
    size_t n = report(&g, 100, true, true, 10, 20, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_DOWN && out[0].id == 0,
          "first report = DOWN id0");
    n = report(&g, 120, true, true, 10, 20, false, 0, 0, out);
    CHECK(n == 0, "same position = no MOVE");
    n = report(&g, 140, true, true, 30, 20, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_MOVE, "moved = MOVE");
    n = report(&g, 200, true, false, 0, 0, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_UP,
          "vanished = UP at last position");
    /* Tracks are independent: id1 alone works. */
    n = report(&g, 300, false, false, 0, 0, true, 100, 100, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_DOWN && out[0].id == 1,
          "lone id1 = DOWN id1");
    n = report(&g, 400, false, false, 0, 0, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_UP && out[0].id == 1,
          "id1 lift = UP id1");
}

static void test_gesture_long_press(void)
{
    toy_prod_gesture_t g;
    toy_prod_gesture_init(&g);
    toy_event_t out[6];
    (void)report(&g, 1000, true, true, 50, 50, false, 0, 0, out);
    CHECK(toy_prod_gesture_poll(&g, 1999, out, 6) == 0,
          "999 ms: no ESC yet");
    size_t n = toy_prod_gesture_poll(&g, 2000, out, 6);
    CHECK(n == 1 && out[0].type == TOY_EV_KEY_DOWN &&
              out[0].id == TOY_KEY_ESC,
          "1000 ms stationary: KEY_ESC exactly");
    CHECK(toy_prod_gesture_poll(&g, 2300, out, 6) == 0,
          "ESC fires once per press");
    n = report(&g, 2400, true, true, 90, 90, false, 0, 0, out);
    CHECK(n == 0, "consumed contact: MOVE suppressed");
    n = report(&g, 2500, true, false, 0, 0, false, 0, 0, out);
    CHECK(n == 0, "consumed contact: UP suppressed (no leak)");
    /* A fresh press after lift behaves normally again. */
    n = report(&g, 3000, true, true, 50, 50, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_DOWN,
          "next press is a clean DOWN");
    /* Short press never fires. */
    CHECK(toy_prod_gesture_poll(&g, 3700, out, 6) == 0, "700 ms: quiet");
    /* Two contacts held: no long-press for either. */
    (void)report(&g, 3750, false, true, 50, 50, true, 60, 60, out);
    CHECK(toy_prod_gesture_poll(&g, 5000, out, 6) == 0,
          "two contacts: long-press suppressed");
}

static void test_gesture_long_press_drag(void)
{
    /* The live defect: a press-hold-drag (a cart's core verb)
     * must never exit. Moving 20 px from the DOWN point at
     * 500 ms cancels the timer; the forwarded stream is
     * otherwise exactly DOWN/MOVE/.../UP. */
    toy_prod_gesture_t g;
    toy_prod_gesture_init(&g);
    toy_event_t out[6];
    size_t n = report(&g, 0, true, true, 100, 100, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_DOWN,
          "drag: DOWN forwarded");
    n = report(&g, 500, true, true, 120, 100, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_MOVE,
          "drag: 20 px MOVE forwarded (and cancels long-press)");
    CHECK(toy_prod_gesture_poll(&g, 1000, out, 6) == 0,
          "drag: no ESC at 1000 ms");
    CHECK(toy_prod_gesture_poll(&g, 2000, out, 6) == 0,
          "drag: no ESC at 2000 ms either");
    n = report(&g, 2100, true, true, 160, 130, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_MOVE,
          "drag: later MOVEs still forwarded (not consumed)");
    n = report(&g, 2200, true, false, 0, 0, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_UP,
          "drag: UP forwarded (not consumed)");

    /* Jitter inside the 12 px tolerance (incl. exactly 12 px:
     * cancellation needs MORE than the tolerance) still fires. */
    toy_prod_gesture_init(&g);
    (void)report(&g, 0, true, true, 100, 100, false, 0, 0, out);
    n = report(&g, 400, true, true, 112, 100, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_MOVE,
          "jitter: 12 px MOVE forwarded");
    (void)report(&g, 700, true, true, 108, 108, false, 0, 0, out);
    CHECK(toy_prod_gesture_poll(&g, 999, out, 6) == 0,
          "jitter: quiet at 999 ms");
    n = toy_prod_gesture_poll(&g, 1000, out, 6);
    CHECK(n == 1 && out[0].type == TOY_EV_KEY_DOWN &&
              out[0].id == TOY_KEY_ESC,
          "jitter within tolerance: ESC at 1000 ms");
    n = report(&g, 1100, true, false, 0, 0, false, 0, 0, out);
    CHECK(n == 0, "jitter: post-ESC UP suppressed (consumed)");

    /* Drag out and back to the start point, then hold: the
     * cancel is permanent for this contact. A fresh press
     * after lift re-arms normally. */
    toy_prod_gesture_init(&g);
    (void)report(&g, 0, true, true, 100, 100, false, 0, 0, out);
    (void)report(&g, 300, true, true, 120, 100, false, 0, 0, out);
    n = report(&g, 600, true, true, 100, 100, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_MOVE,
          "drag-return: MOVE back to origin forwarded");
    CHECK(toy_prod_gesture_poll(&g, 1000, out, 6) == 0,
          "drag-return: no ESC at 1000 ms (cancel is permanent)");
    CHECK(toy_prod_gesture_poll(&g, 5000, out, 6) == 0,
          "drag-return: still no ESC at 5000 ms");
    (void)report(&g, 5100, true, false, 0, 0, false, 0, 0, out);
    (void)report(&g, 6000, true, true, 100, 100, false, 0, 0, out);
    n = toy_prod_gesture_poll(&g, 7000, out, 6);
    CHECK(n == 1 && out[0].id == TOY_KEY_ESC,
          "fresh stationary press after a cancelled drag re-arms");
}

static void test_gesture_two_finger(void)
{
    toy_prod_gesture_t g;
    toy_prod_gesture_init(&g);
    toy_event_t out[6];
    /* RUNNING: second finger at +300 ms -> KEY_X, no DOWN1. */
    (void)report(&g, 0, true, true, 10, 10, false, 0, 0, out);
    size_t n = report(&g, 300, true, true, 10, 10, true, 200, 200, out);
    CHECK(n == 1 && out[0].type == TOY_EV_KEY_DOWN &&
              out[0].id == TOY_KEY_X,
          "second finger at 300 ms = KEY_X (running)");
    n = report(&g, 400, true, true, 30, 10, true, 220, 200, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_MOVE && out[0].id == 0,
          "finger 0 still moves; consumed finger 1 silent");
    n = report(&g, 500, true, false, 0, 0, false, 0, 0, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_UP && out[0].id == 0,
          "only finger 0 releases visibly");
    /* RUNNING: second finger at +301 ms is an ordinary DOWN. */
    toy_prod_gesture_init(&g);
    (void)report(&g, 0, true, true, 10, 10, false, 0, 0, out);
    n = report(&g, 301, true, true, 10, 10, true, 200, 200, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_DOWN && out[0].id == 1,
          "301 ms: ordinary DOWN id1");
    /* Not running: even an instant second finger is a DOWN. */
    toy_prod_gesture_init(&g);
    (void)report(&g, 0, false, true, 10, 10, false, 0, 0, out);
    n = report(&g, 50, false, true, 10, 10, true, 200, 200, out);
    CHECK(n == 1 && out[0].type == TOY_EV_POINTER_DOWN,
          "shell screens: two-finger synthesis off");
    /* Simultaneous pair from idle (both in one report): the
     * low id is processed first, so id1 is "second" at dt 0. */
    toy_prod_gesture_init(&g);
    n = report(&g, 100, true, true, 10, 10, true, 200, 200, out);
    CHECK(n == 2 && out[0].type == TOY_EV_POINTER_DOWN &&
              out[1].type == TOY_EV_KEY_DOWN && out[1].id == TOY_KEY_X,
          "same-report pair: DOWN then KEY_X");
}

/* --- tilt + dial + calibration ---------------------------------- */

static void test_tilt_gate(void)
{
    toy_prod_tilt_gate_t t;
    toy_prod_tilt_gate_init(&t);
    toy_event_t ev;
    CHECK(toy_prod_tilt_gate_poll(&t, 0, 0, 0, &ev) &&
              ev.type == TOY_EV_TILT && ev.x == 0 && ev.y == 0,
          "first sample always sends (flat = 0,0)");
    CHECK(!toy_prod_tilt_gate_poll(&t, 10, -500, 0, &ev),
          "10 ms later: rate gate blocks");
    CHECK(!toy_prod_tilt_gate_poll(&t, 33, -19, 0, &ev),
          "33 ms but only 19 mg: change gate blocks");
    /* Default signs fold anti-gravity: ax=-20 -> ev.x=+20. */
    CHECK(toy_prod_tilt_gate_poll(&t, 40, -20, 0, &ev) && ev.x == 20 &&
              ev.y == 0,
          "20 mg on x sends, sign folded to fall-direction");
    CHECK(!toy_prod_tilt_gate_poll(&t, 80, -39, 19, &ev),
          "delta measured from last SENT (19,19): blocked");
    CHECK(toy_prod_tilt_gate_poll(&t, 90, -40, 0, &ev) && ev.x == 40,
          "20 mg from last sent sends");
    CHECK(toy_prod_tilt_gate_poll(&t, 200, 9000, -9000, &ev) &&
              ev.x == -2000 && ev.y == 2000,
          "wild accel clamps to +/-2000 mg");
}

static void test_dial(void)
{
    toy_prod_dial_t d;
    toy_prod_dial_init(&d, 0);
    toy_event_t ev;
    /* 360 dps = 360000 mdps for 25 ms = 9 deg = 25 mt: the
     * exact first emission threshold. */
    CHECK(!toy_prod_dial_feed(&d, 360000, 24, &ev), "24 ms: 24 mt owed, held");
    CHECK(toy_prod_dial_feed(&d, 360000, 1, &ev) &&
              ev.type == TOY_EV_DIAL && ev.id == 0 && ev.v == 25,
          "25th ms emits exactly +25 milliturns");
    /* Remainder kept: another 25 ms emits another +25, not 24. */
    CHECK(toy_prod_dial_feed(&d, 360000, 25, &ev) && ev.v == 25,
          "steady spin emits steady +25s");
    /* Slow drift below threshold accumulates across calls. */
    toy_prod_dial_init(&d, 0);
    int emitted = 0;
    for (int i = 0; i < 24; i++)
        if (toy_prod_dial_feed(&d, 360000, 1, &ev))
            emitted++;
    CHECK(emitted == 0, "1 ms slices below threshold emit nothing");
    CHECK(toy_prod_dial_feed(&d, 360000, 1, &ev) && ev.v == 25,
          "slices accumulate to the same +25");
    /* Negative wrap: -360 dps for 30 ms = -30 mt in one event
     * (exact, no remainder), then a fresh -25 needs a full
     * 25 ms more. Fractional remainder: -180 dps slices owe
     * 0.5 mt each and must pair up inside the accumulator. */
    toy_prod_dial_init(&d, 0);
    CHECK(toy_prod_dial_feed(&d, -360000, 30, &ev) && ev.v == -30,
          "negative spin emits -30 at once");
    CHECK(!toy_prod_dial_feed(&d, -360000, 24, &ev),
          "-24 into the next event: held");
    CHECK(toy_prod_dial_feed(&d, -360000, 1, &ev) && ev.v == -25,
          "25th ms emits -25");
    toy_prod_dial_init(&d, 0);
    CHECK(!toy_prod_dial_feed(&d, -180000, 49, &ev),
          "49 half-milliturn ms = -24.5 mt: held whole");
    CHECK(toy_prod_dial_feed(&d, -180000, 1, &ev) && ev.v == -25,
          "fractional remainders sum to -25 exactly");
    /* Reversal cancels inside the accumulator. */
    toy_prod_dial_init(&d, 0);
    (void)toy_prod_dial_feed(&d, 360000, 20, &ev);
    CHECK(!toy_prod_dial_feed(&d, -360000, 20, &ev),
          "equal reversal nets to zero, nothing emitted");
    /* Bias subtraction: still desk at exactly the bias emits
     * nothing forever; bias+360 dps behaves like 360 dps. */
    toy_prod_dial_init(&d, 1500);
    emitted = 0;
    for (int i = 0; i < 100; i++)
        if (toy_prod_dial_feed(&d, 1500, 100, &ev))
            emitted++;
    CHECK(emitted == 0, "reading == bias integrates to silence");
    CHECK(toy_prod_dial_feed(&d, 361500, 25, &ev) && ev.v == 25,
          "rate above bias integrates net");
    /* Zero dt is a no-op even at full rate. */
    toy_prod_dial_init(&d, 0);
    CHECK(!toy_prod_dial_feed(&d, 360000, 0, &ev), "dt=0 integrates nothing");
}

static void test_calibration(void)
{
    toy_prod_cal_t c;
    int32_t bias = -99999;
    toy_prod_cal_init(&c);
    for (int i = 0; i < 50; i++)
        toy_prod_cal_add(&c, 12, -30, -998, 1400 + (i % 5) * 40 - 80);
    CHECK(toy_prod_cal_finish(&c, &bias) && bias == 1400,
          "still desk (az=-998): bias = mean gz 1400");
    /* Not at 1 g (held/shaken): rejected. */
    toy_prod_cal_init(&c);
    for (int i = 0; i < 50; i++)
        toy_prod_cal_add(&c, 12, -30, -700, 1400);
    CHECK(!toy_prod_cal_finish(&c, &bias), "|a| ~700 mg rejects");
    /* Rotating: wide gz spread rejects. */
    toy_prod_cal_init(&c);
    for (int i = 0; i < 50; i++)
        toy_prod_cal_add(&c, 0, 0, -1000, (i % 2) ? 9000 : -9000);
    CHECK(!toy_prod_cal_finish(&c, &bias), "gz spread rejects");
    /* Steady fast spin looks like bias but implausible: rejected. */
    toy_prod_cal_init(&c);
    for (int i = 0; i < 50; i++)
        toy_prod_cal_add(&c, 0, 0, -1000, 20000);
    CHECK(!toy_prod_cal_finish(&c, &bias), "20 dps mean is not a bias");
    /* Too few samples: rejected. */
    toy_prod_cal_init(&c);
    for (int i = 0; i < 10; i++)
        toy_prod_cal_add(&c, 0, 0, -1000, 100);
    CHECK(!toy_prod_cal_finish(&c, &bias), "10 samples < minimum rejects");
    /* Negative bias fine. */
    toy_prod_cal_init(&c);
    for (int i = 0; i < 40; i++)
        toy_prod_cal_add(&c, -5, 8, -1001, -625);
    CHECK(toy_prod_cal_finish(&c, &bias) && bias == -625,
          "negative bias -625 mdps accepted");
    /* One bad sample poisons the set. */
    toy_prod_cal_init(&c);
    for (int i = 0; i < 40; i++)
        toy_prod_cal_add(&c, 0, 0, i == 20 ? -300 : -1000, 100);
    CHECK(!toy_prod_cal_finish(&c, &bias), "single jolt rejects all");
}

/* --- IRQ pending merge + gallery tap-launch (touchfix2) -------- */

static toy_event_t ptr_ev(uint32_t type, uint32_t id, int32_t x, int32_t y)
{
    toy_event_t e = { 0 };
    e.type = type;
    e.id = id;
    e.x = x;
    e.y = y;
    return e;
}

static void test_touch_pending_merge(void)
{
    CHECK(!toy_prod_touch_pending_merge(false, false),
          "merge: no latch, INT high = not pending");
    CHECK(toy_prod_touch_pending_merge(true, false),
          "merge: IRQ latch alone is pending (pulse already rose)");
    CHECK(toy_prod_touch_pending_merge(false, true),
          "merge: INT low now is pending");
    CHECK(toy_prod_touch_pending_merge(true, true),
          "merge: both is pending");
}

static void test_gallery_row_math(void)
{
    /* Same constants as core/toy_shell.c ML_G_ROW_Y=-620,
     * ML_G_ROW_H=155, 6 visible rows -> band [-620, 310). */
    CHECK(toy_prod_gallery_row_at(-620, 0, 25) == 0, "row band top = row 0");
    CHECK(toy_prod_gallery_row_at(-466, 0, 25) == 0, "y=-466 still row 0");
    CHECK(toy_prod_gallery_row_at(-465, 0, 25) == 1, "y=-465 = row 1");
    CHECK(toy_prod_gallery_row_at(309, 0, 25) == 5, "y=309 = last visible row");
    CHECK(toy_prod_gallery_row_at(310, 0, 25) == -1,
          "y=310 = detail gap, no row");
    CHECK(toy_prod_gallery_row_at(-621, 0, 25) == -1, "above band: no row");
    CHECK(toy_prod_gallery_row_at(740, 0, 25) == -1,
          "footer y>=740: no row (never tap-launched)");
    CHECK(toy_prod_gallery_row_at(900, 0, 25) == -1, "deep footer: no row");
    CHECK(toy_prod_gallery_row_at(-620, 7, 25) == 7, "scroll offsets the row");
    CHECK(toy_prod_gallery_row_at(309, 20, 25) == -1,
          "row past total rejected");
}

static void test_tap_launch(void)
{
    toy_prod_tap_launch_t t;
    toy_event_t synth;
    /* Gallery: 23 carts -> total 25 rows; y=-400 is row 1
     * ((-400+620)/155 = 1) at scroll 0. */
    toy_event_t down = ptr_ev(TOY_EV_POINTER_DOWN, 0, 0, -400);
    toy_event_t up = ptr_ev(TOY_EV_POINTER_UP, 0, 10, -390);

    toy_prod_tap_launch_init(&t);
    CHECK(toy_prod_tap_launch_note_down(&t, 1000, true, false, 0, 23, 1,
                                        &down),
          "tap: selecting DOWN on cart row arms");
    CHECK(toy_prod_tap_launch_take_up(&t, 1200, true, 0, 23, 1, &up,
                                      &synth) &&
              synth.type == TOY_EV_POINTER_DOWN && synth.id == 0 &&
              synth.x == 0 && synth.y == -400,
          "tap: short still UP synthesises DOWN at DOWN coords");
    CHECK(!toy_prod_tap_launch_take_up(&t, 1250, true, 0, 23, 1, &up,
                                       &synth),
          "tap: synth fires at most once per tap");

    /* Already launched by the real DOWN: never arm. */
    toy_prod_tap_launch_init(&t);
    CHECK(!toy_prod_tap_launch_note_down(&t, 0, true, true, 0, 23, 1, &down),
          "tap: START_CART DOWN does not arm");
    /* Not gallery (SETTINGS/CLOCKSET/RUNNING posture). */
    CHECK(!toy_prod_tap_launch_note_down(&t, 0, false, false, 0, 23, 1,
                                         &down),
          "tap: non-gallery DOWN does not arm");
    /* Footer tap (RUN/SET zone). */
    toy_event_t foot = ptr_ev(TOY_EV_POINTER_DOWN, 0, -300, 800);
    CHECK(!toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1,
                                         &foot),
          "tap: footer DOWN does not arm");
    /* Virtual rows: row 23 = SET CLOCK, row 24 = CLOCK FACE
     * (visible when scrolled: y=-620 with scroll 23/24). */
    toy_event_t vdown = ptr_ev(TOY_EV_POINTER_DOWN, 0, 0, -620);
    CHECK(!toy_prod_tap_launch_note_down(&t, 0, true, false, 23, 23, 23,
                                         &vdown),
          "tap: SET CLOCK row does not arm");
    CHECK(!toy_prod_tap_launch_note_down(&t, 0, true, false, 24, 23, 24,
                                         &vdown),
          "tap: CLOCK FACE row does not arm");
    /* DOWN that did not select this row (selection elsewhere). */
    CHECK(!toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 5,
                                         &down),
          "tap: row != selection after DOWN does not arm");

    /* Slow press: 501 ms is not a tap; exactly 500 ms is. */
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    CHECK(!toy_prod_tap_launch_take_up(&t, 501, true, 0, 23, 1, &up, &synth),
          "tap: 501 ms press does not launch");
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    CHECK(toy_prod_tap_launch_take_up(&t, 500, true, 0, 23, 1, &up, &synth),
          "tap: 500 ms press still launches");

    /* Drag: a MOVE 100 milli away disarms; UP far away too. */
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    toy_event_t drag = ptr_ev(TOY_EV_POINTER_MOVE, 0, 100, -400);
    toy_prod_tap_launch_note_move(&t, &drag);
    toy_event_t up_home = ptr_ev(TOY_EV_POINTER_UP, 0, 0, -400);
    CHECK(!toy_prod_tap_launch_take_up(&t, 100, true, 0, 23, 1, &up_home,
                                       &synth),
          "tap: drag away and back does not launch");
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    toy_event_t up_far = ptr_ev(TOY_EV_POINTER_UP, 0, 0, -100);
    CHECK(!toy_prod_tap_launch_take_up(&t, 100, true, 0, 23, 1, &up_far,
                                       &synth),
          "tap: UP in another row does not launch");

    /* Screen or selection moved on before the UP. */
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    CHECK(!toy_prod_tap_launch_take_up(&t, 100, false, 0, 23, 1, &up, &synth),
          "tap: UP outside gallery does not launch");
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    CHECK(!toy_prod_tap_launch_take_up(&t, 100, true, 0, 23, 3, &up, &synth),
          "tap: selection moved before UP does not launch");
    /* Wrong contact's UP cannot spend the tap. */
    toy_prod_tap_launch_init(&t);
    (void)toy_prod_tap_launch_note_down(&t, 0, true, false, 0, 23, 1, &down);
    toy_event_t up1 = ptr_ev(TOY_EV_POINTER_UP, 1, 0, -400);
    CHECK(!toy_prod_tap_launch_take_up(&t, 100, true, 0, 23, 1, &up1, &synth),
          "tap: other finger's UP does not launch");
    CHECK(toy_prod_tap_launch_take_up(&t, 110, true, 0, 23, 1, &up, &synth),
          "tap: own UP still launches after other finger lifted");
}

int main(void)
{
    printf("clock civil <-> epoch\n");
    test_epoch_anchors();
    test_leap_years();
    test_round_trips();
    test_commit_and_tz();
    printf("touch mapping\n");
    test_touch_mapping();
    printf("gestures (fake clock)\n");
    test_gesture_basic();
    test_gesture_long_press();
    test_gesture_long_press_drag();
    test_gesture_two_finger();
    printf("tilt gate / dial / calibration\n");
    test_tilt_gate();
    test_dial();
    test_calibration();
    printf("irq merge / gallery tap-launch\n");
    test_touch_pending_merge();
    test_gallery_row_math();
    test_tap_launch();
    if (failures == 0) {
        printf("product_input_test: PASS (%d checks)\n", checks);
        return 0;
    }
    printf("product_input_test: FAIL (%d/%d checks failed)\n",
           failures, checks);
    return 1;
}
