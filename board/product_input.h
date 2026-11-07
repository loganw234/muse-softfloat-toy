/* product_input.h - pure live-input seams for the product
 * shell runner (toy5-shell): CST9217 report -> pointer events
 * + runner gestures, QMI8658 accel -> TILT, gyro -> DIAL.
 *
 * Host-compilable and SDK-free like the sensor drivers: every
 * function here is a pure integer transform over caller state
 * and a caller-supplied millisecond clock, so
 * tools/product_input_test.c drives the whole seam with a
 * fake clock. main_product.c owns the I2C polling and feeds
 * decoded points/samples in; nothing here touches hardware,
 * the shell, or a cartridge.
 *
 * Event vocabulary is core/toy_input.h verbatim (milli-radius
 * pointer coords via toy_pointer_event, milli-g TILT, signed
 * milliturn DIAL, KEY_ESC/KEY_X for the shell gestures per
 * core/toy_shell.h). Produced events still go to
 * toy_shell_event FIRST in the runner; carts only ever see
 * what the shell forwards, at step boundaries.
 */
#ifndef TOY_PRODUCT_INPUT_H
#define TOY_PRODUCT_INPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "toy_input.h" /* core: toy_event_t, toy_pointer_event */

/* --- touch coordinate mapping ---------------------------------
 *
 * The CST9217 reports controller space 0..465 per axis on the
 * 466x466 glass (board/touch_cst9217.h). Waveshare's driver
 * for this exact board mirrors BOTH axes for display
 * (466 - v, i.e. 465 - v on a 0-based axis; board/SENSORS.md).
 * That mirror is vendor-stated and WAS live-proven only as
 * far as coordinates existing in 0..465 - the drag-direction
 * check is still owed by the owner on the first shell flash
 * (board/PRODUCT.md script). If the glass turns out mirrored
 * the other way, TOY_PROD_TOUCH_MIRROR is the one define to
 * flip (0 = controller space is display space).
 */
#ifndef TOY_PROD_TOUCH_MIRROR
#define TOY_PROD_TOUCH_MIRROR 1
#endif

#define TOY_PROD_GLASS_PX 466 /* CO5300 glass, px per axis */
#define TOY_PROD_COORD_MAX_PX 465

/* One controller axis -> display-glass axis, clamped to the
 * panel (the parser does not clamp; a stray 466+ must not
 * mirror-wrap into a wild negative milli value). */
static inline int32_t toy_prod_touch_axis(int32_t controller_v)
{
    if (controller_v < 0)
        controller_v = 0;
    if (controller_v > TOY_PROD_COORD_MAX_PX)
        controller_v = TOY_PROD_COORD_MAX_PX;
#if TOY_PROD_TOUCH_MIRROR
    return TOY_PROD_COORD_MAX_PX - controller_v;
#else
    return controller_v;
#endif
}

/* Controller point -> pointer event in toy milli coords
 * (origin glass centre, unit 1/1000 radius, +y down - the
 * toy_px_to_milli convention every cart already consumes). */
static inline void toy_prod_pointer_event(toy_event_t *ev, uint32_t type,
                                          uint32_t contact_id,
                                          int32_t controller_x,
                                          int32_t controller_y)
{
    toy_pointer_event(ev, type, contact_id,
                      toy_prod_touch_axis(controller_x),
                      toy_prod_touch_axis(controller_y),
                      TOY_PROD_GLASS_PX);
}

static inline toy_event_t toy_prod_key_event(uint32_t key)
{
    toy_event_t e = { 0 };
    e.type = TOY_EV_KEY_DOWN;
    e.id = key;
    return e;
}

/* --- gesture tracker (CST9217 has 2 tracks) -------------------
 *
 * Derivation mirrors main_sensor_test.c phase 5 (a track
 * appearing = DOWN, moving = MOVE, vanishing = UP - the chip
 * has no up-code), plus the two runner-synthesised shell
 * gestures from toy_shell.h:
 *
 *   LONG-PRESS: one contact held STILL >=
 *   TOY_PROD_LONG_PRESS_MS while it is the ONLY active
 *   contact synthesises KEY_ESC once (exit when RUNNING;
 *   back on shell screens - the shell gives KEY_ESC those
 *   meanings itself). "Still" is load-bearing: if the
 *   contact ever strays more than TOY_PROD_LONG_PRESS_TOL_PX
 *   from its DOWN point, the timer is cancelled for that
 *   contact permanently (a drag is a cart verb, never an
 *   exit; returning to the start point does not re-arm -
 *   only a fresh DOWN does). Cancellation suppresses only
 *   the synthesised ESC; DOWN/MOVE/UP forward unchanged.
 *   The press's
 *   DOWN was already delivered (and forwarded to the cart if
 *   RUNNING) before the timing could be known; toy_shell.h
 *   acknowledges exactly this pattern. After the ESC the
 *   contact is CONSUMED: its MOVE/UP are suppressed until it
 *   lifts, so the exit gesture cannot leak a trailing
 *   release into whatever screen comes next.
 *
 *   TWO-FINGER TAP: a second contact going down within
 *   TOY_PROD_TWO_FINGER_MS of the first, while the first is
 *   still held, synthesises KEY_X (settings) - but only when
 *   the runner passes running=true. The second finger's own
 *   DOWN is replaced by the KEY_X and that finger is consumed
 *   until it lifts; the first finger stays a normal pointer.
 *   On shell screens (running=false) the second finger is an
 *   ordinary DOWN and the shell's own pointer handling owns
 *   it, untimed.
 *
 * Call toy_prod_gesture_report() on each decoded report and
 * toy_prod_gesture_poll() every runner frame (long-press is a
 * timer, not a report). Both append into a caller array;
 * 5 slots always suffice (report: 2 UPs + KEY_X + ... worst
 * case is DOWN+MOVE mixes bounded by 2 tracks + 1 key; poll:
 * 1 key). now_ms is monotonic (toy_clock_mono_ms domain);
 * a backwards clock simply delays the long-press.
 */
#define TOY_PROD_MAX_CONTACTS 2u
#define TOY_PROD_LONG_PRESS_MS 1000u
#define TOY_PROD_LONG_PRESS_TOL_PX 12 /* glass px from DOWN point */
#define TOY_PROD_TWO_FINGER_MS 300u

typedef struct {
    bool active;      /* track present in the latest report */
    bool consumed;    /* gesture claimed it: suppress to lift */
    bool long_fired;  /* KEY_ESC already sent for this press */
    bool long_cancelled; /* moved past tolerance: no ESC this press */
    uint64_t down_ms; /* when this contact went down */
    int32_t down_cx, down_cy; /* controller-space DOWN point */
    int32_t cx, cy;   /* last controller-space position */
} toy_prod_contact_t;

typedef struct {
    toy_prod_contact_t contact[TOY_PROD_MAX_CONTACTS];
    bool pair_fired;    /* KEY_X sent for the current overlap */
    uint64_t first_down_ms; /* down time of the earlier contact */
    int first_id;       /* which contact went down first, -1 none */
} toy_prod_gesture_t;

static inline void toy_prod_gesture_init(toy_prod_gesture_t *g)
{
    for (unsigned i = 0; i < TOY_PROD_MAX_CONTACTS; i++) {
        g->contact[i].active = false;
        g->contact[i].consumed = false;
        g->contact[i].long_fired = false;
        g->contact[i].long_cancelled = false;
        g->contact[i].down_ms = 0;
        g->contact[i].down_cx = 0;
        g->contact[i].down_cy = 0;
        g->contact[i].cx = 0;
        g->contact[i].cy = 0;
    }
    g->pair_fired = false;
    g->first_down_ms = 0;
    g->first_id = -1;
}

static inline bool toy_prod_gesture_any_active(const toy_prod_gesture_t *g)
{
    return g->contact[0].active || g->contact[1].active;
}

/* present[id]/px/py describe the new report in controller
 * space. Returns the number of events written to out[]. */
static inline size_t toy_prod_gesture_report(
    toy_prod_gesture_t *g, uint64_t now_ms, bool running,
    const bool present[TOY_PROD_MAX_CONTACTS],
    const int32_t px[TOY_PROD_MAX_CONTACTS],
    const int32_t py[TOY_PROD_MAX_CONTACTS],
    toy_event_t *out, size_t out_cap)
{
    size_t n = 0;
    /* Releases first (sensor-test order: a vanished track's
     * UP precedes any new DOWN in the same frame). */
    for (unsigned id = 0; id < TOY_PROD_MAX_CONTACTS; id++) {
        toy_prod_contact_t *c = &g->contact[id];
        if (c->active && !present[id]) {
            if (!c->consumed && n < out_cap)
                toy_prod_pointer_event(&out[n++], TOY_EV_POINTER_UP,
                                       id, c->cx, c->cy);
            c->active = false;
            c->consumed = false;
            c->long_fired = false;
            c->long_cancelled = false;
        }
    }
    if (!toy_prod_gesture_any_active(g)) {
        g->pair_fired = false;
        g->first_id = -1;
    }
    for (unsigned id = 0; id < TOY_PROD_MAX_CONTACTS; id++) {
        toy_prod_contact_t *c = &g->contact[id];
        if (!present[id])
            continue;
        if (!c->active) {
            /* New contact. Is it the second finger of a tap? */
            unsigned other = id ^ 1u;
            bool is_pair = running && !g->pair_fired &&
                           g->contact[other].active &&
                           !g->contact[other].consumed &&
                           g->first_id == (int)other &&
                           now_ms >= g->first_down_ms &&
                           now_ms - g->first_down_ms <=
                               TOY_PROD_TWO_FINGER_MS;
            c->active = true;
            c->long_fired = false;
            c->long_cancelled = false;
            c->down_ms = now_ms;
            c->down_cx = px[id];
            c->down_cy = py[id];
            c->cx = px[id];
            c->cy = py[id];
            if (is_pair) {
                c->consumed = true;
                g->pair_fired = true;
                if (n < out_cap)
                    out[n++] = toy_prod_key_event(TOY_KEY_X);
            } else {
                c->consumed = false;
                if (g->first_id < 0) {
                    g->first_id = (int)id;
                    g->first_down_ms = now_ms;
                }
                if (n < out_cap)
                    toy_prod_pointer_event(&out[n++],
                                           TOY_EV_POINTER_DOWN, id,
                                           c->cx, c->cy);
            }
        } else {
            /* Straying past tolerance from the DOWN point makes
             * this contact a drag: cancel its long-press for
             * good (re-arming needs a fresh DOWN). The event
             * stream below is untouched by the cancellation. */
            int64_t dx = (int64_t)px[id] - c->down_cx;
            int64_t dy = (int64_t)py[id] - c->down_cy;
            if (dx * dx + dy * dy >
                (int64_t)TOY_PROD_LONG_PRESS_TOL_PX *
                    TOY_PROD_LONG_PRESS_TOL_PX)
                c->long_cancelled = true;
            if (!c->consumed &&
                (px[id] != c->cx || py[id] != c->cy)) {
                c->cx = px[id];
                c->cy = py[id];
                if (n < out_cap)
                    toy_prod_pointer_event(&out[n++],
                                           TOY_EV_POINTER_MOVE, id,
                                           c->cx, c->cy);
            } else {
                c->cx = px[id];
                c->cy = py[id];
            }
        }
    }
    return n;
}

/* Frame-cadence long-press timer. Writes at most one KEY_ESC. */
static inline size_t toy_prod_gesture_poll(toy_prod_gesture_t *g,
                                           uint64_t now_ms,
                                           toy_event_t *out,
                                           size_t out_cap)
{
    unsigned active = (g->contact[0].active ? 1u : 0u) +
                      (g->contact[1].active ? 1u : 0u);
    if (active != 1u || out_cap == 0u)
        return 0;
    unsigned id = g->contact[0].active ? 0u : 1u;
    toy_prod_contact_t *c = &g->contact[id];
    if (c->consumed || c->long_fired || c->long_cancelled)
        return 0;
    if (now_ms < c->down_ms ||
        now_ms - c->down_ms < TOY_PROD_LONG_PRESS_MS)
        return 0;
    c->long_fired = true;
    c->consumed = true;
    out[0] = toy_prod_key_event(TOY_KEY_ESC);
    return 1;
}

/* --- tilt ------------------------------------------------------
 *
 * TILT event convention (cart_gravity.c/cart_pond.c headers):
 * x/y are milli-g in DISPLAY space (+x right, +y down); the
 * carts flip y to sim-up internally, exactly once, so the
 * runner must NOT flip it again. (300, 400) in their proof
 * scripts tips the world toward the display lower-right.
 *
 * Sensor frame facts (hardware-proven): lying flat screen-up
 * the QMI8658 reads az ~ -1000 mg dominant, i.e. the sensor
 * +Z axis points AWAY from the viewer through the back of
 * the board. The in-plane axis alignment was NOT pinned by
 * the sensors parcel (SENSORS.md: "measured on the first
 * board run, not assumed" - the run pinned az only), so the
 * mapping below takes ax -> display +x, ay -> display +y as
 * the starting convention with one sign define per axis; the
 * first-flash script (PRODUCT.md) checks the gravity cart's
 * tilt spoke against a known tip and a flip is one define.
 *
 * Physical reading: at rest the accelerometer reports the
 * support force, so tipping the right edge DOWN moves the
 * gravity vector toward display +x in proper-acceleration
 * terms as -x... no: rest az=-1000 screen-up means the
 * reported vector points opposite gravity (up, out of the
 * screen reads negative on an into-screen axis). Carts treat
 * TILT (x,y) as "the direction things fall" in display
 * space. Falling direction in-plane = -(reported in-plane)
 * when the reported z is anti-gravity. The signs below fold
 * that in: ev = -(a) per axis under the ax/ay alignment
 * above, so tip-right-edge-down yields ev.x > 0 (fall toward
 * +right). Both the alignment and this fold are the two
 * documented unknowns the first flash resolves; each axis is
 * one define.
 */
#ifndef TOY_PROD_TILT_X_SIGN
#define TOY_PROD_TILT_X_SIGN (-1)
#endif
#ifndef TOY_PROD_TILT_Y_SIGN
#define TOY_PROD_TILT_Y_SIGN (-1)
#endif

#define TOY_PROD_TILT_MIN_MS 33u   /* at most one TILT per 33 ms */
#define TOY_PROD_TILT_MIN_MG 20    /* and only on a >=20 mg move */

static inline int32_t toy_prod_clamp_mg(int32_t v)
{
    if (v > 2000)
        return 2000;
    if (v < -2000)
        return -2000;
    return v;
}

static inline void toy_prod_tilt_event(toy_event_t *ev, int32_t ax_mg,
                                       int32_t ay_mg)
{
    toy_event_t e = { 0 };
    e.type = TOY_EV_TILT;
    e.x = toy_prod_clamp_mg(TOY_PROD_TILT_X_SIGN * ax_mg);
    e.y = toy_prod_clamp_mg(TOY_PROD_TILT_Y_SIGN * ay_mg);
    *ev = e;
}

typedef struct {
    bool have;         /* a TILT has been sent */
    int32_t last_x;    /* last sent event coords (milli-g) */
    int32_t last_y;
    uint64_t last_ms;  /* when it was sent */
} toy_prod_tilt_gate_t;

static inline void toy_prod_tilt_gate_init(toy_prod_tilt_gate_t *t)
{
    t->have = false;
    t->last_x = 0;
    t->last_y = 0;
    t->last_ms = 0;
}

static inline int32_t toy_prod_i32_abs(int32_t v)
{
    return v < 0 ? -v : v;
}

/* Decide + record: true (and *ev filled) iff this sample
 * should become a TILT now (>=33 ms since the last sent AND
 * some axis moved >=20 mg from the last SENT values; the
 * first sample always sends). */
static inline bool toy_prod_tilt_gate_poll(toy_prod_tilt_gate_t *t,
                                           uint64_t now_ms, int32_t ax_mg,
                                           int32_t ay_mg, toy_event_t *ev)
{
    toy_event_t cand;
    toy_prod_tilt_event(&cand, ax_mg, ay_mg);
    if (t->have) {
        if (now_ms < t->last_ms ||
            now_ms - t->last_ms < TOY_PROD_TILT_MIN_MS)
            return false;
        if (toy_prod_i32_abs(cand.x - t->last_x) < TOY_PROD_TILT_MIN_MG &&
            toy_prod_i32_abs(cand.y - t->last_y) < TOY_PROD_TILT_MIN_MG)
            return false;
    }
    t->have = true;
    t->last_x = cand.x;
    t->last_y = cand.y;
    t->last_ms = now_ms;
    *ev = cand;
    return true;
}

/* --- device-as-dial (gyro Z integration) -----------------------
 *
 * Screen-normal axis: az is the screen-normal accel axis
 * (flat screen-up az ~ -1000 mg), so gz is rotation about
 * the display normal - the device-as-dial axis. Sensor +Z
 * points away from the viewer (above), so positive gz
 * (right-hand about an away-pointing axis) is CLOCKWISE as
 * the viewer sees the glass. DIAL v > 0 scrolls the shell
 * selection down / increases values (toy_shell.c ev_detents),
 * and clockwise = increase is the natural dial reading, so
 * positive gz integrates to positive milliturns. One define
 * flips it if the first flash disagrees.
 *
 * Fixed-point integration: milliturns = mdps * dt_ms /
 * 360000 exactly (mdps*ms = 1e-6 deg; 1 milliturn = 0.36
 * deg). The accumulator keeps the raw mdps*ms numerator so
 * sub-milliturn remainders never leak; an event is emitted
 * once >= TOY_PROD_DIAL_MIN_MT milliturns are owed, carrying
 * the WHOLE owed delta (fast spins emit one big v, which
 * the shell's detent division and the DIAL carts' own
 * accumulators both handle), remainder kept.
 */
#ifndef TOY_PROD_DIAL_SIGN
#define TOY_PROD_DIAL_SIGN 1
#endif

#define TOY_PROD_DIAL_MIN_MT 25 /* milliturns per emitted event */
#define TOY_PROD_MDPS_MS_PER_MT 360000ll

typedef struct {
    int64_t acc_mdps_ms; /* un-emitted mdps*ms remainder */
    int32_t bias_mdps;   /* still-desk gyro Z bias, 0 if uncalibrated */
} toy_prod_dial_t;

static inline void toy_prod_dial_init(toy_prod_dial_t *d, int32_t bias_mdps)
{
    d->acc_mdps_ms = 0;
    d->bias_mdps = bias_mdps;
}

/* Feed one gyro-Z sample held for dt_ms. True (and *ev
 * filled, v = signed milliturn delta, |v| >= 25) when an
 * event is owed. dt_ms == 0 integrates nothing. */
static inline bool toy_prod_dial_feed(toy_prod_dial_t *d, int32_t gz_mdps,
                                      uint32_t dt_ms, toy_event_t *ev)
{
    int64_t rate = (int64_t)TOY_PROD_DIAL_SIGN *
                   ((int64_t)gz_mdps - (int64_t)d->bias_mdps);
    d->acc_mdps_ms += rate * (int64_t)dt_ms;
    int64_t mt = d->acc_mdps_ms / TOY_PROD_MDPS_MS_PER_MT;
    if (mt >= TOY_PROD_DIAL_MIN_MT || mt <= -TOY_PROD_DIAL_MIN_MT) {
        toy_event_t e = { 0 };
        e.type = TOY_EV_DIAL;
        e.id = 0; /* the device itself */
        e.v = (int32_t)mt;
        *ev = e;
        d->acc_mdps_ms -= mt * TOY_PROD_MDPS_MS_PER_MT;
        return true;
    }
    return false;
}

/* --- boot still-desk gyro bias calibration ---------------------
 *
 * The runner offers ~1 s of samples at boot; this decides
 * whether they look like a still desk and, if so, the mean
 * gz becomes the dial bias. Fail-closed at every step: any
 * sample with |a| outside 950..1050 mg (not resting at 1 g),
 * a gz spread over TOY_PROD_CAL_MAX_SPREAD_MDPS (moving; the
 * part's own noise is ~119 mdps RMS per SENSORS.md, so a
 * still second spreads far inside this), or a mean beyond
 * TOY_PROD_CAL_MAX_BIAS_MDPS (implausible as bias - that is
 * rotation, not offset) rejects the whole calibration and
 * the dial runs with bias 0 rather than a guessed one.
 */
#define TOY_PROD_CAL_MIN_SAMPLES 20u
#define TOY_PROD_CAL_MAX_SPREAD_MDPS 4000
#define TOY_PROD_CAL_MAX_BIAS_MDPS 8000

typedef struct {
    uint32_t count;
    bool bad; /* a sample already disqualified the set */
    int64_t sum_gz_mdps;
    int32_t min_gz_mdps;
    int32_t max_gz_mdps;
} toy_prod_cal_t;

/* Integer sqrt (floor) for the |a| check; values here are
 * ~1e6 (mg^2 sums), the Newton loop is a handful of steps. */
static inline uint32_t toy_prod_isqrt64(uint64_t v)
{
    uint64_t r = v;
    uint64_t y = (r + 1u) / 2u;
    while (y < r) {
        r = y;
        y = (r + v / r) / 2u;
    }
    return (uint32_t)r;
}

static inline void toy_prod_cal_init(toy_prod_cal_t *c)
{
    c->count = 0;
    c->bad = false;
    c->sum_gz_mdps = 0;
    c->min_gz_mdps = 0;
    c->max_gz_mdps = 0;
}

static inline void toy_prod_cal_add(toy_prod_cal_t *c, int32_t ax_mg,
                                    int32_t ay_mg, int32_t az_mg,
                                    int32_t gz_mdps)
{
    uint64_t mag2 = (uint64_t)((int64_t)ax_mg * ax_mg) +
                    (uint64_t)((int64_t)ay_mg * ay_mg) +
                    (uint64_t)((int64_t)az_mg * az_mg);
    uint32_t mag = toy_prod_isqrt64(mag2);
    if (mag < 950u || mag > 1050u)
        c->bad = true;
    if (c->count == 0u) {
        c->min_gz_mdps = gz_mdps;
        c->max_gz_mdps = gz_mdps;
    } else {
        if (gz_mdps < c->min_gz_mdps)
            c->min_gz_mdps = gz_mdps;
        if (gz_mdps > c->max_gz_mdps)
            c->max_gz_mdps = gz_mdps;
    }
    c->sum_gz_mdps += gz_mdps;
    c->count++;
}

/* True + *bias_out iff the set reads as a still desk. */
static inline bool toy_prod_cal_finish(const toy_prod_cal_t *c,
                                       int32_t *bias_out)
{
    if (c->bad || c->count < TOY_PROD_CAL_MIN_SAMPLES)
        return false;
    if (c->max_gz_mdps - c->min_gz_mdps > TOY_PROD_CAL_MAX_SPREAD_MDPS)
        return false;
    int64_t mean = c->sum_gz_mdps / (int64_t)c->count;
    if (mean > TOY_PROD_CAL_MAX_BIAS_MDPS ||
        mean < -TOY_PROD_CAL_MAX_BIAS_MDPS)
        return false;
    *bias_out = (int32_t)mean;
    return true;
}

/* --- IRQ-latched touch pending merge (touchfix2) --------------
 *
 * The CST9217 vendor flow is one report per INT falling edge
 * (board/touch_cst9217.h: Waveshare registers GPIO_IRQ_EDGE_FALL;
 * board/SENSORS.md). No sourced driver states INT stays low
 * until the report is read, so the product runner must not
 * level-sample it once per ~55 ms frame: a pulse that rises
 * again during render+flush would leave cst9217_should_read
 * with nothing but the 1 Hz probe. main_product.c therefore
 * latches every falling edge in an IRQ (flag + edge count,
 * nothing else in IRQ context) and merges it with the live
 * level here; the merged value is the int_pending input to
 * the untouched cst9217_should_read drain policy (pending OR
 * contact-tracked OR probe-due). The gesture tracker above
 * stays pure and unchanged: the IRQ only replaces the
 * level-poll's fall detection.
 */
static inline bool toy_prod_touch_pending_merge(bool irq_latched,
                                                bool int_low_now)
{
    return irq_latched || int_low_now;
}

/* --- gallery single-tap launch (touchfix2, runner-side) --------
 *
 * On glass the shell's pointer rule is select-then-tap-again
 * (core/toy_shell.c shell_gallery_event: a POINTER_DOWN on an
 * unselected gallery row only selects it; a DOWN on the
 * selected row activates). With taps delivered at frame rate
 * that made launching a two-lottery-tap sequence, so the
 * product runner (never the shell: keyboard/emu flow stays
 * select+enter, and core/ is untouched) confirms a real
 * gallery tap: when a DOWN in the cart rows selected a row
 * without TOY_SH_ACT_START_CART, the matching short, still UP
 * feeds ONE synthesised POINTER_DOWN at the DOWN coords,
 * which the shell then sees as a tap on the selected row.
 * GALLERY cart rows only: footer (RUN/SET), the SET CLOCK /
 * CLOCK FACE virtual rows, every other screen, an
 * already-launching DOWN, a >TOY_PROD_TAP_MAX_MS press, and
 * a press that strayed >TOY_PROD_TAP_TOL_MILLI from its DOWN
 * point (a drag) never synthesise.
 *
 * Row math replicates shell_gallery_event exactly (private
 * ML_* constants in core/toy_shell.c, milli-radius):
 * rows start at y=-620, height 155, TOY_SHELL_VISIBLE_ROWS=6
 * on glass, footer at y>=740.
 */
#define TOY_PROD_GALLERY_ROW_Y (-620)
#define TOY_PROD_GALLERY_ROW_H 155
#define TOY_PROD_GALLERY_VISIBLE_ROWS 6
#define TOY_PROD_GALLERY_FOOTER_Y 740
#define TOY_PROD_TAP_MAX_MS 500u
#define TOY_PROD_TAP_TOL_MILLI 80 /* ~19 glass px from DOWN point */

/* Gallery row under milli y, or -1 outside the row band
 * (title, detail gap and footer all return -1). total_rows
 * is cart_count + 2 (the two virtual rows), as in the shell. */
static inline int32_t toy_prod_gallery_row_at(int32_t milli_y,
                                              int32_t scroll,
                                              int32_t total_rows)
{
    if (milli_y < TOY_PROD_GALLERY_ROW_Y ||
        milli_y >= TOY_PROD_GALLERY_ROW_Y +
                       TOY_PROD_GALLERY_VISIBLE_ROWS *
                           TOY_PROD_GALLERY_ROW_H)
        return -1;
    int32_t row = scroll +
                  (milli_y - TOY_PROD_GALLERY_ROW_Y) /
                      TOY_PROD_GALLERY_ROW_H;
    return (row >= 0 && row < total_rows) ? row : -1;
}

typedef struct {
    bool armed;
    int32_t row;
    int32_t down_x; /* milli */
    int32_t down_y;
    uint64_t down_ms;
} toy_prod_tap_slot_t;

typedef struct {
    toy_prod_tap_slot_t slot[TOY_PROD_MAX_CONTACTS];
} toy_prod_tap_launch_t;

static inline void toy_prod_tap_launch_init(toy_prod_tap_launch_t *t)
{
    for (unsigned i = 0; i < TOY_PROD_MAX_CONTACTS; i++) {
        t->slot[i].armed = false;
        t->slot[i].row = -1;
        t->slot[i].down_x = 0;
        t->slot[i].down_y = 0;
        t->slot[i].down_ms = 0;
    }
}

static inline bool toy_prod_tap_still(int32_t x0, int32_t y0, int32_t x1,
                                      int32_t y1)
{
    int64_t dx = (int64_t)x1 - x0;
    int64_t dy = (int64_t)y1 - y0;
    return dx * dx + dy * dy <=
           (int64_t)TOY_PROD_TAP_TOL_MILLI * TOY_PROD_TAP_TOL_MILLI;
}

/* Call after a real POINTER_DOWN has been through the shell.
 * selected_after is the shell's gallery selection once the
 * DOWN landed; already_started is (act & START_CART). Arms
 * only when the DOWN selected a cart row without launching. */
static inline bool toy_prod_tap_launch_note_down(
    toy_prod_tap_launch_t *t, uint64_t now_ms, bool in_gallery,
    bool already_started, int32_t scroll, int32_t cart_count,
    int32_t selected_after, const toy_event_t *down)
{
    if (!down || down->type != TOY_EV_POINTER_DOWN ||
        down->id >= TOY_PROD_MAX_CONTACTS)
        return false;
    toy_prod_tap_slot_t *s = &t->slot[down->id];
    s->armed = false;
    if (!in_gallery || already_started)
        return false;
    int32_t row = toy_prod_gallery_row_at(down->y, scroll,
                                          cart_count + 2);
    if (row < 0 || row >= cart_count || row != selected_after)
        return false;
    s->armed = true;
    s->row = row;
    s->down_x = down->x;
    s->down_y = down->y;
    s->down_ms = now_ms;
    return true;
}

/* A MOVE that strays past the tap tolerance is a drag:
 * disarm for good (same discipline as the long-press). */
static inline void toy_prod_tap_launch_note_move(
    toy_prod_tap_launch_t *t, const toy_event_t *move)
{
    if (!move || move->type != TOY_EV_POINTER_MOVE ||
        move->id >= TOY_PROD_MAX_CONTACTS)
        return;
    toy_prod_tap_slot_t *s = &t->slot[move->id];
    if (s->armed &&
        !toy_prod_tap_still(s->down_x, s->down_y, move->x, move->y))
        s->armed = false;
}

/* Call on the matching real POINTER_UP (after it has been
 * through the shell). True + *synth filled (POINTER_DOWN at
 * the original DOWN milli coords) iff this press was a short
 * still tap on the still-selected cart row in GALLERY. The
 * slot is consumed either way: one tap, at most one synth. */
static inline bool toy_prod_tap_launch_take_up(
    toy_prod_tap_launch_t *t, uint64_t now_ms, bool in_gallery,
    int32_t scroll, int32_t cart_count, int32_t selected_now,
    const toy_event_t *up, toy_event_t *synth)
{
    if (!up || up->type != TOY_EV_POINTER_UP ||
        up->id >= TOY_PROD_MAX_CONTACTS || !synth)
        return false;
    toy_prod_tap_slot_t *s = &t->slot[up->id];
    if (!s->armed)
        return false;
    toy_prod_tap_slot_t was = *s;
    s->armed = false;
    if (!in_gallery || selected_now != was.row)
        return false;
    if (now_ms < was.down_ms ||
        now_ms - was.down_ms > TOY_PROD_TAP_MAX_MS)
        return false;
    if (!toy_prod_tap_still(was.down_x, was.down_y, up->x, up->y))
        return false;
    if (toy_prod_gallery_row_at(up->y, scroll, cart_count + 2) !=
        was.row)
        return false;
    toy_event_t e = { 0 };
    e.type = TOY_EV_POINTER_DOWN;
    e.id = up->id;
    e.x = was.down_x;
    e.y = was.down_y;
    *synth = e;
    return true;
}

#endif
