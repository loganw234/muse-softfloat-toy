/* live_feed.h - pure live-IMU feed seam for the product
 * runner (toy5-livefeed): QMI8658 raw counts -> the synthetic
 * scenarios' sample frame, latest-sample hold, source
 * selection, and the LIVE toy_sig_info_t a feed cart is
 * stamped with.
 *
 * Why this exists board-side, with no core change: core has
 * no live generator by design. toy_sig_poll() returns 0 /
 * END for KIND_LIVE (core/toy_signal.c), toy_feed_step() is
 * the proof-strict helper ("a future live runner feeds gaps
 * itself and does not use this proof-strict helper",
 * core/toy_cart.h), and toy_sig_label_feed()'s contract
 * already names that runner (core/toy_signal.h). So the
 * product runner is that future live runner: it fills the
 * frame here and calls cart->feed() directly with a LIVE
 * info. Proof runners (emu, main_board.c, WASM) never see
 * this header and stay synthetic-only.
 *
 * Posture, stated once: LATEST-SAMPLE HOLD. The core
 * contract asks for sig_spec.samples_per_step samples per
 * step regardless of wall time; the product polls the IMU
 * once per presented frame (tilt/dial cadence in
 * main_product.c, shared-bus accounting untouched). A step
 * therefore repeats the freshest real reading to fill its
 * frame. Repeats are counted (fresh vs repeated below),
 * never hidden, and motion is never fabricated: on a read
 * error the last good sample is held and the error is
 * counted + flagged; before the first good sample nothing
 * is fed at all (fill returns 0 and the cart renders its
 * NO SIG state). The sensor runs at its configured 125 Hz
 * ODR, but the runner consumes one reading per frame, so
 * the achieved fresh rate is the frame rate - the telemetry
 * in main_product.c prints it against the cart's spec rate
 * every second. info.sample_rate_hz therefore carries the
 * CART's assumed rate (the timebase its per-sample maths
 * was pinned against), not the ODR and not the achieved
 * fresh rate; both of those are telemetry, on serial.
 *
 * Host-compilable and SDK-free like product_input.h; the
 * only link dependency is core/toy_signal.c
 * (toy_sig_param_stream_id). tools/live_feed_test.c drives
 * every function here with a fake clock.
 */
#ifndef TOY_LIVE_FEED_H
#define TOY_LIVE_FEED_H

#include <stdbool.h>
#include <stdint.h>

#include "orientation.h" /* gravity-quadrant vector rotation */
#include "toy_signal.h" /* core: info/spec/label, param stream id */

/* --- source selection -------------------------------------------
 *
 * Decided once per cart start in the runner: a feed cart
 * whose device the board can serve live (IMU6 - the QMI8658
 * is the only live sensor) starts LIVE iff the IMU is up;
 * any other feed cart keeps its synthetic proof scenario
 * (the runner prints that loudly). A signal-deaf cart needs
 * no source. Selection is start-time only: an IMU that dies
 * mid-run degrades to hold + error counts, and an IMU that
 * recovers mid-run does not silently swap a running cart's
 * provenance - relaunch to go live. */
#define TOY_LIVE_FEED_NONE      0
#define TOY_LIVE_FEED_SYNTHETIC 1
#define TOY_LIVE_FEED_LIVE      2

static inline int toy_live_feed_select(int has_feed, uint32_t device,
                                       int imu_ok)
{
    if (!has_feed)
        return TOY_LIVE_FEED_NONE;
    if (device == TOY_SIG_DEVICE_IMU6 && imu_ok)
        return TOY_LIVE_FEED_LIVE;
    return TOY_LIVE_FEED_SYNTHETIC;
}

/* --- axis transform: sensor frame -> synthetic frame ------------
 *
 * Synthetic scenarios (core/toy_signal.c): flat screen-up
 * rests at az = +15864 (desk) / +16384 (chest) counts, and
 * the carts read the frame as ax right, ay screen-down, az
 * out of the screen (cart_gpend.c/cart_attitude.c: "ay
 * points screen-down", G = (-ax, +ay, -az) in right/up/out).
 * Hardware (board/SENSORS.md, product_input.h): flat
 * screen-up the QMI8658 reads az ~ -1000 mg, i.e. sensor +Z
 * points away from the viewer into the board. So az flips.
 * In-plane: ax/ay pass through under the same starting
 * alignment product_input.h's tilt map assumes (sensor ax
 * -> display +x, ay -> display +y/down), which also makes
 * a feed cart's falling direction agree with the TILT
 * events by construction: tilt sends (-ax, -ay) mg and
 * gpend's display falling direction is (-feed_ax,
 * -feed_ay). Gyro takes the accel signs per axis (no feed
 * cart consumes gyro today; gz flips with az so its sign
 * stays consistent with the screen-normal accel axis).
 * The in-plane alignment is NOT yet owner-observed on
 * glass - same open item as the tilt signs, one define per
 * axis to flip (board/PRODUCT.md live-feed checks). */
#ifndef TOY_LIVE_FEED_AX_SIGN
#define TOY_LIVE_FEED_AX_SIGN 1
#endif
#ifndef TOY_LIVE_FEED_AY_SIGN
#define TOY_LIVE_FEED_AY_SIGN 1
#endif
#ifndef TOY_LIVE_FEED_AZ_SIGN
#define TOY_LIVE_FEED_AZ_SIGN (-1)
#endif

static inline int16_t toy_live_feed_axis(int16_t v, int sign)
{
    int32_t x = sign < 0 ? -(int32_t)v : (int32_t)v;
    if (x > 32767)
        x = 32767; /* -(-32768) saturates, never wraps */
    return (int16_t)x;
}

/* raw6: sensor-native ax,ay,az,gx,gy,gz counts at the
 * driver's pinned scales (16384 LSB/g, 128 LSB/(deg/s) -
 * identical to the synthetic scales, board/imu_qmi8658.h),
 * so counts drop in unscaled. out6 may alias raw6. */
static inline void toy_live_feed_transform(const int16_t raw6[6],
                                           int16_t out6[6])
{
    int16_t tmp[6];
    tmp[0] = toy_live_feed_axis(raw6[0], TOY_LIVE_FEED_AX_SIGN);
    tmp[1] = toy_live_feed_axis(raw6[1], TOY_LIVE_FEED_AY_SIGN);
    tmp[2] = toy_live_feed_axis(raw6[2], TOY_LIVE_FEED_AZ_SIGN);
    tmp[3] = toy_live_feed_axis(raw6[3], TOY_LIVE_FEED_AX_SIGN);
    tmp[4] = toy_live_feed_axis(raw6[4], TOY_LIVE_FEED_AY_SIGN);
    tmp[5] = toy_live_feed_axis(raw6[5], TOY_LIVE_FEED_AZ_SIGN);
    for (int i = 0; i < 6; i++)
        out6[i] = tmp[i];
}

/* Gravity-quadrant rotation (toy5-rotate): the transform
 * above produces glass-frame in-plane axes; once the
 * presenter turns the picture with the device, a feed cart
 * must receive its in-plane vectors in canonical (picture)
 * coords or its drawn gravity would rotate AWAY from the
 * viewer's down (the same reason TILT events rotate in
 * product_input.h). So the runner uses this form: in-plane
 * accel (ax,ay) and gyro (gx,gy) pairs are inverse-rotated
 * by the quadrant; the screen-normal az/gz are unchanged.
 * Rotation is swap/sign only - magnitudes, |a|, and the
 * noise character are untouched. Quadrant 0 == the
 * unrotated transform verbatim. Proof/synthetic paths never
 * call either form. */
static inline int16_t toy_live_feed_sat16(int32_t v)
{
    if (v > 32767)
        return 32767;
    if (v < -32768)
        return -32768;
    return (int16_t)v;
}

static inline void toy_live_feed_transform_rot(const int16_t raw6[6],
                                               int16_t out6[6],
                                               int rotation)
{
    int16_t tmp[6];
    toy_live_feed_transform(raw6, tmp);
    int32_t rx, ry;
    toy_orient_inv_vec(rotation, tmp[0], tmp[1], &rx, &ry);
    tmp[0] = toy_live_feed_sat16(rx);
    tmp[1] = toy_live_feed_sat16(ry);
    toy_orient_inv_vec(rotation, tmp[3], tmp[4], &rx, &ry);
    tmp[3] = toy_live_feed_sat16(rx);
    tmp[4] = toy_live_feed_sat16(ry);
    for (int i = 0; i < 6; i++)
        out6[i] = tmp[i];
}

/* --- hold state + accounting ------------------------------------
 *
 * Runner-owned, never cartridge state (the source is the
 * host layer's, core/toy_signal.h). fresh/repeated are
 * counted per FED sample: a fill of n samples whose newest
 * reading has not been fed before counts 1 fresh + (n-1)
 * repeated; a fill with no new reading counts n repeated.
 * Window counters (*_w) reset at each 1 Hz telemetry print;
 * *_tot run since boot. max_age_w is the oldest
 * last-good age any fill in the window actually fed. */
typedef struct {
    int16_t last[6];      /* freshest good sample, synth frame */
    bool have;            /* a good sample exists at all */
    bool err_pending;     /* a read error since the last fill */
    uint64_t sample_seq;  /* good readings noted */
    uint64_t fed_seq;     /* sample_seq at the last fill */
    uint64_t last_good_ms;
    uint32_t fresh_w, rep_w, err_w, max_age_w;
    uint64_t fresh_tot, rep_tot, err_tot;
} toy_live_feed_t;

static inline void toy_live_feed_init(toy_live_feed_t *f)
{
    for (int i = 0; i < 6; i++)
        f->last[i] = 0;
    f->have = false;
    f->err_pending = false;
    f->sample_seq = 0;
    f->fed_seq = 0;
    f->last_good_ms = 0;
    f->fresh_w = f->rep_w = f->err_w = f->max_age_w = 0;
    f->fresh_tot = f->rep_tot = f->err_tot = 0;
}

/* Per-cart-start reset: counters restart, the held sample
 * survives (it is still the freshest real reading), and it
 * is marked already-fed so only readings taken after the
 * start count as this run's fresh ones. */
static inline void toy_live_feed_reset_run(toy_live_feed_t *f)
{
    f->fed_seq = f->sample_seq;
    f->err_pending = false;
    f->fresh_w = f->rep_w = f->err_w = f->max_age_w = 0;
}

static inline void toy_live_feed_reset_window(toy_live_feed_t *f)
{
    f->fresh_w = f->rep_w = f->err_w = f->max_age_w = 0;
}

static inline void toy_live_feed_note_sample_rot(toy_live_feed_t *f,
                                                 const int16_t raw6[6],
                                                 uint64_t now_ms,
                                                 int rotation)
{
    toy_live_feed_transform_rot(raw6, f->last, rotation);
    f->have = true;
    f->sample_seq++;
    f->last_good_ms = now_ms;
}

static inline void toy_live_feed_note_sample(toy_live_feed_t *f,
                                             const int16_t raw6[6],
                                             uint64_t now_ms)
{
    toy_live_feed_note_sample_rot(f, raw6, now_ms, 0);
}

static inline void toy_live_feed_note_error(toy_live_feed_t *f)
{
    f->err_w++;
    f->err_tot++;
    f->err_pending = true;
}

/* Fill one step's frame: n_samples x 6 interleaved counts,
 * the held sample repeated. Returns n_samples, or 0 when
 * no good sample exists yet (caller feeds nothing - the
 * cart keeps its KIND_NONE / NO SIG rendering; zero counts
 * would fabricate free-fall). *flags_out (may be NULL)
 * gets TOY_SIG_FLAG_UNDERRUN iff a read error landed since
 * the previous fill, so the cart's hashed label counts the
 * gap (toy_sig_label_feed) even though the held values are
 * real. A backwards clock reads as age 0. */
static inline uint32_t toy_live_feed_fill(toy_live_feed_t *f, int16_t *out,
                                          uint32_t n_samples,
                                          uint64_t now_ms,
                                          uint32_t *flags_out)
{
    if (flags_out)
        *flags_out = 0;
    if (!f->have || n_samples == 0u)
        return 0;
    for (uint32_t i = 0; i < n_samples; i++)
        for (int c = 0; c < 6; c++)
            out[(size_t)i * 6u + (size_t)c] = f->last[c];
    uint32_t fresh = (f->sample_seq != f->fed_seq) ? 1u : 0u;
    f->fresh_w += fresh;
    f->fresh_tot += fresh;
    f->rep_w += n_samples - fresh;
    f->rep_tot += n_samples - fresh;
    f->fed_seq = f->sample_seq;
    if (now_ms > f->last_good_ms) {
        uint64_t age = now_ms - f->last_good_ms;
        uint32_t age32 = age > 0xffffffffull ? 0xffffffffu
                                             : (uint32_t)age;
        if (age32 > f->max_age_w)
            f->max_age_w = age32;
    }
    if (f->err_pending) {
        if (flags_out)
            *flags_out = TOY_SIG_FLAG_UNDERRUN;
        f->err_pending = false;
    }
    return n_samples;
}

/* --- the LIVE info a feed cart is stamped with ------------------
 *
 * kind/orig LIVE, the spec's device and assumed rate, the
 * driver's pinned scales (identical to synthetic). seed 0
 * (recordings/live carry no seed, core/toy_signal.h),
 * scenario 0 (no synthetic scenario), n_samples 0
 * (unbounded). stream_id is the parameter-block hash with
 * kind=LIVE inside it, so a live run can never compare
 * equal to a synthetic one - the labelling rule does the
 * rest: feed() copies kind into hashed state via
 * toy_sig_label_feed and every feed cart renders that kind
 * (provenance() in each cart: SYNTHETIC/SYN vs LIVE). */
static inline void toy_live_feed_info(const toy_sig_spec_t *spec,
                                      toy_sig_info_t *info)
{
    info->kind = TOY_SIG_KIND_LIVE;
    info->orig_kind = TOY_SIG_KIND_LIVE;
    info->device = spec->device;
    info->channels = toy_sig_device_channels(spec->device);
    info->sample_rate_hz = spec->sample_rate_hz;
    info->scale_num = 16384u; /* LSB per g, as synthetic + driver */
    info->scale2_num = 128u;  /* LSB per deg/s, likewise */
    info->scenario_id = 0u;
    info->seed = 0u;
    info->n_samples = 0u;
    info->stream_id = toy_sig_param_stream_id(info);
}

#endif
