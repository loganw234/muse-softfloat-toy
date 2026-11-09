/* cart_attitude.c - artificial horizon + gravity ball (IMU accel).
 *
 * Binding: IMU6 / SYNTHETIC proof / scenario 1 ("desk") / 100 Hz,
 * 1 sample per step, dt = 0.01 s - the study's typical attitude
 * binding (docs/study-signals.md section 4 rule 5) and gpend's
 * landed precedent. The sample index of every consumed frame is
 * the step index; feed() converts, step() derives, render()
 * only places state on the glass.
 *
 * Accel ONLY, deliberately: the desk scenario's gyro channels
 * are tremor-only and are NOT the derivative of the accel
 * attitude (study section 8, "SIG-6" note; core/toy_signal.h),
 * so fusing them would manufacture rotation the stream does
 * not contain. Gyro counts (channels 3-5) are read nowhere
 * here. A gyro-coherent scenario is a future stream change,
 * not something this cart can infer.
 *
 * Axis convention (device frame = display frame, same as
 * cart_gpend): ax points screen-right, ay points screen-down,
 * az points out of the screen. An accelerometer at rest reads
 * specific force, the negative of gravity, so with a in g units
 * and y measured UP on screen the gravity vector is
 *   G = (-ax, +ay, -az)   in (right, up, out), g units.
 * Level glass face-up therefore reads az ~ +1 g, G ~ (0,0,-1):
 * in-plane tilt ~0. The desk stream instead holds az ~0.968 g
 * while (ax, ay) describe a 0.25 g circle once per 20 s, so
 * in-plane down precesses through a full turn every 20 s.
 * Kit axis orientation is unverified until hardware lands;
 * this convention is defined here and shared with gpend.
 *
 * feed() converts all three accel axes from int16 counts via
 * toy_f64_bits_from_i32 and the reset-derived 1/16384 g/LSB
 * scale, negates into gravity, and folds each sample into a
 * smoothed vector with the softfloat exponential average
 *   g += alpha * (raw - g),  alpha = 0.05 per sample
 * (tau ~19.5 samples ~0.2 s at 100 Hz: band tremor sd scaled
 * by ~0.16, phase lag on the 20 s circle ~0.06 rad). The
 * first fed sample loads the average outright. Before any
 * feed the vector is exactly zero, have_grav is 0 and the
 * embedded label says NO SIGNAL - no fake level is drawn.
 *
 * No angles anywhere, by design. The sealed library offers
 * forward trig in pi units (sinPi/cosPi) but no inverse trig,
 * and this display needs none: pitch/roll for a face-up glass
 * are exactly the in-plane gravity components. step() derives
 * in binary64 softfloat, from the smoothed vector alone,
 *   tilt  = sqrt(gx^2 + gy^2)        (in-plane magnitude, g)
 *   total = sqrt(gx^2 + gy^2 + gz^2) (|G|, g)
 *   level = (tilt < 0.02 g)          (sign-bit test, in state)
 * tilt is sin(angle from face-normal) only when |G| = 1 g; it
 * is NEVER renormalised by total, so a level glass reads
 * tilt ~0 (ball centred, LEVEL shown) and dynamic accel (the
 * desk tap, live shaking) honestly moves the ball - accel
 * alone cannot separate gravity from acceleration, and this
 * cart does not pretend otherwise.
 *
 * Render semantics (native double places state only):
 *  - sky/ground split: the disc half toward in-plane down is
 *    ground, the opposite half sky, divided by the horizon
 *    line through the centre perpendicular to in-plane down.
 *    The line's ORIENTATION uses direction only; magnitude
 *    lives in the ball. Below the level threshold direction
 *    is noise, so no split is drawn and LEVEL is shown.
 *  - ball-in-disc: ball offset = (gx, gy) at 2 disc radii
 *    per g, unclamped and unrenormalised; the disc rim is
 *    0.50 g and rings sit at 0.125/0.25/0.375/0.50 g, so the
 *    desk ball rides the 0.25 g ring, precessing with the
 *    stream, and >0.50 g drives it past the rim. A numeric
 *    TILT readout (hundredths of g) states the magnitude.
 *  - provenance text is rendered from the hashed label:
 *    SYNTHETIC is named synthetic, never dressed as measured.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define ATRAIL 240 /* (gx, gy) samples, one per 10 steps (24 s) */

typedef struct {
    f64 gx, gy, gz;      /* smoothed gravity, g (right, up, out)  */
    f64 tilt;            /* sqrt(gx^2+gy^2), g, step-derived      */
    f64 total;           /* |G|, g, step-derived                  */
    f64 g_per_lsb;       /* 1/16384, from decimal at reset        */
    f64 alpha;           /* EMA weight per fed sample (0.05)      */
    f64 level_thr;       /* level threshold (0.02 g)              */
    toy_sig_label_t label; /* stream provenance, in hashed state  */
    uint64_t steps;
    f64 hist[ATRAIL][2]; /* (gx, gy) history, g                   */
    int32_t raw[3];      /* newest accel counts ax, ay, az        */
    uint32_t have_grav;  /* 0 until the first fed sample          */
    uint32_t hist_writes;/* history samples stored so far         */
    uint32_t level;      /* 1 when fed and tilt < level_thr       */
} attitude_state;
/* State bytes: 8 f64 (64) + label (32) + steps (8) +
 * hist (240*2*8 = 3840) + six 4-byte words (24) = 3968 bytes,
 * fields in descending alignment, padding zeroed by reset. */

static void attitude_reset(void *st_)
{
    attitude_state *s = (attitude_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->g_per_lsb = f64_div(F64_ONE, f64_dec("16384"));
    s->alpha = f64_dec("0.05");
    s->level_thr = f64_dec("0.02");
    toy_sig_label_reset(&s->label); /* NONE: visible NO SIGNAL */
}

/* feed(): state effects only. Convert each fed accel frame to
 * a gravity vector in g units and fold into the EMA. Gyro
 * counts are ignored by design (see header). No derivation
 * here; step() owns the attitude quantities. */
static void attitude_feed(void *st_, const int16_t *samples, uint32_t n,
                          uint32_t channels, const toy_sig_info_t *info,
                          uint32_t flags)
{
    attitude_state *s = (attitude_state *)st_;
    toy_sig_label_feed(&s->label, info, n, flags);
    if (!samples || channels < 3u) return;
    for (uint32_t i = 0; i < n; i++) {
        const int16_t *f = samples + (size_t)i * channels;
        s->raw[0] = f[0];
        s->raw[1] = f[1];
        s->raw[2] = f[2];
        f64 ax = f64_mul((f64)toy_f64_bits_from_i32(f[0]), s->g_per_lsb);
        f64 ay = f64_mul((f64)toy_f64_bits_from_i32(f[1]), s->g_per_lsb);
        f64 az = f64_mul((f64)toy_f64_bits_from_i32(f[2]), s->g_per_lsb);
        /* G in (right, up, out), g: gravity = -specific force,
         * and ay points screen-down so its up-component flips. */
        f64 rgx = f64_neg(ax);
        f64 rgy = ay;
        f64 rgz = f64_neg(az);
        if (!s->have_grav) {
            s->gx = rgx; s->gy = rgy; s->gz = rgz;
            s->have_grav = 1;
        } else {
            s->gx = f64_fma(s->alpha, f64_sub(rgx, s->gx), s->gx);
            s->gy = f64_fma(s->alpha, f64_sub(rgy, s->gy), s->gy);
            s->gz = f64_fma(s->alpha, f64_sub(rgz, s->gz), s->gz);
        }
    }
}

static void attitude_step(void *st_)
{
    attitude_state *s = (attitude_state *)st_;
    s->steps++;
    if (!s->have_grav) return; /* tilt/total stay +0, level 0 */
    f64 tilt2 = f64_fma(s->gx, s->gx, f64_mul(s->gy, s->gy));
    s->tilt = f64_sqrt(tilt2);
    s->total = f64_sqrt(f64_fma(s->gz, s->gz, tilt2));
    f64 d = f64_sub(s->tilt, s->level_thr);
    s->level = ((d & F64_SIGN) != 0) ? 1u : 0u; /* tilt < thr */
    if ((s->steps % 10u) == 0u) {
        size_t slot = (size_t)(s->hist_writes % ATRAIL);
        s->hist[slot][0] = s->gx;
        s->hist[slot][1] = s->gy;
        s->hist_writes++;
    }
}

static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

static void text_centred(toy_fb_t *fb, int cx, int y, int sc, uint16_t col,
                         const char *s)
{
    int n = 0;
    while (s[n]) n++;
    toy_fb_text(fb, cx - (n * 4 - 1) * sc / 2, y, sc, col, s);
}

static const char *provenance(const toy_sig_label_t *l)
{
    if (l->kind == TOY_SIG_KIND_SYNTHETIC) return "SYNTHETIC IMU";
    if (l->kind == TOY_SIG_KIND_LIVE) return "LIVE IMU";
    if (l->kind == TOY_SIG_KIND_REPLAY) {
        if (l->orig_kind == TOY_SIG_KIND_LIVE) return "REPLAY-LIVE";
        if (l->orig_kind == TOY_SIG_KIND_SYNTHETIC) return "REPLAY-SYN";
        return "REPLAY";
    }
    return "NO SIGNAL";
}

static uint16_t fade_colour(unsigned p, unsigned fade)
{
    return (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                      ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                      (((p & 31u) * fade) >> 8));
}

/* Exact floor(sqrt(v)) for v >= 0 (bit method, long math so
 * no intermediate overflows for any int v). */
static int at_isqrt(int v)
{
    long vv = v, r = 0, bit = 1L << 30;
    while (bit > vv) bit >>= 2;
    while (bit) {
        if (vv >= r + bit) { vv -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return (int)r;
}

/* The sky/ground split test, the original expression on the
 * original operands. */
static int at_is_ground(int x, int y, double ux, double uy)
{
    return (double)x * ux + (double)y * uy > 0.0;
}

/* Fill relative span x0..x1 on absolute row py under the same
 * [0,w) x [0,h) clip toy_fb_px applies per pixel. */
static void at_span(toy_fb_t *fb, int cx, int py, int x0, int x1,
                    uint16_t c)
{
    if ((unsigned)py >= (unsigned)fb->h) return;
    int ax0 = cx + x0, ax1 = cx + x1;
    if (ax0 < 0) ax0 = 0;
    if (ax1 > fb->w - 1) ax1 = fb->w - 1;
    uint16_t *row = fb->px + (size_t)py * (size_t)fb->w;
    for (int x = ax0; x <= ax1; x++) row[x] = c;
}

static void attitude_render(const void *st_, toy_fb_t *fb)
{
    const attitude_state *s = (const attitude_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    int sc = r >= 150 ? 2 : 1;
    int rd = (r * 70) / 100; /* horizon disc radius; rim = 0.50 g */
    uint16_t ink = toy_rgb565(150, 170, 200);
    uint16_t sky = toy_rgb565(26, 72, 138);
    uint16_t ground = toy_rgb565(96, 62, 32);
    uint16_t ball_c = toy_rgb565(255, 170, 40);
    uint16_t down_c = toy_rgb565(70, 220, 255);
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));

    /* Screen-space down (right, screen-down) from state. */
    double gx = d64(s->gx), gy = d64(s->gy), tilt = d64(s->tilt);
    int split = s->have_grav && !s->level && tilt > 0.0;
    double ux = 0.0, uy = 0.0; /* unit down, screen coords */
    if (split) { ux = gx / tilt; uy = -gy / tilt; }

    /* Sky/ground fill, then the horizon line itself.
     * Cost shape (toy5-render, pixel-identical): each row is
     * clipped to the disc by exact integer sqrt (the same
     * x*x + y*y <= rd*rd set the square scan tested), and the
     * split test - this frame's soft-double bulk on target,
     * ~50k dot products at 360 - is evaluated only to binary-
     * search each row's sky/ground edge (~8 evaluations/row).
     * For fixed y the tested value is non-decreasing in x
     * when ux > 0: the exact products (double)x * ux increase
     * with x, IEEE rounding is monotone, and adding the row
     * constant (double)y * uy keeps both. Ground pixels are
     * therefore one suffix of the row (a prefix for ux < 0).
     * The search evaluates the original predicate itself, so
     * it only chooses WHICH x to test, never a different
     * arithmetic. ux == 0 makes the test x-independent
     * ((double)x * +/-0 is +/-0, and +/-0 + v compares as v -
     * the +/-0 corner stays not > 0 either way; NaN ux/uy
     * makes it false everywhere), so one evaluation colours
     * the row; non-split frames were already one colour. */
    for (int y = -rd; y <= rd; y++) {
        int xmax = at_isqrt(rd * rd - y * y);
        if (!split) {
            at_span(fb, cx, cy + y, -xmax, xmax,
                    s->have_grav ? sky : toy_rgb565(16, 20, 34));
        } else if (ux > 0.0) {
            int lo = -xmax, hi = xmax, edge = xmax + 1;
            while (lo <= hi) {
                int mid = lo + (hi - lo) / 2;
                if (at_is_ground(mid, y, ux, uy)) { edge = mid; hi = mid - 1; }
                else lo = mid + 1;
            }
            if (edge > -xmax) at_span(fb, cx, cy + y, -xmax, edge - 1, sky);
            if (edge <= xmax) at_span(fb, cx, cy + y, edge, xmax, ground);
        } else if (ux < 0.0) {
            int lo = -xmax, hi = xmax, edge = -xmax - 1;
            while (lo <= hi) {
                int mid = lo + (hi - lo) / 2;
                if (at_is_ground(mid, y, ux, uy)) { edge = mid; lo = mid + 1; }
                else hi = mid - 1;
            }
            if (edge >= -xmax) at_span(fb, cx, cy + y, -xmax, edge, ground);
            if (edge < xmax) at_span(fb, cx, cy + y, edge + 1, xmax, sky);
        } else {
            at_span(fb, cx, cy + y, -xmax, xmax,
                    at_is_ground(0, y, ux, uy) ? ground : sky);
        }
    }
    if (split) {
        int ex = (int)(-uy * (double)rd), ey = (int)(ux * (double)rd);
        toy_fb_line(fb, cx - ex, cy - ey, cx + ex, cy + ey, 0xffffu);
    }

    /* Tilt rings at 0.125/0.25/0.375/0.50 g and fixed glass
     * furniture: rim ticks and a short centre cross that stays
     * with the glass while down precesses around it. */
    uint16_t ring_c = toy_rgb565(190, 200, 220);
    toy_fb_ring(fb, cx, cy, rd / 4, ring_c);
    toy_fb_ring(fb, cx, cy, rd / 2, ring_c);
    toy_fb_ring(fb, cx, cy, (3 * rd) / 4, ring_c);
    toy_fb_ring(fb, cx, cy, rd, ring_c);
    toy_fb_line(fb, cx, cy - rd, cx, cy - rd + 5 * sc, 0xffffu);
    toy_fb_line(fb, cx, cy + rd - 5 * sc, cx, cy + rd, 0xffffu);
    toy_fb_line(fb, cx - rd, cy, cx - rd + 5 * sc, cy, 0xffffu);
    toy_fb_line(fb, cx + rd - 5 * sc, cy, cx + rd, cy, 0xffffu);
    int tick = rd / 8 + 2;
    toy_fb_line(fb, cx - tick, cy, cx + tick, cy, 0xffffu);
    toy_fb_line(fb, cx, cy - tick, cx, cy + tick, 0xffffu);

    if (s->have_grav) {
        /* Ball + down vector + trail at 2 radii per g, from the
         * unrenormalised (gx, gy) in state. */
        double k = 2.0 * (double)rd;
        uint32_t valid = s->hist_writes < ATRAIL ? s->hist_writes : ATRAIL;
        for (uint32_t t = 0; t < valid; t++) {
            size_t idx = (size_t)((s->hist_writes - 1u - t) % ATRAIL);
            int x = cx + (int)(d64(s->hist[idx][0]) * k);
            int y = cy - (int)(d64(s->hist[idx][1]) * k);
            unsigned fade = 200u - (unsigned)(t * 150u / ATRAIL);
            toy_fb_px(fb, x, y, fade_colour(down_c, fade));
        }
        int bx = cx + (int)(gx * k);
        int by = cy - (int)(gy * k);
        toy_fb_line(fb, cx, cy, bx, by, down_c);
        toy_fb_disc(fb, bx, by, r / 45 + 2, ball_c);
        toy_fb_disc(fb, bx, by, r / 90 + 1, 0xffffu);
        if (s->level)
            text_centred(fb, cx, cy + rd / 3, sc, 0xffffu, "LEVEL");
    } else {
        text_centred(fb, cx, cy - 3 * sc, sc, toy_rgb565(255, 90, 90),
                     "NO SIGNAL");
    }
    toy_fb_disc(fb, cx, cy, r / 70 + 1, 0xffffu);

    text_centred(fb, cx, cy - r + 10 * sc, sc, ink, "ATTITUDE");
    if (s->have_grav) {
        /* Display-only formatting of the state tilt, hundredths
         * of g (rounded); never flows back into state. */
        int hund = (int)(tilt * 100.0 + 0.5);
        if (hund < 0) hund = 0;
        if (hund > 999) hund = 999;
        char buf[16];
        buf[0] = 'T'; buf[1] = 'I'; buf[2] = 'L'; buf[3] = 'T';
        buf[4] = ' ';
        buf[5] = (char)('0' + hund / 100);
        buf[6] = '.';
        buf[7] = (char)('0' + (hund / 10) % 10);
        buf[8] = (char)('0' + hund % 10);
        buf[9] = 'G';
        buf[10] = '\0';
        text_centred(fb, cx, cy + r - 31 * sc, sc, ink, buf);
    }
    text_centred(fb, cx, cy + r - 22 * sc, sc,
                 s->label.kind == TOY_SIG_KIND_NONE
                     ? toy_rgb565(255, 90, 90) : ink,
                 provenance(&s->label));
}

/* demo 3 steps/frame: at the nominal 30 fps the 20 s precession
 * plays in ~22 s of wall time, near stream rate (gpend's pace).
 * proof 2500 steps x 1 sample = 25 s: one full 20 s precession
 * (2000 steps) plus a quarter turn; reports every 250 steps pin
 * eight 45-degree tilt phases and the return past full turn.
 * Per-step work is a handful of softfloat ops plus two sqrts,
 * so the board proof is far inside the routine gate budget. */
const toy_cart_t toy_cart_attitude = {
    "attitude", "Attitude horizon (IMU)",
    "Artificial horizon and gravity ball from fed accel. Synthetic desk tilt precesses every 20 s.",
    sizeof(attitude_state), 3, 2500, 250,
    attitude_reset, attitude_step, attitude_render,
    0, 0, 0, 0, 0, 0, 0,
    attitude_feed,
    { TOY_SIG_DEVICE_IMU6, TOY_SIG_KIND_SYNTHETIC, 1u, 100u, 1u,
      UINT64_C(0x5eed) }
};
