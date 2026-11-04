/* cart_gpend.c - a pendulum that falls toward fed IMU "down".
 *
 * The first registered signal-feed cartridge (SIG-1 seam,
 * docs/study-signals.md). Binding: IMU6 / SYNTHETIC proof /
 * scenario 1 ("desk") / 100 Hz, 1 sample per step, dt = 0.01 s
 * - exactly the study's typical binding (section 4 rule 5), so
 * the sample index of every consumed frame is the step index,
 * and the gravity a step integrates is the frame fed at its
 * boundary: feed() converts, step() integrates.
 *
 * The pendulum is constrained to the display plane, so what
 * drives it is the IN-PLANE component of gravity (the
 * out-of-plane component does no work on a planar bob; it is
 * still converted and stored, gz, inside the hashed bytes).
 * Axis convention (defined here, device frame = display frame):
 * ax points screen-right, ay points screen-down, az points out
 * of the screen. An accelerometer at rest reads specific force,
 * the negative of gravity, so with a in g units and y measured
 * UP on screen the gravity vector is
 *   G = 9.81 * (-ax, +ay, -az)   m/s^2 in (right, up, out).
 * In the desk stream (ax, ay) describe a 0.25 g circle once per
 * 20 s at az ~= 0.968 g, so the in-plane down direction rotates
 * through a full turn every 20 s and the bob visibly chases it.
 * The arrow length is proportional to the in-plane component
 * (~0.25 g on the desk), never renormalised to full length.
 *
 * Accel only, deliberately: desk-scenario gyro channels are
 * tremor-only and incoherent with the accel attitude
 * (core/toy_signal.h), so fusing them would manufacture
 * attitude motion the stream does not contain. They are read
 * nowhere here.
 *
 * Smoothing: band-limited tremor (~80 LSB sd) rides on the
 * 4096 LSB tilt circle. feed() applies a softfloat exponential
 * average g += 0.1 * (raw - g) per sample (tau ~ 0.1 s at
 * 100 Hz: tremor sd scaled by ~0.23, phase lag on the 20 s
 * circle ~0.03 rad). The first fed sample loads the average
 * outright. Before any feed, gravity is exactly zero and the
 * label says NO SIGNAL - no fake down is ever drawn.
 *
 * Integration: RK4 in binary64 softfloat, one dt per step. The
 * swing angle is held in CYCLES u in [0,1) (phi = 2 pi u,
 * measured from screen-down) with w = du/dt in cycles/s, so
 * trig is f64_sin_pi/f64_cos_pi at 2u and nothing winds without
 * bound even though the equilibrium precesses forever:
 *   dw/dt = (Gx cos phi + Gy sin phi) / (2 pi L) - damp * w.
 * L = 0.25 m under the desk's 0.25 g in-plane field gives a
 * ~2.0 s small-swing period, a decade faster than the 20 s
 * down precession, so the bob tracks down with a live swing
 * about it. The tap at t = 30 s in the proof stream reaches ax
 * at quarter amplitude and kicks the in-plane field visibly.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define GTRAIL 180 /* bob tip trail, one point per step (1.8 s)  */
#define DTRAIL 240 /* down samples, one per 10 steps (24 s)       */

typedef struct {
    f64 u;          /* swing angle from screen-down, cycles [0,1) */
    f64 w;          /* du/dt, cycles/s                            */
    f64 gx, gy, gz; /* smoothed gravity, m/s^2 (right, up, out)   */
    f64 dt, len, damp;
    f64 g0;            /* 9.81 m/s^2 per g                        */
    f64 g_per_lsb;     /* 1/16384: +/-2 g scale, from decimal     */
    f64 alpha;         /* EMA weight per fed sample               */
    f64 inv_2pi_len;   /* 1/(2 pi L), derived at reset            */
    toy_sig_label_t label; /* stream provenance, in hashed state  */
    uint64_t steps;
    f64 tip[GTRAIL][2];   /* bob direction history (right, up)    */
    f64 down[DTRAIL][2];  /* (gx, gy) history, m/s^2              */
    int32_t raw[3];       /* newest accel counts ax, ay, az       */
    uint32_t have_grav;   /* 0 until the first fed sample         */
    uint32_t down_writes; /* down-history samples stored so far   */
} gpend_state;

static void tip_pos(f64 u, f64 *x, f64 *y)
{
    f64 two_u = f64_mul(F64_TWO, u);
    *x = f64_sin_pi(two_u);
    *y = f64_neg(f64_cos_pi(two_u));
}

static void gpend_reset(void *st_)
{
    gpend_state *s = (gpend_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->dt = f64_dec("0.01");   /* 1 sample/step at 100 Hz */
    s->len = f64_dec("0.25");
    s->damp = f64_dec("0.08");
    s->g0 = f64_dec("9.81");
    s->g_per_lsb = f64_div(F64_ONE, f64_dec("16384"));
    s->alpha = f64_dec("0.1");
    s->inv_2pi_len =
        f64_div(F64_ONE, f64_mul(f64_dec("6.283185307179586"), s->len));
    /* u = 0.4 cycles = 144 deg from screen-down: near the desk
     * stream's initial down (screen-up at t = 0), offset by
     * 36 deg so the swing about fed down starts immediately. */
    s->u = f64_dec("0.4");
    toy_sig_label_reset(&s->label); /* NONE: visible NO SIGNAL */
    f64 x, y;
    tip_pos(s->u, &x, &y);
    for (int t = 0; t < GTRAIL; t++) { s->tip[t][0] = x; s->tip[t][1] = y; }
}

/* feed(): state effects only. Convert each fed accel frame from
 * int16 counts to g units via the seam's integer->f64 helper and
 * the reset-derived decimal scale, take the negative (gravity,
 * not specific force), and fold into the smoothed vector.
 * Gyro counts (channels 3-5) are ignored by design (see header).
 * No integration happens here; step() owns time. */
static void gpend_feed(void *st_, const int16_t *samples, uint32_t n,
                       uint32_t channels, const toy_sig_info_t *info,
                       uint32_t flags)
{
    gpend_state *s = (gpend_state *)st_;
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
        /* G in (right, up, out), m/s^2: gravity = -specific force,
         * and ay points screen-down so its up-component flips. */
        f64 rgx = f64_neg(f64_mul(s->g0, ax));
        f64 rgy = f64_mul(s->g0, ay);
        f64 rgz = f64_neg(f64_mul(s->g0, az));
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

typedef struct { f64 u, w; } gp_deriv;

static gp_deriv rhs(const gpend_state *s, f64 u, f64 w)
{
    f64 two_u = f64_mul(F64_TWO, u);
    /* Tangential gravity G.t, t = (cos phi, sin phi). */
    f64 tang = f64_fma(s->gy, f64_sin_pi(two_u),
                       f64_mul(s->gx, f64_cos_pi(two_u)));
    gp_deriv r;
    r.u = w;
    r.w = f64_sub(f64_mul(tang, s->inv_2pi_len), f64_mul(s->damp, w));
    return r;
}

/* Cycles stay in [0,1): |du| per step is < ~0.03 even under a
 * saturated 2 g axis, so a few conditional turns always land
 * back in range. Sign-bit comparisons as in cart_harmonograph. */
static f64 wrap_cycles(f64 u)
{
    for (int i = 0; i < 4; i++) {
        if ((u & F64_SIGN) != 0 && u != F64_SIGN) { /* u < 0 */
            u = f64_add(u, F64_ONE);
            continue;
        }
        f64 d = f64_sub(u, F64_ONE);
        if ((d & F64_SIGN) != 0) break; /* u < 1: done */
        u = d;
    }
    return u;
}

static void gpend_step(void *st_)
{
    gpend_state *s = (gpend_state *)st_;
    f64 h = s->dt, hh = f64_mul(h, F64_HALF), h6 = f64_div(h, f64_dec("6"));
    gp_deriv k1 = rhs(s, s->u, s->w);
    gp_deriv k2 = rhs(s, f64_fma(k1.u, hh, s->u), f64_fma(k1.w, hh, s->w));
    gp_deriv k3 = rhs(s, f64_fma(k2.u, hh, s->u), f64_fma(k2.w, hh, s->w));
    gp_deriv k4 = rhs(s, f64_fma(k3.u, h, s->u), f64_fma(k3.w, h, s->w));
    f64 du = f64_add(f64_add(k1.u, f64_mul(F64_TWO, f64_add(k2.u, k3.u))), k4.u);
    f64 dw = f64_add(f64_add(k1.w, f64_mul(F64_TWO, f64_add(k2.w, k3.w))), k4.w);
    s->u = wrap_cycles(f64_fma(du, h6, s->u));
    s->w = f64_fma(dw, h6, s->w);
    s->steps++;
    f64 x, y;
    tip_pos(s->u, &x, &y);
    size_t slot = (size_t)(s->steps % GTRAIL);
    s->tip[slot][0] = x;
    s->tip[slot][1] = y;
    if (s->have_grav && (s->steps % 10u) == 0u) {
        size_t dslot = (size_t)(s->down_writes % DTRAIL);
        s->down[dslot][0] = s->gx;
        s->down[dslot][1] = s->gy;
        s->down_writes++;
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
    if (l->kind == TOY_SIG_KIND_SYNTHETIC) return "SYNTHETIC";
    if (l->kind == TOY_SIG_KIND_LIVE) return "LIVE";
    if (l->kind == TOY_SIG_KIND_REPLAY) {
        if (l->orig_kind == TOY_SIG_KIND_LIVE) return "REPLAY-LIVE";
        if (l->orig_kind == TOY_SIG_KIND_SYNTHETIC) return "REPLAY-SYN";
        return "REPLAY";
    }
    return "NO SIGNAL";
}

static void gpend_render(const void *st_, toy_fb_t *fb)
{
    const gpend_state *s = (const gpend_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    int sc = r >= 150 ? 2 : 1;
    uint16_t bob = toy_rgb565(255, 170, 40);
    uint16_t down_c = toy_rgb565(70, 220, 255);
    uint16_t ink = toy_rgb565(150, 170, 200);
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));
    /* Screen-down reference tick at the rim, so fed down moving
     * away from the bottom of the glass reads at a glance. */
    toy_fb_line(fb, cx, cy + r - 12 * sc, cx, cy + r - 3, ink);

    if (s->have_grav) {
        /* Down history: (gx, gy)/9.81 at 1.2 R per g, so the
         * desk's 0.25 g circle lands at 0.30 R. Nativeness is
         * display-only placement of already-computed state. */
        toy_fb_ring(fb, cx, cy, (r * 30) / 100, toy_rgb565(18, 34, 48));
        double k = 1.2 * (double)r / 9.81;
        uint32_t valid = s->down_writes < DTRAIL ? s->down_writes : DTRAIL;
        for (uint32_t t = 0; t < valid; t++) {
            size_t idx = (size_t)((s->down_writes - 1u - t) % DTRAIL);
            int x = cx + (int)(d64(s->down[idx][0]) * k);
            int y = cy - (int)(d64(s->down[idx][1]) * k);
            unsigned fade = 200u - (unsigned)(t * 160u / DTRAIL);
            unsigned p = down_c;
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        int dx = cx + (int)(d64(s->gx) * k);
        int dy = cy - (int)(d64(s->gy) * k);
        toy_fb_line(fb, cx, cy, dx, dy, down_c);
        toy_fb_line(fb, cx + 1, cy, dx + 1, dy, down_c);
        toy_fb_disc(fb, dx, dy, r / 60 + 1, down_c);
    }

    /* Bob trail, then rod and bob at 0.78 R. */
    double scale = (double)r * 0.78;
    size_t head = (size_t)(s->steps % GTRAIL);
    for (int t = 0; t < GTRAIL; t++) {
        size_t idx = (head + GTRAIL - (size_t)t) % GTRAIL;
        int x = cx + (int)(d64(s->tip[idx][0]) * scale);
        int y = cy - (int)(d64(s->tip[idx][1]) * scale);
        unsigned fade = 255u - (unsigned)(t * 210 / GTRAIL);
        unsigned p = bob;
        uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                (((p & 31u) * fade) >> 8));
        toy_fb_px(fb, x, y, c);
    }
    f64 tx, ty;
    tip_pos(s->u, &tx, &ty);
    int bx = cx + (int)(d64(tx) * scale);
    int by = cy - (int)(d64(ty) * scale);
    toy_fb_line(fb, cx, cy, bx, by, bob);
    toy_fb_disc(fb, bx, by, r / 45 + 2, bob);
    toy_fb_disc(fb, cx, cy, r / 70 + 1, 0xffffu);

    text_centred(fb, cx, cy - r + 10 * sc, sc, ink, "GPEND");
    text_centred(fb, cx, cy + r - 22 * sc, sc,
                 s->label.kind == TOY_SIG_KIND_NONE
                     ? toy_rgb565(255, 90, 90) : ink,
                 provenance(&s->label));
}

const toy_cart_t toy_cart_gpend = {
    "gpend", "Gravity pendulum (IMU)",
    "A pendulum falling toward the fed accel's down. Proof stream is synthetic desk tilt: watch down precess.",
    sizeof(gpend_state), 3, 3200, 400,
    gpend_reset, gpend_step, gpend_render,
    0, 0, 0, 0, 0, 0, 0,
    gpend_feed,
    { TOY_SIG_DEVICE_IMU6, TOY_SIG_KIND_SYNTHETIC, 1u, 100u, 1u,
      UINT64_C(0x5eed) }
};
