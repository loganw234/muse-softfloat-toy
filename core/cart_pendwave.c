/* cart_pendwave.c - pendulum-wave ring.
 *
 * Twenty nonlinear pendulums, th'' = -(g/L) sin(th), integrated by
 * one RK4 step of dt = 0.01 s per step() in muse-softfloat binary64.
 * Trig is f64_sin_rad (sinPi behind it, see toy_math.h); angles stay
 * in [-0.6, 0.6] rad, never wind, so radians are safe here.
 *
 * Classic pendulum-wave tuning: in one recurrence period
 * T_rec = 30 s pendulum k completes exactly 20+k whole oscillations
 * (k = 0..19, frequencies 2/3 Hz .. 1.3 Hz). At T_rec/2 neighbours
 * are antiphased, at T_rec/4 the phases repeat every four, and at
 * T_rec every pendulum is back at the common start phase.
 *
 * Lengths are derived at reset entirely in softfloat from decimal
 * constants: L = g / omega0^2 with omega0 = F * 2*pi*(20+k)/T_rec.
 * F is the nonlinear period correction at amplitude th0 = 0.6 rad,
 * F = (2/pi) K(sin(th0/2)) = sum_n ((2n-1)!!/(2n)!!)^2 m^n with
 * m = sin(th0/2)^2, taken through m^5. The exact large-angle period
 * is T = (2*pi/omega0) F, so dividing it out at reset makes the
 * *nonlinear* frequencies land on (20+k)/T_rec, not the small-angle
 * ones; F is common to all k because all amplitudes are equal.
 * Resulting lengths are about 0.534 m (k=0) down to 0.141 m (k=19).
 *
 * dt choice: omega0*dt <= 8.36 * 0.01 = 0.084, so the RK4 phase
 * error over 39 cycles is of order (omega*T)*(omega*dt)^4/120
 * ~ 1e-4 rad (a native-double probe of the same tuning at dt=0.01
 * returns to th0 within 4e-6 rad at T_rec) - far below the phase
 * differences that draw the patterns. 3000 steps = one recurrence;
 * demo pacing 75 steps/frame makes the standard 40-frame sheet
 * span exactly one recurrence, last frame re-converged.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define PW_N 20
#define PW_REC_STEPS 3000u /* T_rec / dt = 30 s / 0.01 s */

typedef struct {
    f64 th[PW_N], w[PW_N];   /* angle (rad), angular velocity */
    f64 om2[PW_N];           /* g/L per pendulum */
    f64 len[PW_N];           /* derived lengths (m); kept for the record */
    f64 rx[PW_N], ry[PW_N];  /* unit ring directions, reset-time constants */
    f64 dt, dt_half, dt_sixth;
    f64 g, th0, corr;
    uint64_t steps;
} pendwave_state;

static f64 accel(f64 om2, f64 th)
{
    return f64_neg(f64_mul(om2, f64_sin_rad(th)));
}

static void rk4_one(f64 om2, f64 *thp, f64 *wp, f64 h, f64 hh, f64 h6)
{
    f64 th = *thp, w = *wp;
    f64 k1t = w,                    k1w = accel(om2, th);
    f64 k2t = f64_fma(k1w, hh, w),  k2w = accel(om2, f64_fma(k1t, hh, th));
    f64 k3t = f64_fma(k2w, hh, w),  k3w = accel(om2, f64_fma(k2t, hh, th));
    f64 k4t = f64_fma(k3w, h, w),   k4w = accel(om2, f64_fma(k3t, h, th));
    f64 two = F64_TWO;
    *thp = f64_fma(f64_add(f64_add(k1t, f64_mul(two, f64_add(k2t, k3t))), k4t), h6, th);
    *wp  = f64_fma(f64_add(f64_add(k1w, f64_mul(two, f64_add(k2w, k3w))), k4w), h6, w);
}

static void pendwave_reset(void *st_)
{
    pendwave_state *s = (pendwave_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->g = f64_dec("9.81");
    s->th0 = f64_dec("0.6");
    s->dt = f64_dec("0.01");
    s->dt_half = f64_mul(s->dt, F64_HALF);
    s->dt_sixth = f64_div(s->dt, f64_dec("6"));
    f64 pi = f64_dec("3.14159265358979323846264338327950288");
    f64 trec = f64_dec("30");

    /* F = (2/pi) K(sin(th0/2)), series in m = sin(th0/2)^2, m <= m^5.
     * Coefficients ((2n-1)!!/(2n)!!)^2: 1/4, 9/64, 25/256,
     * 1225/16384, 3969/65536 (all exact in binary). */
    f64 sn = f64_sin_rad(f64_mul(s->th0, F64_HALF));
    f64 m = f64_mul(sn, sn);
    f64 m2 = f64_mul(m, m);
    f64 corr = F64_ONE;
    corr = f64_fma(f64_dec("0.25"), m, corr);
    corr = f64_fma(f64_dec("0.140625"), m2, corr);
    corr = f64_fma(f64_dec("0.09765625"), f64_mul(m2, m), corr);
    corr = f64_fma(f64_dec("0.07476806640625"), f64_mul(m2, m2), corr);
    corr = f64_fma(f64_dec("0.0605621337890625"), f64_mul(m2, f64_mul(m2, m)), corr);
    s->corr = corr;

    f64 two_pi = f64_mul(F64_TWO, pi);
    f64 nf = f64_dec("20");
    f64 cycles = f64_dec("20"); /* base oscillations per T_rec */
    f64 kf = F64_ZERO;
    for (int k = 0; k < PW_N; k++) {
        f64 freq = f64_div(cycles, trec);
        f64 omega0 = f64_mul(corr, f64_mul(two_pi, freq));
        s->len[k] = f64_div(s->g, f64_mul(omega0, omega0));
        s->om2[k] = f64_div(s->g, s->len[k]);
        s->th[k] = s->th0;
        s->w[k] = F64_ZERO;
        /* Ring direction: cosPi/sinPi of 2k/N (angle in units of pi). */
        f64 arg = f64_div(f64_mul(F64_TWO, kf), nf);
        s->rx[k] = f64_cos_pi(arg);
        s->ry[k] = f64_sin_pi(arg);
        cycles = f64_add(cycles, F64_ONE);
        kf = f64_add(kf, F64_ONE);
    }
}

static void pendwave_step(void *st_)
{
    pendwave_state *s = (pendwave_state *)st_;
    for (int k = 0; k < PW_N; k++)
        rk4_one(s->om2[k], &s->th[k], &s->w[k], s->dt, s->dt_half, s->dt_sixth);
    s->steps++;
}

static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Six-sector rainbow by index; integer arithmetic only. */
static uint16_t pw_color(int k, int shift)
{
    int pos = (k * 1536) / PW_N;
    int sector = pos >> 8;
    unsigned f = (unsigned)(pos & 255);
    unsigned r = 0, g = 0, b = 0;
    switch (sector) {
    case 0:  r = 255u;    g = f;                       break;
    case 1:  r = 255u - f; g = 255u;                    break;
    case 2:  g = 255u;    b = f;                        break;
    case 3:  g = 255u - f; b = 255u;                    break;
    case 4:  r = f;       b = 255u;                     break;
    default: r = 255u;    b = 255u - f;                 break;
    }
    return toy_rgb565(r >> shift, g >> shift, b >> shift);
}

static void pendwave_render(const void *st_, toy_fb_t *fb)
{
    const pendwave_state *s = (const pendwave_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb);
    int R = toy_r(fb);
    double neutral = (double)R * 0.66;
    double amp = (double)R * 0.19;
    int bx[PW_N], by[PW_N];
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));
    toy_fb_ring(fb, cx, cy, (int)(neutral + amp), toy_rgb565(18, 24, 40));
    toy_fb_ring(fb, cx, cy, (int)neutral, toy_rgb565(34, 44, 66));
    toy_fb_ring(fb, cx, cy, (int)(neutral - amp), toy_rgb565(18, 24, 40));
    for (int k = 0; k < PW_N; k++) {
        double disp = d64(s->th[k]) / d64(s->th0); /* -1..1, display only */
        double radius = neutral + disp * amp;
        bx[k] = cx + (int)(d64(s->rx[k]) * radius);
        by[k] = cy - (int)(d64(s->ry[k]) * radius);
    }
    for (int k = 0; k < PW_N; k++)
        toy_fb_line(fb, cx, cy, bx[k], by[k], pw_color(k, 2));
    for (int k = 0; k < PW_N; k++) {
        int nxt = (k + 1) % PW_N;
        toy_fb_line(fb, bx[k], by[k], bx[nxt], by[nxt], pw_color(k, 1));
    }
    for (int k = 0; k < PW_N; k++) {
        toy_fb_disc(fb, bx[k], by[k], R / 40 + 2, pw_color(k, 0));
        toy_fb_disc(fb, bx[k], by[k], R / 100 + 1, 0xffffu);
    }
    int sc = R >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 15 * sc, cy - R + 10 * sc, sc, toy_rgb565(150, 170, 200), "PENDWAVE");
    /* Elapsed fraction of the current recurrence, centred. */
    unsigned pct = (unsigned)((s->steps % PW_REC_STEPS) * 100u / PW_REC_STEPS);
    char buf[6];
    buf[0] = 'T'; buf[1] = '+';
    buf[2] = (char)('0' + (int)(pct / 10u));
    buf[3] = (char)('0' + (int)(pct % 10u));
    buf[4] = '%'; buf[5] = '\0';
    toy_fb_text(fb, cx - 10 * sc, cy - 2 * sc, sc, toy_rgb565(190, 205, 230), buf);
}

const toy_cart_t toy_cart_pendwave = {
    "pendwave", "Pendulum Wave",
    "Twenty pendulums, 20 to 39 beats in 30 s: order, patterns, return.",
    sizeof(pendwave_state), 75, 3375, 375, pendwave_reset, pendwave_step, pendwave_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
