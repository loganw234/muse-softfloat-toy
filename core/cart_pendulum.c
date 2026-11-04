/* cart_pendulum.c - double-pendulum twins, one ulp apart.
 *
 * Classic double pendulum (m1=m2=L1=L2=1, g=9.81) under RK4 in
 * muse-softfloat binary64. Two instances run side by side; the
 * second's first angle is f64_next_up of the first's - the smallest
 * difference the format can express. They agree, then they do not:
 * chaos as an exact, reproducible measurement. Trig via sinPi/cosPi
 * (see toy_math.h); angles in radians.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define PTRAIL 220

typedef struct {
    f64 th[2][2]; /* [instance][link] angles */
    f64 w[2][2];  /* angular velocities */
    f64 dt, g;
    uint64_t steps;
    f64 tip[2][PTRAIL][2]; /* tip positions, unit-length space */
} pend_state;

typedef struct { f64 th1, th2, w1, w2; } deriv;

static deriv rhs(f64 g, f64 th1, f64 th2, f64 w1, f64 w2)
{
    /* m1=m2=L1=L2=1: standard textbook form. */
    f64 d = f64_sub(th1, th2);
    f64 sd = f64_sin_rad(d), cd = f64_cos_rad(d);
    f64 s1 = f64_sin_rad(th1), c1 = f64_cos_rad(th1);
    f64 s12 = f64_sin_rad(f64_sub(th1, f64_mul(F64_TWO, th2)));
    f64 den = f64_sub(f64_dec("3"), f64_cos_rad(f64_mul(F64_TWO, d))); /* 2m1+m2 - m2 cos2d, m=1 */
    /* a1 numerator: -g*3*sin(th1) - g*sin(th1-2th2) - 2 sd (w2^2 + w1^2 cd) */
    f64 n1 = f64_neg(f64_mul(f64_dec("3"), f64_mul(g, s1)));
    n1 = f64_sub(n1, f64_mul(g, s12));
    f64 inner1 = f64_fma(w1, f64_mul(w1, cd), f64_mul(w2, w2));
    n1 = f64_sub(n1, f64_mul(F64_TWO, f64_mul(sd, inner1)));
    /* a2 numerator: 2 sd (2 w1^2 + 2 g cos(th1) + w2^2 cd) */
    f64 inner2 = f64_mul(F64_TWO, f64_mul(w1, w1));
    inner2 = f64_fma(g, f64_mul(F64_TWO, c1), inner2);
    inner2 = f64_fma(w2, f64_mul(w2, cd), inner2);
    f64 n2 = f64_mul(F64_TWO, f64_mul(sd, inner2));
    deriv r = { w1, w2, f64_div(n1, den), f64_div(n2, den) };
    return r;
}

static void rk4(f64 g, f64 *th1, f64 *th2, f64 *w1, f64 *w2, f64 h)
{
    f64 hh = f64_mul(h, F64_HALF), h6 = f64_div(h, f64_dec("6"));
    deriv k1 = rhs(g, *th1, *th2, *w1, *w2);
    deriv k2 = rhs(g, f64_fma(k1.th1, hh, *th1), f64_fma(k1.th2, hh, *th2),
                   f64_fma(k1.w1, hh, *w1), f64_fma(k1.w2, hh, *w2));
    deriv k3 = rhs(g, f64_fma(k2.th1, hh, *th1), f64_fma(k2.th2, hh, *th2),
                   f64_fma(k2.w1, hh, *w1), f64_fma(k2.w2, hh, *w2));
    deriv k4 = rhs(g, f64_fma(k3.th1, h, *th1), f64_fma(k3.th2, h, *th2),
                   f64_fma(k3.w1, h, *w1), f64_fma(k3.w2, h, *w2));
    f64 two = F64_TWO;
    *th1 = f64_fma(f64_add(f64_add(k1.th1, f64_mul(two, f64_add(k2.th1, k3.th1))), k4.th1), h6, *th1);
    *th2 = f64_fma(f64_add(f64_add(k1.th2, f64_mul(two, f64_add(k2.th2, k3.th2))), k4.th2), h6, *th2);
    *w1  = f64_fma(f64_add(f64_add(k1.w1,  f64_mul(two, f64_add(k2.w1,  k3.w1))),  k4.w1),  h6, *w1);
    *w2  = f64_fma(f64_add(f64_add(k1.w2,  f64_mul(two, f64_add(k2.w2,  k3.w2))),  k4.w2),  h6, *w2);
}

static void tips(const pend_state *s, int inst, f64 *x1, f64 *y1, f64 *x2, f64 *y2)
{
    f64 th1 = s->th[inst][0], th2 = s->th[inst][1];
    *x1 = f64_sin_rad(th1);
    *y1 = f64_neg(f64_cos_rad(th1));
    *x2 = f64_add(*x1, f64_sin_rad(th2));
    *y2 = f64_sub(*y1, f64_cos_rad(th2));
}

static void pend_reset(void *st_)
{
    pend_state *s = (pend_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->th[0][0] = f64_dec("2.0");
    s->th[0][1] = f64_dec("2.4");
    s->th[1][0] = f64_next_up(s->th[0][0]); /* one ulp later, chaos */
    s->th[1][1] = s->th[0][1];
    s->dt = f64_dec("0.004");
    s->g = f64_dec("9.81");
    for (int i = 0; i < 2; i++) {
        f64 x1, y1, x2, y2;
        tips(s, i, &x1, &y1, &x2, &y2);
        for (int t = 0; t < PTRAIL; t++) { s->tip[i][t][0] = x2; s->tip[i][t][1] = y2; }
    }
}

static void pend_step(void *st_)
{
    pend_state *s = (pend_state *)st_;
    for (int sub = 0; sub < 2; sub++)
        for (int i = 0; i < 2; i++)
            rk4(s->g, &s->th[i][0], &s->th[i][1], &s->w[i][0], &s->w[i][1], s->dt);
    s->steps++;
    size_t slot = (size_t)(s->steps % PTRAIL);
    for (int i = 0; i < 2; i++) {
        f64 x1, y1, x2, y2;
        tips(s, i, &x1, &y1, &x2, &y2);
        s->tip[i][slot][0] = x2;
        s->tip[i][slot][1] = y2;
    }
}

static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

static void pend_render(const void *st_, toy_fb_t *fb)
{
    const pend_state *s = (const pend_state *)st_;
    static const uint16_t cols[2] = { 0x07ffu, 0xfc60u };
    int cx = toy_cx(fb), cy = toy_cy(fb);
    double scale = (double)toy_r(fb) * 0.44; /* arm length 1 -> 0.44 R */
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, toy_r(fb) - 2, toy_rgb565(28, 34, 54));
    size_t head = (size_t)(s->steps % PTRAIL);
    for (int i = 0; i < 2; i++) {
        for (int t = 0; t < PTRAIL; t++) {
            size_t idx = (head + PTRAIL - (size_t)t) % PTRAIL;
            int x = cx + (int)(d64(s->tip[i][idx][0]) * scale);
            int y = cy - (int)(d64(s->tip[i][idx][1]) * scale);
            unsigned fade = 255u - (unsigned)(t * 210 / PTRAIL);
            unsigned p = cols[i];
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        f64 x1, y1, x2, y2;
        tips(s, i, &x1, &y1, &x2, &y2);
        int ax = cx + (int)(d64(x1) * scale), ay = cy - (int)(d64(y1) * scale);
        int bx = cx + (int)(d64(x2) * scale), by = cy - (int)(d64(y2) * scale);
        toy_fb_line(fb, cx, cy, ax, ay, cols[i]);
        toy_fb_line(fb, ax, ay, bx, by, cols[i]);
        toy_fb_disc(fb, ax, ay, toy_r(fb) / 60 + 1, cols[i]);
        toy_fb_disc(fb, bx, by, toy_r(fb) / 45 + 2, cols[i]);
    }
    toy_fb_disc(fb, cx, cy, toy_r(fb) / 70 + 1, 0xffffu);
    int sc = toy_r(fb) >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 20 * sc, cy - toy_r(fb) + 10 * sc, sc, toy_rgb565(150, 170, 200), "TWINS 1ULP");
}

const toy_cart_t toy_cart_pendulum = {
    "pendulum", "Twins, 1 ulp apart",
    "Two double pendulums differing by one ulp. They agree, then they do not.",
    sizeof(pend_state), 30, 1000, 250, pend_reset, pend_step, pend_render
};
