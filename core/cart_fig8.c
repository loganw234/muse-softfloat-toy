/* cart_fig8.c - the Chenciner-Montgomery figure-8 choreography.
 *
 * Three equal masses chase each other along one figure-8 curve.
 * Initial conditions as published by Chenciner & Montgomery,
 * "A remarkable periodic solution of the three-body problem in the
 * case of equal masses", Annals of Mathematics 152 (2000):
 *   x1 = -x2 = (0.97000436, -0.24308753), x3 = 0
 *   v3 = (0.93240737, 0.86473146), v1 = v2 = -v3/2, G = m = 1.
 * Period T ~= 6.3259. Integrator: Yoshida 4th-order composition of
 * kick-drift-kick leapfrog at fixed dt = 0.01, all arithmetic via
 * muse-softfloat binary64, so the orbit is bit-identical on every
 * host and board. Constants are parsed from decimal strings at
 * reset (correctly rounded by the library), never typed as hex.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define TRAIL 288

typedef struct {
    f64 x[3][2], v[3][2];
    f64 w1, w0, dt;
    uint64_t steps;
    f64 trail[3][TRAIL][2];
    uint64_t tperiods; /* whole periods completed (display only) */
} fig8_state;

static void accel(const fig8_state *s, f64 a[3][2])
{
    for (int i = 0; i < 3; i++) a[i][0] = a[i][1] = F64_ZERO;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (j == i) continue;
            f64 dx = f64_sub(s->x[j][0], s->x[i][0]);
            f64 dy = f64_sub(s->x[j][1], s->x[i][1]);
            f64 r2 = f64_fma(dx, dx, f64_mul(dy, dy));
            f64 inv = f64_div(F64_ONE, f64_mul(r2, f64_sqrt(r2))); /* 1/r^3, m=1 */
            a[i][0] = f64_fma(dx, inv, a[i][0]);
            a[i][1] = f64_fma(dy, inv, a[i][1]);
        }
    }
}

static void leapfrog(fig8_state *s, f64 h)
{
    f64 a[3][2];
    f64 hh = f64_mul(h, F64_HALF);
    accel(s, a);
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 2; k++) s->v[i][k] = f64_fma(a[i][k], hh, s->v[i][k]);
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 2; k++) s->x[i][k] = f64_fma(s->v[i][k], h, s->x[i][k]);
    accel(s, a);
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 2; k++) s->v[i][k] = f64_fma(a[i][k], hh, s->v[i][k]);
}

static void fig8_reset(void *st_)
{
    fig8_state *s = (fig8_state *)st_;
    toy_memset(s, 0, sizeof *s);
    f64 px = f64_dec("0.97000436"), py = f64_dec("-0.24308753");
    f64 vx = f64_dec("0.93240737"), vy = f64_dec("0.86473146");
    s->x[0][0] = px;            s->x[0][1] = py;
    s->x[1][0] = f64_neg(px);   s->x[1][1] = f64_neg(py);
    s->x[2][0] = F64_ZERO;      s->x[2][1] = F64_ZERO;
    f64 hx = f64_mul(vx, F64_HALF), hy = f64_mul(vy, F64_HALF);
    s->v[0][0] = f64_neg(hx);   s->v[0][1] = f64_neg(hy);
    s->v[1][0] = f64_neg(hx);   s->v[1][1] = f64_neg(hy);
    s->v[2][0] = vx;            s->v[2][1] = vy;
    /* Yoshida weights: w1 = 1/(2 - 2^(1/3)), w0 = 1 - 2*w1. */
    f64 cbrt2 = f64_dec("1.2599210498948731647672106072782283505702514647");
    s->w1 = f64_div(F64_ONE, f64_sub(F64_TWO, cbrt2));
    s->w0 = f64_sub(F64_ONE, f64_mul(F64_TWO, s->w1));
    s->dt = f64_dec("0.01");
    for (int i = 0; i < 3; i++)
        for (int t = 0; t < TRAIL; t++) {
            s->trail[i][t][0] = s->x[i][0];
            s->trail[i][t][1] = s->x[i][1];
        }
}

static void fig8_step(void *st_)
{
    fig8_state *s = (fig8_state *)st_;
    leapfrog(s, f64_mul(s->w1, s->dt));
    leapfrog(s, f64_mul(s->w0, s->dt));
    leapfrog(s, f64_mul(s->w1, s->dt));
    s->steps++;
    if ((s->steps % 4u) == 0) {
        size_t slot = (size_t)((s->steps / 4u) % TRAIL);
        for (int i = 0; i < 3; i++) {
            s->trail[i][slot][0] = s->x[i][0];
            s->trail[i][slot][1] = s->x[i][1];
        }
    }
    /* Display-only period counter: 633 steps ~ one period at dt=0.01. */
    s->tperiods = s->steps / 633u;
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

static void fig8_render(const void *st_, toy_fb_t *fb)
{
    const fig8_state *s = (const fig8_state *)st_;
    static const uint16_t body_col[3] = { 0x07ffu, 0xfc60u, 0xf81fu }; /* cyan amber magenta */
    int cx = toy_cx(fb), cy = toy_cy(fb);
    double scale = (double)toy_r(fb) * 0.78 / 1.35;
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, toy_r(fb) - 2, toy_rgb565(28, 34, 54));
    size_t head = (size_t)((s->steps / 4u) % TRAIL);
    for (int i = 0; i < 3; i++) {
        for (int t = 0; t < TRAIL; t++) {
            size_t idx = (head + TRAIL - (size_t)t) % TRAIL;
            int x = cx + (int)(d64(s->trail[i][idx][0]) * scale);
            int y = cy - (int)(d64(s->trail[i][idx][1]) * scale);
            unsigned fade = 255u - (unsigned)(t * 200 / TRAIL);
            unsigned p = body_col[i];
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        int x = cx + (int)(d64(s->x[i][0]) * scale);
        int y = cy - (int)(d64(s->x[i][1]) * scale);
        toy_fb_disc(fb, x, y, toy_r(fb) / 40 + 2, body_col[i]);
        toy_fb_disc(fb, x, y, toy_r(fb) / 80 + 1, 0xffffu);
    }
    int sc = toy_r(fb) >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 8 * sc, cy - toy_r(fb) + 10 * sc, sc, toy_rgb565(150, 170, 200), "FIG-8");
}

const toy_cart_t toy_cart_fig8 = {
    "fig8", "Eternal Return",
    "Three equal masses chase one curve forever. Watch the return.",
    sizeof(fig8_state), 4, 2000, 500, fig8_reset, fig8_step, fig8_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
