/* cart_cloud.c - twelve figure-8s, a few ulps apart.
 *
 * NCOPY copies of the Chenciner-Montgomery figure-8 choreography
 * (same ICs, G = m = 1, and Yoshida 4th-order kick-drift-kick
 * leapfrog at fixed dt = 0.01 as cart_fig8.c; ICs as published by
 * Chenciner & Montgomery, Annals of Mathematics 152 (2000):
 * x1 = -x2 = (0.97000436, -0.24308753), x3 = 0,
 * v3 = (0.93240737, 0.86473146), v1 = v2 = -v3/2; period ~6.3259).
 *
 * Copy 0 is the unperturbed reference. Copy k (1..NCOPY-1) is
 * identical except that body 0's x-coordinate (+0.97000436) is
 * stepped up k times with f64_next_up: a perturbation of exactly
 * k ulps in one initial coordinate and nothing else (the resulting
 * O(k ulp) centre-of-mass/energy offset is part of the experiment).
 * The figure-8 is a near-stable choreography, so the cloud does
 * not explode like the double pendulum: it lets go slowly. The
 * separation diagnostic in state (softfloat, updated every step)
 * is the honest show while the spread is still far sub-pixel:
 * sep[k] is the RMS distance over the 3 bodies between copy k and
 * the reference, sep_max its maximum over the cloud. The renderer
 * prints sep_max as text and magnifies body 0's cloud in an inset
 * whose half-width is a fixed 5e-12 orbit units at any resolution.
 *
 * Descriptor choices (see parcel report for measurements):
 * demo 4 steps/frame matches fig8's screen speed; proof 1000
 * steps (~1.6 periods) already grows sep_max ~x244 from its
 * 7e-16 start. (As landed the parcel used 6000; the lead
 * trimmed it at merge after measuring 434 ms/step on the
 * RP2040 - 6000 steps is a ~43-minute board proof, 1000 is
 * ~7 minutes - with per-step maths unchanged.)
 * All state arithmetic is muse-softfloat binary64 via toy_math.h,
 * so every copy is bit-identical on host, board and WASM.
 * Textbook integrator only; no third-party code.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define NCOPY 12
#define CTRAIL 96

typedef struct {
    f64 x[NCOPY][3][2], v[NCOPY][3][2];
    f64 w1, w0, dt, three;
    uint64_t steps;
    f64 sep[NCOPY]; /* RMS body distance to reference; sep[0] = 0 */
    f64 sep_max;    /* max of sep[k] over k */
    f64 trail[NCOPY][3][CTRAIL][2];
} cloud_state;

static void accel(const cloud_state *s, int c, f64 a[3][2])
{
    for (int i = 0; i < 3; i++) a[i][0] = a[i][1] = F64_ZERO;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (j == i) continue;
            f64 dx = f64_sub(s->x[c][j][0], s->x[c][i][0]);
            f64 dy = f64_sub(s->x[c][j][1], s->x[c][i][1]);
            f64 r2 = f64_fma(dx, dx, f64_mul(dy, dy));
            f64 inv = f64_div(F64_ONE, f64_mul(r2, f64_sqrt(r2))); /* 1/r^3, m=1 */
            a[i][0] = f64_fma(dx, inv, a[i][0]);
            a[i][1] = f64_fma(dy, inv, a[i][1]);
        }
    }
}

static void leapfrog(cloud_state *s, int c, f64 h)
{
    f64 a[3][2];
    f64 hh = f64_mul(h, F64_HALF);
    accel(s, c, a);
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 2; k++) s->v[c][i][k] = f64_fma(a[i][k], hh, s->v[c][i][k]);
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 2; k++) s->x[c][i][k] = f64_fma(s->v[c][i][k], h, s->x[c][i][k]);
    accel(s, c, a);
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 2; k++) s->v[c][i][k] = f64_fma(a[i][k], hh, s->v[c][i][k]);
}

/* Separation diagnostic, softfloat only. Max uses bit-pattern
 * order: IEEE-754 non-negative values order like their uint64
 * bits, and every sep[k] is a sqrt, hence >= +0. */
static void compute_sep(cloud_state *s)
{
    s->sep[0] = F64_ZERO;
    f64 best = F64_ZERO;
    for (int c = 1; c < NCOPY; c++) {
        f64 sum = F64_ZERO;
        for (int i = 0; i < 3; i++) {
            f64 dx = f64_sub(s->x[c][i][0], s->x[0][i][0]);
            f64 dy = f64_sub(s->x[c][i][1], s->x[0][i][1]);
            sum = f64_add(sum, f64_fma(dx, dx, f64_mul(dy, dy)));
        }
        s->sep[c] = f64_sqrt(f64_div(sum, s->three));
        if (s->sep[c] > best) best = s->sep[c];
    }
    s->sep_max = best;
}

static void cloud_reset(void *st_)
{
    cloud_state *s = (cloud_state *)st_;
    toy_memset(s, 0, sizeof *s);
    f64 px = f64_dec("0.97000436"), py = f64_dec("-0.24308753");
    f64 vx = f64_dec("0.93240737"), vy = f64_dec("0.86473146");
    f64 hx = f64_mul(vx, F64_HALF), hy = f64_mul(vy, F64_HALF);
    for (int c = 0; c < NCOPY; c++) {
        s->x[c][0][0] = px;            s->x[c][0][1] = py;
        s->x[c][1][0] = f64_neg(px);   s->x[c][1][1] = f64_neg(py);
        s->x[c][2][0] = F64_ZERO;      s->x[c][2][1] = F64_ZERO;
        s->v[c][0][0] = f64_neg(hx);   s->v[c][0][1] = f64_neg(hy);
        s->v[c][1][0] = f64_neg(hx);   s->v[c][1][1] = f64_neg(hy);
        s->v[c][2][0] = vx;            s->v[c][2][1] = vy;
        /* The whole experiment: body 0's x, up exactly c ulps. */
        for (int k = 0; k < c; k++)
            s->x[c][0][0] = f64_next_up(s->x[c][0][0]);
    }
    /* Yoshida weights: w1 = 1/(2 - 2^(1/3)), w0 = 1 - 2*w1. */
    f64 cbrt2 = f64_dec("1.2599210498948731647672106072782283505702514647");
    s->w1 = f64_div(F64_ONE, f64_sub(F64_TWO, cbrt2));
    s->w0 = f64_sub(F64_ONE, f64_mul(F64_TWO, s->w1));
    s->dt = f64_dec("0.01");
    s->three = f64_dec("3");
    for (int c = 0; c < NCOPY; c++)
        for (int i = 0; i < 3; i++)
            for (int t = 0; t < CTRAIL; t++) {
                s->trail[c][i][t][0] = s->x[c][i][0];
                s->trail[c][i][t][1] = s->x[c][i][1];
            }
    compute_sep(s);
}

static void cloud_step(void *st_)
{
    cloud_state *s = (cloud_state *)st_;
    for (int c = 0; c < NCOPY; c++) {
        leapfrog(s, c, f64_mul(s->w1, s->dt));
        leapfrog(s, c, f64_mul(s->w0, s->dt));
        leapfrog(s, c, f64_mul(s->w1, s->dt));
    }
    s->steps++;
    if ((s->steps % 4u) == 0) {
        size_t slot = (size_t)((s->steps / 4u) % CTRAIL);
        for (int c = 0; c < NCOPY; c++)
            for (int i = 0; i < 3; i++) {
                s->trail[c][i][slot][0] = s->x[c][i][0];
                s->trail[c][i][slot][1] = s->x[c][i][1];
            }
    }
    compute_sep(s);
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Colour by perturbation size: reference (c=0) plain white, the
 * rest a bright-cyan to dim-violet ramp of falling brightness. */
static uint16_t copy_col(int c)
{
    if (c == 0) return 0xffffu;
    int t = c - 1, n = NCOPY - 2; /* 0..n across perturbed copies */
    unsigned r = (unsigned)(70 + (60 - 70) * t / n);
    unsigned g = (unsigned)(225 + (50 - 225) * t / n);
    unsigned b = (unsigned)(255 + (150 - 255) * t / n);
    return toy_rgb565(r, g, b);
}

static uint16_t dim565(uint16_t p, unsigned fade)
{
    return (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                      ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                      (((p & 31u) * fade) >> 8));
}

/* "SEP 1.3E-12" from the softfloat diagnostic; native double and
 * integer digit work here are display-only render code. */
static int sep_text(char *buf, f64 sep)
{
    int n = 0;
    buf[n++] = 'S'; buf[n++] = 'E'; buf[n++] = 'P'; buf[n++] = ' ';
    double v = d64(sep);
    if (!(v > 0.0)) { buf[n++] = '0'; buf[n] = '\0'; return n; }
    int e = 0;
    while (v >= 10.0 && e < 400) { v /= 10.0; e++; }
    while (v < 1.0 && e > -400) { v *= 10.0; e--; }
    int d1 = (int)v;
    int d2 = (int)((v - (double)d1) * 10.0);
    if (d2 < 0) d2 = 0;
    if (d2 > 9) d2 = 9;
    buf[n++] = (char)('0' + d1);
    buf[n++] = '.';
    buf[n++] = (char)('0' + d2);
    buf[n++] = 'E';
    int ae = e;
    if (ae < 0) { buf[n++] = '-'; ae = -ae; }
    if (ae >= 100) buf[n++] = (char)('0' + ae / 100);
    if (ae >= 10) buf[n++] = (char)('0' + (ae / 10) % 10);
    buf[n++] = (char)('0' + ae % 10);
    buf[n] = '\0';
    return n;
}

static void cloud_render(const void *st_, toy_fb_t *fb)
{
    const cloud_state *s = (const cloud_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    double scale = (double)r * 0.78 / 1.35; /* as fig8 */
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));
    size_t head = (size_t)((s->steps / 4u) % CTRAIL);
    /* Dimmest (largest perturbation) first, reference last. */
    for (int c = NCOPY - 1; c >= 0; c--) {
        uint16_t col = copy_col(c);
        for (int i = 0; i < 3; i++) {
            for (int t = 0; t < CTRAIL; t++) {
                size_t idx = (head + CTRAIL - (size_t)t) % CTRAIL;
                int x = cx + (int)(d64(s->trail[c][i][idx][0]) * scale);
                int y = cy - (int)(d64(s->trail[c][i][idx][1]) * scale);
                unsigned fade = 255u - (unsigned)(t * 200 / CTRAIL);
                toy_fb_px(fb, x, y, dim565(col, fade));
            }
            int x = cx + (int)(d64(s->x[c][i][0]) * scale);
            int y = cy - (int)(d64(s->x[c][i][1]) * scale);
            if (c == 0) {
                toy_fb_disc(fb, x, y, r / 40 + 2, col);
            } else {
                toy_fb_disc(fb, x, y, r / 60 + 1, col);
            }
        }
    }
    /* Magnifier: body 0 of every copy relative to the reference
     * body 0 (fixed screen spot, half-width 5e-12 orbit units). */
    int icx = cx, icy = cy + (r * 58) / 100, ir = r / 5;
    toy_fb_disc(fb, icx, icy, ir, toy_rgb565(10, 14, 28));
    toy_fb_ring(fb, icx, icy, ir, toy_rgb565(90, 110, 150));
    double mag = (double)ir / 5e-12;
    double rx = d64(s->x[0][0][0]), ry = d64(s->x[0][0][1]);
    for (int c = NCOPY - 1; c >= 1; c--) {
        double fx = (d64(s->x[c][0][0]) - rx) * mag;
        double fy = (d64(s->x[c][0][1]) - ry) * mag;
        if (!(fx >= -(double)ir && fx <= (double)ir &&
              fy >= -(double)ir && fy <= (double)ir))
            continue; /* outside the glass (also guards the casts) */
        int dx = (int)fx, dy = (int)fy;
        if (dx * dx + dy * dy <= (ir - 1) * (ir - 1))
            toy_fb_disc(fb, icx + dx, icy - dy, ir / 24 + 1, copy_col(c));
    }
    toy_fb_disc(fb, icx, icy, ir / 24 + 1, 0xffffu); /* reference body 0 */
    int sc = r >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 10 * sc, cy - r + 10 * sc, sc, toy_rgb565(150, 170, 200), "CLOUD");
    char buf[20];
    int len = sep_text(buf, s->sep_max);
    toy_fb_text(fb, cx - (len * 4 * sc) / 2, cy + r - 16 * sc, sc,
                toy_rgb565(150, 170, 200), buf);
}

const toy_cart_t toy_cart_cloud = {
    "cloud", "A Few Ulps of Weather",
    "12 figure-8s a few ulps apart, white is exact. Watch SEP grow.",
    sizeof(cloud_state), 4, 1000, 250, cloud_reset, cloud_step, cloud_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
