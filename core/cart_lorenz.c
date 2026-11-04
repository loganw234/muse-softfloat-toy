/* cart_lorenz.c - the Lorenz attractor, with a one-ulp twin.
 *
 * Lorenz, "Deterministic Nonperiodic Flow", J. Atmos. Sci. 20
 * (1963), at the standard chaotic parameters sigma=10, rho=28,
 * beta=8/3 (decimal digit string, correctly rounded by f64_dec):
 *   dx/dt = sigma*(y - x)
 *   dy/dt = x*(rho - z) - y
 *   dz/dt = x*y - beta*z
 * Two trajectories start 1 ulp apart (twin B's x is f64_next_up
 * of twin A's) and are integrated with fixed-step RK4, dt=0.005,
 * entirely in muse-softfloat binary64. The start point (5,5,20)
 * sits in the attractor's band (wing centres are at z=27), so the
 * transient is short. A 1-ulp twin does not survive from every
 * start in this strongly contracting flow - from (1,1,1), say,
 * both states round to identical bits within a few steps - so
 * survival here was verified by direct measurement: the twins
 * stay bit-close for ~6.3k steps and then separate to O(1)
 * within a few dozen steps (largest Lyapunov exponent ~0.9).
 *
 * The 3D trail of both twins is simulation history and lives in
 * state. The projection azimuth also lives in state (in cycles,
 * advanced by softfloat addition in step, wrapped below 1); only
 * its sine/cosine for placing pixels is computed at draw time,
 * display-only, and never flows back into state.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

/* Trail slots per twin. Size arithmetic (all f64/uint64_t, so no
 * padding): header 14 f64 + 1 uint64_t = 120 bytes; trail costs
 * 2 twins * 3 coords * 8 bytes = 48 bytes per slot, so the
 * 64 KiB cap admits floor((65536-120)/48) = 1362 slots. 1280
 * leaves headroom: state is 120 + 1280*48 = 61560 bytes.
 * Stride 2 steps at dt=0.005: the trail spans 12.8 time units,
 * many wing circuits, so the full butterfly is on screen. */
#define LTRAIL 1280

typedef struct {
    f64 p[2][3];                 /* [twin][x,y,z] */
    f64 sigma, rho, beta;
    f64 dt, hh, h6;              /* dt, dt/2, dt/6 (precomputed) */
    f64 rot, dphi;               /* view azimuth, cycles + /step */
    uint64_t steps;
    f64 trail[2][LTRAIL][3];
} lorenz_state;

static void rhs(const lorenz_state *s, f64 x, f64 y, f64 z, f64 o[3])
{
    o[0] = f64_mul(s->sigma, f64_sub(y, x));
    o[1] = f64_fma(x, f64_sub(s->rho, z), f64_neg(y));
    o[2] = f64_fma(f64_neg(s->beta), z, f64_mul(x, y));
}

static void rk4(const lorenz_state *s, f64 p[3])
{
    f64 k1[3], k2[3], k3[3], k4[3], t[3];
    rhs(s, p[0], p[1], p[2], k1);
    for (int i = 0; i < 3; i++) t[i] = f64_fma(k1[i], s->hh, p[i]);
    rhs(s, t[0], t[1], t[2], k2);
    for (int i = 0; i < 3; i++) t[i] = f64_fma(k2[i], s->hh, p[i]);
    rhs(s, t[0], t[1], t[2], k3);
    for (int i = 0; i < 3; i++) t[i] = f64_fma(k3[i], s->dt, p[i]);
    rhs(s, t[0], t[1], t[2], k4);
    for (int i = 0; i < 3; i++)
        p[i] = f64_fma(f64_add(f64_add(k1[i], f64_mul(F64_TWO, f64_add(k2[i], k3[i]))), k4[i]), s->h6, p[i]);
}

static void lorenz_reset(void *st_)
{
    lorenz_state *s = (lorenz_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->sigma = f64_dec("10");
    s->rho = f64_dec("28");
    s->beta = f64_dec("2.6666666666666666666666666666666666666666"); /* 8/3 */
    s->dt = f64_dec("0.005");
    s->hh = f64_mul(s->dt, F64_HALF);
    s->h6 = f64_div(s->dt, f64_dec("6"));
    /* One full view rotation per 16000 steps (80 time units). */
    s->dphi = f64_dec("0.0000625");
    s->rot = F64_ZERO;
    s->p[0][0] = f64_dec("5");
    s->p[0][1] = f64_dec("5");
    s->p[0][2] = f64_dec("20");
    s->p[1][0] = f64_next_up(s->p[0][0]); /* one ulp later, chaos */
    s->p[1][1] = s->p[0][1];
    s->p[1][2] = s->p[0][2];
    for (int i = 0; i < 2; i++)
        for (int t = 0; t < LTRAIL; t++)
            for (int k = 0; k < 3; k++)
                s->trail[i][t][k] = s->p[i][k];
}

static void lorenz_step(void *st_)
{
    lorenz_state *s = (lorenz_state *)st_;
    rk4(s, s->p[0]);
    rk4(s, s->p[1]);
    s->steps++;
    /* Azimuth is state: advance in cycles, wrap below 1. rot is
     * never negative, so the f64 bit patterns order like values. */
    s->rot = f64_add(s->rot, s->dphi);
    if (s->rot >= F64_ONE) s->rot = f64_sub(s->rot, F64_ONE);
    if ((s->steps % 2u) == 0) {
        size_t slot = (size_t)((s->steps / 2u) % LTRAIL);
        for (int i = 0; i < 2; i++)
            for (int k = 0; k < 3; k++)
                s->trail[i][slot][k] = s->p[i][k];
    }
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Project a state-space point: rotate (x,y) about the attractor's
 * vertical (z) axis by the stored azimuth, then a fixed display
 * tilt so depth reads as height. z is centred on 25 (attractor
 * spans roughly x +-20, y +-28, z 0..50). Native double only
 * places already-computed state on screen. */
static void project(f64 fx, f64 fy, f64 fz, double cu, double su,
                    double scale, int cx, int cy, int *px, int *py)
{
    double x = d64(fx), y = d64(fy), z = d64(fz) - 25.0;
    double xr = x * cu - y * su;
    double yr = x * su + y * cu;
    double up = z * 0.80 - yr * 0.60; /* fixed tilt, ~37 degrees */
    *px = cx + (int)(xr * scale);
    *py = cy - (int)(up * scale);
}

static void lorenz_render(const void *st_, toy_fb_t *fb)
{
    const lorenz_state *s = (const lorenz_state *)st_;
    static const uint16_t cols[2] = { 0x07ffu, 0xfc60u }; /* cyan, amber */
    int cx = toy_cx(fb), cy = toy_cy(fb);
    double scale = (double)toy_r(fb) * 0.80 / 27.0;
    /* Display-only trig on the stored azimuth: sin/cos(2*pi*rot)
     * via the library's sinPi/cosPi on 2*rot. */
    f64 two_rot = f64_mul(s->rot, F64_TWO);
    double su = d64(f64_sin_pi(two_rot));
    double cu = d64(f64_cos_pi(two_rot));
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, toy_r(fb) - 2, toy_rgb565(28, 34, 54));
    size_t head = (size_t)((s->steps / 2u) % LTRAIL);
    for (int i = 0; i < 2; i++) {
        int px = 0, py = 0;
        for (int age = LTRAIL - 1; age >= 0; age--) {
            size_t idx = (head + LTRAIL - (size_t)age) % LTRAIL;
            int x, y;
            project(s->trail[i][idx][0], s->trail[i][idx][1], s->trail[i][idx][2],
                    cu, su, scale, cx, cy, &x, &y);
            if (age < LTRAIL - 1) {
                unsigned fade = 255u - (unsigned)(age * 200 / LTRAIL);
                unsigned p = cols[i];
                uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                        ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                        (((p & 31u) * fade) >> 8));
                toy_fb_line(fb, px, py, x, y, c);
            }
            px = x; py = y;
        }
        int hx, hy;
        project(s->p[i][0], s->p[i][1], s->p[i][2], cu, su, scale, cx, cy, &hx, &hy);
        toy_fb_disc(fb, hx, hy, toy_r(fb) / 50 + 2, cols[i]);
        toy_fb_disc(fb, hx, hy, toy_r(fb) / 100 + 1, 0xffffu);
    }
    int sc = toy_r(fb) >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 12 * sc, cy - toy_r(fb) + 10 * sc, sc, toy_rgb565(150, 170, 200), "LORENZ");
}

const toy_cart_t toy_cart_lorenz = {
    "lorenz", "The Butterfly",
    "Two Lorenz trajectories one ulp apart in a slowly turning view. They agree, then they do not.",
    sizeof(lorenz_state), 20, 20000, 2000, lorenz_reset, lorenz_step, lorenz_render
};
