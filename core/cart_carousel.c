/* cart_carousel.c - a carousel of three-body choreographies.
 *
 * Three distinct periodic solutions of the equal-mass planar
 * three-body problem (G = m = 1) share one Yoshida-4 integrator
 * (kick-drift-kick leapfrog composition, fixed dt = 0.01, all
 * arithmetic via muse-softfloat binary64, exactly as cart_fig8.c).
 * The state carries the current choreography index and a step
 * timer; when the timer completes a full showing (two whole
 * periods), step() re-initialises positions, velocities and the
 * trail to the next choreography deterministically, with no host
 * involvement. The global step counter keeps counting across
 * showings, so the hash stream never repeats verbatim.
 *
 * Choreographies and sources:
 *
 * 0. Figure-8. Chenciner & Montgomery, "A remarkable periodic
 *    solution of the three-body problem in the case of equal
 *    masses", Annals of Mathematics 152 (2000), 881-901:
 *      x1 = -x2 = (0.97000436, -0.24308753), x3 = 0
 *      v3 = (0.93240737, 0.86473146), v1 = v2 = -v3/2.
 *    Period T ~= 6.3259 (633 steps). Same ICs as cart_fig8.c.
 *
 * 1. Lagrange equilateral. Lagrange, "Essai sur le probleme des
 *    trois corps" (1772): the equilateral homographic solution.
 *    Analytic ICs for side d = 1: circumradius R = 1/sqrt(3),
 *    omega^2 = 3*G*m/d^3 = 3 (net pull sqrt(3)*G*m^2/d^2 toward
 *    the centroid supplies m*omega^2*R), speed omega*R = 1.
 *    Bodies at angles 0/120/240 degrees on the circle of radius
 *    R with unit tangential velocities. Period 2*pi/sqrt(3)
 *    ~= 3.6276 (363 steps). The equal-mass equilateral solution
 *    is linearly unstable on long horizons (Gascheau's
 *    criterion), so the showing is deliberately short - two
 *    periods, verified bounded (side lengths stay 1 to ~1e-7)
 *    over that span - and the carousel then moves on.
 *
 * 2. Euler collinear. Euler, "De motu rectilineo trium corporum
 *    se mutuo attrahentium" (1767): the collinear homographic
 *    solution. Analytic ICs for equal masses at x = -1, 0, +1:
 *    each outer body is pulled toward the centre with
 *    G*m/1^2 + G*m/2^2 = 5/4, so omega^2 = 5/4 and the outer
 *    speed is omega = sqrt(5)/2; the central body sits at the
 *    barycentre with cancelling pulls. Period 4*pi/sqrt(5)
 *    ~= 5.6180 (562 steps).
 *
 * Constants are parsed from decimal strings at (re-)init via
 * f64_dec(), never typed as hex bit patterns.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define TRAIL 288
#define NCHOREO 3

typedef struct {
    f64 x[3][2], v[3][2];
    f64 w1, w0, dt;
    uint64_t steps; /* global, monotone across showings */
    uint64_t idx;   /* current choreography, 0..NCHOREO-1 */
    uint64_t timer; /* steps since the current showing began */
    f64 trail[3][TRAIL][2];
} carousel_state;

/* Two whole periods per showing, in steps at dt = 0.01:
 * fig-8 2*633, Lagrange 2*363, Euler 2*562. */
static uint64_t show_steps(uint64_t idx)
{
    if (idx == 0) return 1266u;
    if (idx == 1) return 726u;
    return 1124u;
}

static void accel(const carousel_state *s, f64 a[3][2])
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

static void leapfrog(carousel_state *s, f64 h)
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

/* (Re-)initialise the sub-state for choreography idx. Leaves the
 * integrator constants and the global step counter untouched. */
static void init_choreo(carousel_state *s, uint64_t idx)
{
    s->idx = idx;
    s->timer = 0;
    if (idx == 0) {
        /* Figure-8: Chenciner & Montgomery (2000), as above. */
        f64 px = f64_dec("0.97000436"), py = f64_dec("-0.24308753");
        f64 vx = f64_dec("0.93240737"), vy = f64_dec("0.86473146");
        s->x[0][0] = px;            s->x[0][1] = py;
        s->x[1][0] = f64_neg(px);   s->x[1][1] = f64_neg(py);
        s->x[2][0] = F64_ZERO;      s->x[2][1] = F64_ZERO;
        f64 hx = f64_mul(vx, F64_HALF), hy = f64_mul(vy, F64_HALF);
        s->v[0][0] = f64_neg(hx);   s->v[0][1] = f64_neg(hy);
        s->v[1][0] = f64_neg(hx);   s->v[1][1] = f64_neg(hy);
        s->v[2][0] = vx;            s->v[2][1] = vy;
    } else if (idx == 1) {
        /* Lagrange (1772) equilateral: R = 1/sqrt(3), speed 1.
         * Positions (R,0), (-R/2, +-1/2); R/2 = 1/(2*sqrt(3)),
         * tangential components sqrt(3)/2 and 1/2. */
        f64 r  = f64_dec("0.57735026918962576451");  /* 1/sqrt(3) */
        f64 hx = f64_dec("-0.28867513459481288225"); /* -1/(2*sqrt(3)) */
        f64 sq = f64_dec("0.86602540378443864676");  /* sqrt(3)/2 */
        s->x[0][0] = r;             s->x[0][1] = F64_ZERO;
        s->x[1][0] = hx;            s->x[1][1] = F64_HALF;
        s->x[2][0] = hx;            s->x[2][1] = f64_neg(F64_HALF);
        s->v[0][0] = F64_ZERO;      s->v[0][1] = F64_ONE;
        s->v[1][0] = f64_neg(sq);   s->v[1][1] = f64_neg(F64_HALF);
        s->v[2][0] = sq;            s->v[2][1] = f64_neg(F64_HALF);
    } else {
        /* Euler (1767) collinear: x = -1, 0, +1, outer speed
         * sqrt(5)/2 in +-y, central body at rest. */
        f64 om = f64_dec("1.11803398874989484820"); /* sqrt(5)/2 */
        s->x[0][0] = f64_neg(F64_ONE); s->x[0][1] = F64_ZERO;
        s->x[1][0] = F64_ZERO;         s->x[1][1] = F64_ZERO;
        s->x[2][0] = F64_ONE;          s->x[2][1] = F64_ZERO;
        s->v[0][0] = F64_ZERO;         s->v[0][1] = f64_neg(om);
        s->v[1][0] = F64_ZERO;         s->v[1][1] = F64_ZERO;
        s->v[2][0] = F64_ZERO;         s->v[2][1] = om;
    }
    for (int i = 0; i < 3; i++)
        for (int t = 0; t < TRAIL; t++) {
            s->trail[i][t][0] = s->x[i][0];
            s->trail[i][t][1] = s->x[i][1];
        }
}

static void carousel_reset(void *st_)
{
    carousel_state *s = (carousel_state *)st_;
    toy_memset(s, 0, sizeof *s);
    /* Yoshida weights: w1 = 1/(2 - 2^(1/3)), w0 = 1 - 2*w1. */
    f64 cbrt2 = f64_dec("1.2599210498948731647672106072782283505702514647");
    s->w1 = f64_div(F64_ONE, f64_sub(F64_TWO, cbrt2));
    s->w0 = f64_sub(F64_ONE, f64_mul(F64_TWO, s->w1));
    s->dt = f64_dec("0.01");
    init_choreo(s, 0);
}

static void carousel_step(void *st_)
{
    carousel_state *s = (carousel_state *)st_;
    leapfrog(s, f64_mul(s->w1, s->dt));
    leapfrog(s, f64_mul(s->w0, s->dt));
    leapfrog(s, f64_mul(s->w1, s->dt));
    s->steps++;
    s->timer++;
    if ((s->timer % 4u) == 0) {
        size_t slot = (size_t)((s->timer / 4u) % TRAIL);
        for (int i = 0; i < 3; i++) {
            s->trail[i][slot][0] = s->x[i][0];
            s->trail[i][slot][1] = s->x[i][1];
        }
    }
    /* Carousel advance: the showing is complete, so re-initialise
     * to the next choreography inside this same step. */
    if (s->timer >= show_steps(s->idx))
        init_choreo(s, (s->idx + 1u) % NCHOREO);
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

static void carousel_render(const void *st_, toy_fb_t *fb)
{
    const carousel_state *s = (const carousel_state *)st_;
    static const uint16_t body_col[3] = { 0x07ffu, 0xfc60u, 0xf81fu }; /* cyan amber magenta */
    static const char *const names[NCHOREO] = { "FIG-8", "LAGRANGE", "EULER" };
    static const double denom[NCHOREO] = { 1.35, 0.75, 1.20 }; /* display extent per orbit */
    uint64_t idx = s->idx < NCHOREO ? s->idx : 0;
    int cx = toy_cx(fb), cy = toy_cy(fb);
    double scale = (double)toy_r(fb) * 0.78 / denom[idx];
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, toy_r(fb) - 2, toy_rgb565(28, 34, 54));
    size_t head = (size_t)((s->timer / 4u) % TRAIL);
    int bx[3], by[3];
    for (int i = 0; i < 3; i++) {
        for (int t = 0; t < TRAIL; t++) {
            size_t ti = (head + TRAIL - (size_t)t) % TRAIL;
            int x = cx + (int)(d64(s->trail[i][ti][0]) * scale);
            int y = cy - (int)(d64(s->trail[i][ti][1]) * scale);
            unsigned fade = 255u - (unsigned)(t * 200 / TRAIL);
            unsigned p = body_col[i];
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        bx[i] = cx + (int)(d64(s->x[i][0]) * scale);
        by[i] = cy - (int)(d64(s->x[i][1]) * scale);
    }
    /* Join the bodies so the instantaneous shape (triangle for
     * Lagrange, a rotating line for Euler) reads at a glance. */
    uint16_t line_col = toy_rgb565(70, 85, 120);
    toy_fb_line(fb, bx[0], by[0], bx[1], by[1], line_col);
    toy_fb_line(fb, bx[1], by[1], bx[2], by[2], line_col);
    toy_fb_line(fb, bx[2], by[2], bx[0], by[0], line_col);
    for (int i = 0; i < 3; i++) {
        toy_fb_disc(fb, bx[i], by[i], toy_r(fb) / 40 + 2, body_col[i]);
        toy_fb_disc(fb, bx[i], by[i], toy_r(fb) / 80 + 1, 0xffffu);
    }
    int sc = toy_r(fb) >= 150 ? 2 : 1;
    const char *nm = names[idx];
    int len = 0;
    while (nm[len]) len++;
    int tw = len * 4 * sc - sc; /* 3px glyph + 1px gap, per char */
    toy_fb_text(fb, cx - tw / 2, cy - toy_r(fb) + 10 * sc, sc,
                toy_rgb565(150, 170, 200), nm);
    /* Which of the three is showing, e.g. "2/3", at the bottom. */
    char page[4];
    page[0] = (char)('1' + (int)idx);
    page[1] = '/';
    page[2] = (char)('0' + NCHOREO);
    page[3] = '\0';
    toy_fb_text(fb, cx - (3 * 4 * sc - sc) / 2, cy + toy_r(fb) - 16 * sc, sc,
                toy_rgb565(110, 125, 155), page);
}

const toy_cart_t toy_cart_carousel = {
    "carousel", "Choreography Carousel",
    "Figure-8, Lagrange triangle and Euler line, two periods each, in turn.",
    sizeof(carousel_state), 4, 4000, 500, carousel_reset, carousel_step, carousel_render, 0, 0, 0, 0, 0, 0, 0
};
