/* cart_orrery.c - Keplerian solar-system orrery clock.
 *
 * Eight planets from the published approximate Keplerian elements
 * of E. M. Standish & J. G. Williams, as tabulated by JPL SSD at
 * https://ssd.jpl.nasa.gov/planets/approx_pos.html (Table 1:
 * elements and rates with respect to the mean ecliptic and equinox
 * of J2000, epoch J2000.0, valid 1800 AD - 2050 AD). "Earth" here
 * is JPL's EM Bary (Earth/Moon barycentre), as in that table.
 * These are best-fit lower-accuracy elements, not an integrated
 * ephemeris: positions are approximate by design.
 *
 * State is elapsed days since J2000.0 plus per-planet positions
 * recomputed every step in muse-softfloat binary64, so the proof
 * hash covers the maths. Each step is a fixed 0.5 day. Elements
 * evolve as elem0 + rate * T, T in Julian centuries past J2000.
 * Mean anomaly is carried as a wrapped phase in cycles rather
 * than as L - varpi in degrees, so its argument never winds
 * without bound; the phase increment is (Ldot - varpidot), which
 * is the same angle modulo one turn.
 *
 * Kepler's equation is solved in cycles,
 *   E - (e / 2pi) * sinPi(2E) = M,
 * by Newton iteration with a FIXED count of 5 iterations from
 * E0 = M + (e / 2pi) * sinPi(2M) (JPL's suggested start). No
 * convergence test, so step() does bounded, data-independent
 * work. Five iterations is ample for these eccentricities
 * (largest e = 0.2056, Mercury): Newton converges quadratically
 * and the stored residual stays at rounding level (measured in
 * the parcel report over the proof run).
 *
 * Render is a top-down ecliptic view. Heliocentric distances span
 * 0.31 - 30.3 au, which does not fit a round glass linearly, so
 * display radius uses fixed log-spaced rings with a fourth-root
 * radial factor for each planet's excursion about its own ring.
 * That scaling is display-only, applied to already-computed
 * state; nothing computed in render flows back into state.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define NPL 8
#define KEPLER_ITERS 5

/* Element slots, in Table 1 order. */
#define EL_A 0     /* semi-major axis, au */
#define EL_E 1     /* eccentricity */
#define EL_I 2     /* inclination, deg */
#define EL_L 3     /* mean longitude, deg */
#define EL_VARPI 4 /* longitude of perihelion, deg */
#define EL_OMEGA 5 /* longitude of ascending node, deg */

typedef struct {
    f64 elem0[NPL][6];   /* J2000.0 elements */
    f64 elemdot[NPL][6]; /* rates per Julian century */
    f64 phase[NPL];      /* mean anomaly, cycles in [0,1) */
    f64 dphase[NPL];     /* mean-anomaly increment per step, cycles */
    f64 pos[NPL][3];     /* heliocentric ecliptic J2000 position, au */
    f64 ecc_anom[NPL];   /* eccentric anomaly, cycles (diagnostic) */
    f64 kepler_resid[NPL]; /* E - (e/2pi) sinPi(2E) - M, cycles */
    f64 radius[NPL];     /* heliocentric distance, au (diagnostic) */
    f64 days;            /* elapsed days since J2000.0 */
    f64 dt_days;         /* fixed step, days */
    f64 two_pi;
    f64 c180, c360, days_per_cy;
    uint64_t steps;
    uint64_t orbits[NPL]; /* completed mean-anomaly turns */
} orrery_state;

/* JPL Table 1 (see header). Order: a, e, I, L, varpi, Omega. */
static const char *const k_elem0[NPL][6] = {
    /* Mercury */ { "0.38709927", "0.20563593", "7.00497902", "252.25032350", "77.45779628", "48.33076593" },
    /* Venus   */ { "0.72333566", "0.00677672", "3.39467605", "181.97909950", "131.60246718", "76.67984255" },
    /* EM Bary */ { "1.00000261", "0.01671123", "-0.00001531", "100.46457166", "102.93768193", "0.0" },
    /* Mars    */ { "1.52371034", "0.09339410", "1.84969142", "-4.55343205", "-23.94362959", "49.55953891" },
    /* Jupiter */ { "5.20288700", "0.04838624", "1.30439695", "34.39644051", "14.72847983", "100.47390909" },
    /* Saturn  */ { "9.53667594", "0.05386179", "2.48599187", "49.95424423", "92.59887831", "113.66242448" },
    /* Uranus  */ { "19.18916464", "0.04725744", "0.77263783", "313.23810451", "170.95427630", "74.01692503" },
    /* Neptune */ { "30.06992276", "0.00859048", "1.77004347", "-55.12002969", "44.96476227", "131.78422574" },
};
static const char *const k_elemdot[NPL][6] = {
    /* Mercury */ { "0.00000037", "0.00001906", "-0.00594749", "149472.67411175", "0.16047689", "-0.12534081" },
    /* Venus   */ { "0.00000390", "-0.00004107", "-0.00078890", "58517.81538729", "0.00268329", "-0.27769418" },
    /* EM Bary */ { "0.00000562", "-0.00004392", "-0.01294668", "35999.37244981", "0.32327364", "0.0" },
    /* Mars    */ { "0.00001847", "0.00007882", "-0.00813131", "19140.30268499", "0.44441088", "-0.29257343" },
    /* Jupiter */ { "-0.00011607", "-0.00013253", "-0.00183714", "3034.74612775", "0.21252668", "0.20469106" },
    /* Saturn  */ { "-0.00125060", "-0.00050991", "0.00193609", "1222.49362201", "-0.41897216", "-0.28867794" },
    /* Uranus  */ { "-0.00196176", "-0.00004397", "-0.00242939", "428.48202785", "0.40805281", "0.04240589" },
    /* Neptune */ { "0.00026291", "0.00005105", "0.00035372", "218.45945325", "-0.32241464", "-0.00508664" },
};

static f64 sin_deg(const orrery_state *s, f64 deg)
{
    return f64_sin_pi(f64_div(deg, s->c180));
}
static f64 cos_deg(const orrery_state *s, f64 deg)
{
    return f64_cos_pi(f64_div(deg, s->c180));
}

/* Recompute every planet position from days + phases, in
 * softfloat, and store them in state. Called by reset (t = 0)
 * and by every step. L is not evaluated here: phase carries
 * M = L - varpi modulo one turn (see header). */
static void compute_positions(orrery_state *s)
{
    f64 T = f64_div(s->days, s->days_per_cy); /* Julian centuries past J2000 */
    for (int i = 0; i < NPL; i++) {
        f64 a = f64_fma(s->elemdot[i][EL_A], T, s->elem0[i][EL_A]);
        f64 e = f64_fma(s->elemdot[i][EL_E], T, s->elem0[i][EL_E]);
        f64 inc = f64_fma(s->elemdot[i][EL_I], T, s->elem0[i][EL_I]);
        f64 varpi = f64_fma(s->elemdot[i][EL_VARPI], T, s->elem0[i][EL_VARPI]);
        f64 node = f64_fma(s->elemdot[i][EL_OMEGA], T, s->elem0[i][EL_OMEGA]);
        f64 w = f64_sub(varpi, node); /* argument of perihelion, deg */
        f64 m = s->phase[i];          /* mean anomaly, cycles */
        f64 k = f64_div(e, s->two_pi);

        /* Kepler solve in cycles, fixed iteration count. */
        f64 E = f64_fma(k, f64_sin_pi(f64_mul(F64_TWO, m)), m);
        f64 sE = F64_ZERO, cE = F64_ONE;
        for (int it = 0; it < KEPLER_ITERS; it++) {
            f64 twoE = f64_mul(F64_TWO, E);
            sE = f64_sin_pi(twoE);
            cE = f64_cos_pi(twoE);
            f64 f = f64_sub(f64_sub(E, f64_mul(k, sE)), m);
            f64 fp = f64_sub(F64_ONE, f64_mul(e, cE));
            E = f64_sub(E, f64_div(f, fp));
        }
        f64 twoE = f64_mul(F64_TWO, E);
        sE = f64_sin_pi(twoE);
        cE = f64_cos_pi(twoE);
        s->ecc_anom[i] = E;
        s->kepler_resid[i] = f64_sub(f64_sub(E, f64_mul(k, sE)), m);

        /* Orbital-plane coordinates, x' toward perihelion. */
        f64 sq = f64_sqrt(f64_fma(f64_neg(e), e, F64_ONE)); /* sqrt(1-e^2) */
        f64 xp = f64_mul(a, f64_sub(cE, e));
        f64 yp = f64_mul(f64_mul(a, sq), sE);

        /* Rotate to ecliptic J2000 (JPL formula). */
        f64 cw = cos_deg(s, w), sw = sin_deg(s, w);
        f64 cO = cos_deg(s, node), sO = sin_deg(s, node);
        f64 cI = cos_deg(s, inc), sI = sin_deg(s, inc);
        f64 m11 = f64_sub(f64_mul(cw, cO), f64_mul(f64_mul(sw, sO), cI));
        f64 m12 = f64_neg(f64_add(f64_mul(sw, cO), f64_mul(f64_mul(cw, sO), cI)));
        f64 m21 = f64_add(f64_mul(cw, sO), f64_mul(f64_mul(sw, cO), cI));
        f64 m22 = f64_sub(f64_mul(f64_mul(cw, cO), cI), f64_mul(sw, sO));
        f64 m31 = f64_mul(sw, sI);
        f64 m32 = f64_mul(cw, sI);
        f64 x = f64_fma(m12, yp, f64_mul(m11, xp));
        f64 y = f64_fma(m22, yp, f64_mul(m21, xp));
        f64 z = f64_fma(m32, yp, f64_mul(m31, xp));
        s->pos[i][0] = x;
        s->pos[i][1] = y;
        s->pos[i][2] = z;
        s->radius[i] = f64_sqrt(f64_fma(x, x, f64_fma(y, y, f64_mul(z, z))));
    }
}

static void orrery_reset(void *st_)
{
    orrery_state *s = (orrery_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->days = F64_ZERO;
    s->dt_days = f64_dec("0.5");
    s->two_pi = f64_dec("6.28318530717958647692528676655900576839433879875");
    s->c180 = f64_dec("180");
    s->c360 = f64_dec("360");
    s->days_per_cy = f64_dec("36525");
    for (int i = 0; i < NPL; i++) {
        for (int j = 0; j < 6; j++) {
            s->elem0[i][j] = f64_dec(k_elem0[i][j]);
            s->elemdot[i][j] = f64_dec(k_elemdot[i][j]);
        }
        /* Initial mean anomaly in cycles, wrapped to [0,1).
         * Table values give (L - varpi)/360 in (-1, 1), so one
         * correction suffices; the fixed loop is margin. Sign and
         * >= 1 tests read the bit pattern as an integer, which is
         * order-preserving for the non-negative values after any
         * sign correction and exact for the sign bit itself. */
        f64 ph = f64_div(f64_sub(s->elem0[i][EL_L], s->elem0[i][EL_VARPI]), s->c360);
        for (int k = 0; k < 4; k++) {
            if ((ph & F64_SIGN) && ph != F64_SIGN)
                ph = f64_add(ph, F64_ONE);
            else if (ph >= F64_ONE)
                ph = f64_sub(ph, F64_ONE);
        }
        s->phase[i] = ph;
        /* Per-step phase increment: (Ldot - varpidot) deg/cy
         * converted to cycles/day, times the fixed step. */
        f64 dm = f64_sub(s->elemdot[i][EL_L], s->elemdot[i][EL_VARPI]);
        s->dphase[i] = f64_mul(f64_div(f64_div(dm, s->c360), s->days_per_cy), s->dt_days);
    }
    compute_positions(s);
}

static void orrery_step(void *st_)
{
    orrery_state *s = (orrery_state *)st_;
    s->days = f64_add(s->days, s->dt_days);
    s->steps++;
    for (int i = 0; i < NPL; i++) {
        f64 ph = f64_add(s->phase[i], s->dphase[i]);
        /* phase and dphase are in [0,1) and (0, 0.01), so the sum
         * is non-negative and below 1.01: one conditional turn
         * subtraction keeps it wrapped. Bit patterns of
         * non-negative binary64 values order like the values. */
        if (ph >= F64_ONE) {
            ph = f64_sub(ph, F64_ONE);
            s->orbits[i]++;
        }
        s->phase[i] = ph;
    }
    compute_positions(s);
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Native display sqrt by fixed Newton iterations; render only,
 * so no libm dependency for freestanding hosts. Inputs here are
 * bounded (au ratios and pixel distances), 24 iterations is far
 * past convergence from the crude start below. */
static double dsqrt(double v)
{
    if (v <= 0.0) return 0.0;
    double g = v > 1.0 ? v : 1.0;
    for (int i = 0; i < 24; i++) g = 0.5 * (g + v / g);
    return g;
}

static void orrery_render(const void *st_, toy_fb_t *fb)
{
    const orrery_state *s = (const orrery_state *)st_;
    /* Log-spaced display rings, fraction of toy_r. Spacing echoes
     * log(a): a wide Mars-Jupiter gap, compressed outer system. */
    static const double ring_frac[NPL] = { 0.19, 0.28, 0.36, 0.45, 0.62, 0.74, 0.86, 0.96 };
    static const unsigned disc_nom[NPL] = { 2, 3, 3, 2, 5, 4, 3, 3 }; /* at R=120 */
    static const unsigned char rgb[NPL][3] = {
        { 175, 165, 155 }, { 232, 192, 110 }, { 80, 145, 255 }, { 232, 102, 60 },
        { 222, 172, 120 }, { 214, 192, 142 }, { 140, 222, 222 }, { 92, 112, 255 },
    };
    static const char *const tag[NPL] = { "ME", "VE", "EA", "MA", "JU", "SA", "UR", "NE" };
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    int sc = R >= 150 ? 2 : 1;
    toy_fb_clear(fb, toy_rgb565(4, 6, 16));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));
    for (int i = 0; i < NPL; i++) {
        uint16_t dim = toy_rgb565(rgb[i][0] / 5, rgb[i][1] / 5, rgb[i][2] / 5 + 8);
        toy_fb_ring(fb, cx, cy, (int)(ring_frac[i] * (double)R), dim);
    }
    /* Sun at the focus. */
    toy_fb_disc(fb, cx, cy, R / 18 + 3, toy_rgb565(255, 205, 70));
    toy_fb_disc(fb, cx, cy, R / 36 + 1, toy_rgb565(255, 244, 200));

    /* Outer planets first so inner discs and tags draw on top. */
    for (int i = NPL - 1; i >= 0; i--) {
        double x = d64(s->pos[i][0]), y = d64(s->pos[i][1]);
        double rp = dsqrt(x * x + y * y); /* ecliptic-projected radius, au */
        if (rp <= 0.0) continue;
        double a = d64(s->elem0[i][EL_A]);
        /* Fourth-root excursion about the planet's log-spaced ring:
         * direction is the true projected direction, eccentricity
         * shows as a compressed radial wobble around the ring. */
        double disp = ring_frac[i] * dsqrt(dsqrt(rp / a)) * (double)R;
        int px = cx + (int)(x / rp * disp);
        int py = cy - (int)(y / rp * disp);
        int disc = (int)(R * (int)disc_nom[i] / 120);
        if (disc < 2) disc = 2;
        uint16_t col = toy_rgb565(rgb[i][0], rgb[i][1], rgb[i][2]);
        toy_fb_disc(fb, px, py, disc, col);
        if (i == 5) toy_fb_ring(fb, px, py, disc + 2, col); /* Saturn */

        /* Two-letter tag centred radially outward of the disc,
         * far enough out that the glyph box clears the disc, and
         * flipped inward when the outward spot would leave the
         * safe circle. */
        double dx = (double)(px - cx), dy = (double)(py - cy);
        double dist = dsqrt(dx * dx + dy * dy);
        if (dist <= 0.0) continue;
        double ux = dx / dist, uy = dy / dist;
        int tw = 2 * 4 * sc - sc;      /* 2 glyphs, 3x5 font */
        int th = 5 * sc;
        int half = (tw > th ? tw : th) / 2;
        int off = disc + half + 2 * sc; /* disc edge to tag centre */
        int tx = px + (int)(ux * (double)off) - tw / 2;
        int ty = py + (int)(uy * (double)off) - th / 2;
        double tcx = (double)(tx + tw / 2 - cx), tcy = (double)(ty + th / 2 - cy);
        if (dsqrt(tcx * tcx + tcy * tcy) > 0.90 * (double)R) {
            tx = px - (int)(ux * (double)off) - tw / 2;
            ty = py - (int)(uy * (double)off) - th / 2;
        }
        toy_fb_text(fb, tx, ty, sc, col, tag[i]);
    }

    toy_fb_text(fb, cx - 12 * sc, cy - R + 10 * sc, sc, toy_rgb565(150, 170, 200), "ORRERY");

    /* Elapsed whole days since J2000.0, decimal by hand. */
    char buf[24];
    char rev[20];
    unsigned long long di = (unsigned long long)d64(s->days);
    int nd = 0;
    if (di == 0) rev[nd++] = '0';
    while (di > 0 && nd < 19) { rev[nd++] = (char)('0' + (int)(di % 10u)); di /= 10u; }
    int n = 0;
    buf[n++] = 'D'; buf[n++] = 'A'; buf[n++] = 'Y'; buf[n++] = ' ';
    for (int k = nd - 1; k >= 0; k--) buf[n++] = rev[k];
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, cy + R - 16 * sc, sc,
                toy_rgb565(150, 170, 200), buf);
}

const toy_cart_t toy_cart_orrery = {
    "orrery", "Orrery Clock",
    "JPL approximate Keplerian elements (J2000). A clock, not an ephemeris.",
    sizeof(orrery_state), 1, 2000, 250, orrery_reset, orrery_step, orrery_render, 0, 0, 0, 0, 0, 0, 0
};
