/* cart_intervallens.c - certified vs naive: an interval-geometry lens.
 *
 * A point orbits the origin while its radius "breathes" sinusoidally
 * across the unit circle (radius 1 +/- 0.25). Each step stores the
 * naive binary64 signed distance d = sqrt(x^2+y^2) - 1 (ties-to-even)
 * and a certified interval enclosure of the same quantity over the
 * lens box [x-delta, x+delta] x [y-delta, y+delta], delta = 0.02,
 * around the stored point. Classes: certified-inside (hi < 0),
 * certified-outside (lo > 0), uncertain (interval straddles 0).
 *
 * Certification scope (exact): the sealed library exposes no interval
 * type (see muse_softfloat.h; only directed rounding modes and the
 * directed-rounding-capable basic ops). The enclosure is therefore
 * built here from directed-rounding library calls - SANCTIONED for
 * this cartridge, flagged below - and encloses the exact real range
 * of sqrt(X^2+Y^2)-1 for (X,Y) in the box around the *stored* f64
 * point (x,y). It makes no claim about the unrounded mathematical
 * orbit point (phase/trig rounding is ~1 ulp, not certified here).
 * Directed calls occur only in reset()/step(), never in render().
 *
 * State stores the naive value, both endpoints, the classification,
 * per-class step tallies (n_in+n_out+n_unc == steps), and a
 * containment-violation counter (naive value outside [lo,hi]); the
 * counter must remain 0 - the naive point is the box centre, so its
 * exact distance is interior to the enclosure by construction.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define ITRAIL 320

enum { CLS_IN = 0, CLS_OUT = 1, CLS_UNC = 2 };

typedef struct {
    f64 phase_o, phase_b;     /* orbit / breathing phase, cycles [0,1) */
    f64 dphase_o, dphase_b;
    f64 amp, delta;           /* breathing amplitude, lens half-width */
    f64 x, y, radius, breath; /* stored naive point and its drivers */
    f64 naive_d, lo_d, hi_d, width_d;
    uint64_t steps;
    uint64_t klass;           /* CLS_* */
    uint64_t n_in, n_out, n_unc;
    uint64_t violations;      /* naive_d outside [lo_d,hi_d]; must stay 0 */
    f64 trail_x[ITRAIL], trail_y[ITRAIL];
    uint64_t trail_class[ITRAIL];
} ilens_state;

/* --- exact finite comparisons on bit patterns (integer only) --- */
/* All values compared here are finite by construction (bounded orbit,
 * bounded box). The radix-order key gives numeric order on finite
 * values; signed zeros are treated as equal. */
static int lt_zero(f64 a) { return (a >> 63) != 0 && (a & ~F64_SIGN) != 0; }
static int gt_zero(f64 a) { return (a >> 63) == 0 && a != 0; }
static uint64_t ord_key(f64 a) { return (a >> 63) ? ~a : (a | F64_SIGN); }
static int f64_lt(f64 a, f64 b)
{
    if ((a & ~F64_SIGN) == 0 && (b & ~F64_SIGN) == 0) return 0; /* +-0 == +-0 */
    return ord_key(a) < ord_key(b);
}

/* --- SANCTIONED directed-rounding library calls (reset/step only) --- */
static f64 add_dn(f64 a, f64 b) { msf_flags_t f = 0; return msf64_add(a, b, MSF_ROUND_TOWARD_NEGATIVE, &f); }
static f64 add_up(f64 a, f64 b) { msf_flags_t f = 0; return msf64_add(a, b, MSF_ROUND_TOWARD_POSITIVE, &f); }
static f64 sub_dn(f64 a, f64 b) { msf_flags_t f = 0; return msf64_sub(a, b, MSF_ROUND_TOWARD_NEGATIVE, &f); }
static f64 sub_up(f64 a, f64 b) { msf_flags_t f = 0; return msf64_sub(a, b, MSF_ROUND_TOWARD_POSITIVE, &f); }
static f64 mul_dn(f64 a, f64 b) { msf_flags_t f = 0; return msf64_mul(a, b, MSF_ROUND_TOWARD_NEGATIVE, &f); }
static f64 mul_up(f64 a, f64 b) { msf_flags_t f = 0; return msf64_mul(a, b, MSF_ROUND_TOWARD_POSITIVE, &f); }
static f64 sqrt_dn(f64 a) { msf_flags_t f = 0; return msf64_sqrt(a, MSF_ROUND_TOWARD_NEGATIVE, &f); }
static f64 sqrt_up(f64 a) { msf_flags_t f = 0; return msf64_sqrt(a, MSF_ROUND_TOWARD_POSITIVE, &f); }

typedef struct { f64 lo, hi; } iv_t;

static iv_t iv_box(f64 c, f64 delta)
{
    iv_t r = { sub_dn(c, delta), add_up(c, delta) };
    return r;
}
static iv_t iv_square(iv_t v)
{
    iv_t r;
    if (!lt_zero(v.lo)) {          /* v >= 0 */
        r.lo = mul_dn(v.lo, v.lo);
        r.hi = mul_up(v.hi, v.hi);
    } else if (!gt_zero(v.hi)) {   /* v <= 0 */
        r.lo = mul_dn(v.hi, v.hi);
        r.hi = mul_up(v.lo, v.lo);
    } else {                       /* straddles 0 */
        f64 a = mul_up(v.lo, v.lo), b = mul_up(v.hi, v.hi);
        r.lo = F64_ZERO;
        r.hi = f64_lt(a, b) ? b : a;
    }
    return r;
}
static iv_t iv_add(iv_t a, iv_t b)
{
    iv_t r = { add_dn(a.lo, b.lo), add_up(a.hi, b.hi) };
    return r;
}
static iv_t iv_sqrt(iv_t v) /* v.lo >= 0 here (sum of squares) */
{
    iv_t r = { sqrt_dn(v.lo), sqrt_up(v.hi) };
    return r;
}
static iv_t iv_sub_one(iv_t v)
{
    iv_t r = { sub_dn(v.lo, F64_ONE), sub_up(v.hi, F64_ONE) };
    return r;
}

/* Naive point (ties-to-even via toy_math) + certified enclosure.
 * Sets display/proof fields; tallies are the caller's job. */
static void evaluate(ilens_state *s)
{
    f64 ang = f64_mul(F64_TWO, s->phase_o);
    s->breath = f64_sin_pi(f64_mul(F64_TWO, s->phase_b));
    s->radius = f64_fma(s->amp, s->breath, F64_ONE);
    s->x = f64_mul(s->radius, f64_cos_pi(ang));
    s->y = f64_mul(s->radius, f64_sin_pi(ang));
    f64 r2 = f64_fma(s->x, s->x, f64_mul(s->y, s->y));
    s->naive_d = f64_sub(f64_sqrt(r2), F64_ONE);
    iv_t d = iv_sub_one(iv_sqrt(iv_add(iv_square(iv_box(s->x, s->delta)),
                                       iv_square(iv_box(s->y, s->delta)))));
    s->lo_d = d.lo;
    s->hi_d = d.hi;
    s->width_d = f64_sub(d.hi, d.lo);
    s->klass = lt_zero(d.hi) ? CLS_IN : (gt_zero(d.lo) ? CLS_OUT : CLS_UNC);
}

static void ilens_reset(void *st_)
{
    ilens_state *s = (ilens_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->phase_o = F64_ZERO;
    s->phase_b = f64_dec("0.75"); /* breath = -1: radius 0.75, certified in */
    s->dphase_o = f64_dec("0.0005"); /* one orbit per 2000 steps */
    s->dphase_b = f64_dec("0.001");  /* one breath per 1000 steps */
    s->amp = f64_dec("0.25");
    s->delta = f64_dec("0.02");
    evaluate(s);
    for (int t = 0; t < ITRAIL; t++) {
        s->trail_x[t] = s->x;
        s->trail_y[t] = s->y;
        s->trail_class[t] = s->klass;
    }
}

static f64 wrap_cycles(f64 p) /* p in [0, 1+dphase): one subtract enough */
{
    return f64_lt(p, F64_ONE) ? p : f64_sub(p, F64_ONE);
}

static void ilens_step(void *st_)
{
    ilens_state *s = (ilens_state *)st_;
    s->phase_o = wrap_cycles(f64_add(s->phase_o, s->dphase_o));
    s->phase_b = wrap_cycles(f64_add(s->phase_b, s->dphase_b));
    evaluate(s);
    s->steps++;
    if (s->klass == CLS_IN) s->n_in++;
    else if (s->klass == CLS_OUT) s->n_out++;
    else s->n_unc++;
    if (f64_lt(s->naive_d, s->lo_d) || f64_lt(s->hi_d, s->naive_d))
        s->violations++;
    size_t slot = (size_t)(s->steps % ITRAIL);
    s->trail_x[slot] = s->x;
    s->trail_y[slot] = s->y;
    s->trail_class[slot] = s->klass;
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Tiny decimal writer for the counts line (render-only, no libc). */
static char *put_u64(char *p, uint64_t v)
{
    char tmp[20];
    int n = 0;
    if (v == 0) tmp[n++] = '0';
    while (v) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n) *p++ = tmp[--n];
    return p;
}
static char *put_str(char *p, const char *s) { while (*s) *p++ = *s++; return p; }

static void ilens_render(const void *st_, toy_fb_t *fb)
{
    const ilens_state *s = (const ilens_state *)st_;
    static const uint16_t col_cls[3] = { 0x07f0u, 0xfd20u, 0xf81fu }; /* green amber magenta */
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    double scale = (double)R * 0.70; /* world unit -> px; 1.25 max radius fits */
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));
    uint16_t faint = toy_rgb565(30, 40, 66);
    toy_fb_ring(fb, cx, cy, (int)(0.75 * scale), faint); /* breath extremes */
    toy_fb_ring(fb, cx, cy, (int)(1.25 * scale), faint);
    /* Current distance-interval band, as concentric radii 1+lo, 1+hi. */
    uint16_t cc = col_cls[s->klass <= 2 ? s->klass : 2];
    unsigned bp = cc;
    uint16_t band = (uint16_t)(((((bp >> 11) & 31u) * 100u) >> 8 << 11) |
                               ((((bp >> 5) & 63u) * 100u) >> 8 << 5) |
                               (((bp & 31u) * 100u) >> 8));
    double rlo = (1.0 + d64(s->lo_d)) * scale, rhi = (1.0 + d64(s->hi_d)) * scale;
    if (rlo > 2.0) toy_fb_ring(fb, cx, cy, (int)rlo, band);
    if (rhi > 2.0) toy_fb_ring(fb, cx, cy, (int)rhi, band);
    /* The certified boundary itself. */
    toy_fb_ring(fb, cx, cy, (int)scale, toy_rgb565(200, 210, 230));
    if (R >= 150) toy_fb_ring(fb, cx, cy, (int)scale - 1, toy_rgb565(120, 130, 155));
    /* Trail, coloured by the classification at each stored step. */
    size_t head = (size_t)(s->steps % ITRAIL);
    for (int t = 0; t < ITRAIL; t++) {
        size_t idx = (head + ITRAIL - (size_t)t) % ITRAIL;
        int x = cx + (int)(d64(s->trail_x[idx]) * scale);
        int y = cy - (int)(d64(s->trail_y[idx]) * scale);
        unsigned fade = 255u - (unsigned)(t * 200 / ITRAIL);
        unsigned p = col_cls[s->trail_class[idx] <= 2 ? s->trail_class[idx] : 2];
        uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                (((p & 31u) * fade) >> 8));
        toy_fb_px(fb, x, y, c);
    }
    int px = cx + (int)(d64(s->x) * scale), py = cy - (int)(d64(s->y) * scale);
    toy_fb_line(fb, cx, cy, px, py, toy_rgb565(52, 58, 84));
    /* Lens box at true scale (half-width at least 2 px so the outline
     * survives at 240); drawn last so the marker cannot swallow it. */
    int half = (int)(d64(s->delta) * scale + 0.5);
    if (half < 2) half = 2;
    toy_fb_disc(fb, px, py, R / 100 + 1, cc);
    toy_fb_line(fb, px - half, py - half, px + half, py - half, cc);
    toy_fb_line(fb, px + half, py - half, px + half, py + half, cc);
    toy_fb_line(fb, px + half, py + half, px - half, py + half, cc);
    toy_fb_line(fb, px - half, py + half, px - half, py - half, cc);
    toy_fb_disc(fb, px, py, 1, 0xffffu);
    toy_fb_disc(fb, cx, cy, R / 70 + 1, 0xffffu);
    /* Text inside the inscribed circle's safe area. */
    int sc = R >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 26 * sc, cy - (R * 3) / 4, sc, toy_rgb565(150, 170, 200), "INTERVAL LENS");
    const char *word = s->klass == CLS_IN ? "CERT IN" : (s->klass == CLS_OUT ? "CERT OUT" : "UNSURE");
    int wlen = s->klass == CLS_OUT ? 8 : (s->klass == CLS_IN ? 7 : 6);
    toy_fb_text(fb, cx - (wlen * 2) * sc, cy + (R * 55) / 100, sc, cc, word);
    int lx = cx - 26 * sc, ly = cy + (R * 66) / 100;
    lx = toy_fb_text(fb, lx, ly, sc, col_cls[CLS_IN], "IN ");
    lx = toy_fb_text(fb, lx, ly, sc, col_cls[CLS_OUT], "OUT ");
    toy_fb_text(fb, lx, ly, sc, col_cls[CLS_UNC], "UNSURE");
    char counts[48];
    char *p = counts;
    p = put_str(p, "I"); p = put_u64(p, s->n_in);
    p = put_str(p, " O"); p = put_u64(p, s->n_out);
    p = put_str(p, " U"); p = put_u64(p, s->n_unc);
    p = put_str(p, " V"); p = put_u64(p, s->violations);
    *p = 0;
    int clen = (int)(p - counts);
    toy_fb_text(fb, cx - (clen * 2) * sc, ly + 8 * sc, sc, toy_rgb565(150, 170, 200), counts);
}

const toy_cart_t toy_cart_intervallens = {
    "intervallens", "Interval Lens",
    "A breathing orbit crosses the unit circle: certified in, certified out, or honestly unsure.",
    sizeof(ilens_state), 10, 4000, 500, ilens_reset, ilens_step, ilens_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
