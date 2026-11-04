/* cart_harmonograph.c - a damped coupled-pendulum drawing machine.
 *
 * Three damped pendulums move one pen. Pendulum 0 swings the pen
 * along x, pendulum 1 along y, and pendulum 2 is the rotary table
 * pendulum, contributing to both axes in quadrature:
 *
 *   x = A0 sin(2 pi ph0) + A2 sin(2 pi ph2)
 *   y = A1 sin(2 pi ph1) + A2 cos(2 pi ph2)
 *
 * Phases ph[] are held in CYCLES in [0,1) and evaluated with
 * f64_sin_pi/f64_cos_pi at 2*ph, so no angle ever winds without
 * bound and no argument reduction is left to chance. Damping is
 * exact per-step multiplication by a stored decay factor (the
 * discrete form of exp(-t/tau); no exp is ever evaluated): each
 * pendulum has its own factor, so the figure spirals inward and
 * slowly changes shape as the pendulums die at different rates.
 *
 * When the reference amplitude A0 falls below the stored
 * threshold, step() deterministically loads the next entry of
 * the frequency-ratio table below (fresh amplitudes, phases and
 * ratio; the trail is refilled with the new pen position, like
 * lifting the pen onto a fresh sheet). Every ratio carries a
 * small deliberate detune on one pendulum, so the closed
 * Lissajous curve precesses and weaves a rosette band instead of
 * retracing itself. With A0 = 0.62, decay 0.99872 and threshold
 * 0.09 one pattern lives ~1507 steps (~6.0 base cycles at the
 * base increment of 0.004 cycles/step), so the 6000-step proof
 * crosses three pattern changes (at steps ~1507, 3014, 4521).
 *
 * The pen trail is a ring buffer in state (simulation history,
 * hashed and checkpointed with everything else). Trail-size
 * arithmetic: HTRAIL points * 2 coordinates * 8 bytes
 * = 2048 * 16 = 32768 bytes; scalars and padding add 160 bytes,
 * for a 32928-byte state, half the 65536-byte cap. The ring is
 * longer than one pattern's life (~1507 points), so a complete
 * decayed figure is still on screen when the pattern switches.
 * Fading is render-only: segments are dimmed by ring age at draw
 * time and never feed back into state.
 *
 * Screen safety: |x| <= A0 + A2 = 0.90 and |y| <= A1 + A2 = 0.88
 * on the first step and amplitudes only shrink, so at the render
 * scale of 0.72 R per unit the pen can never leave the disc of
 * radius 0.72 * sqrt(0.90^2 + 0.88^2) ~= 0.91 R.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define HTRAIL 2048
#define NPEND 3

typedef struct {
    f64 ph[NPEND];    /* phases, cycles, in [0,1) */
    f64 dph[NPEND];   /* phase increments, cycles/step */
    f64 amp[NPEND];   /* current amplitudes, pen units */
    f64 decay[NPEND]; /* per-step amplitude multipliers */
    f64 base_inc;     /* cycles/step for ratio unit 1 */
    f64 threshold;    /* advance pattern when amp[0] < this */
    f64 pen[2];       /* current pen position */
    uint64_t steps;
    uint64_t pattern_steps;
    uint32_t pattern;     /* index into PATTERNS */
    uint32_t trail_head;  /* ring slot of the newest point */
    uint32_t trail_count; /* valid points, <= HTRAIL */
    f64 trail[HTRAIL][2];
} harm_state;

/* One row per pattern: frequency multipliers (nominal integer
 * ratio plus a small detune on one pendulum), initial phases in
 * cycles, and the on-screen nominal-ratio label. */
typedef struct {
    const char *f[NPEND];
    const char *ph[NPEND];
    const char *label;
} harm_pattern;

static const harm_pattern PATTERNS[] = {
    { { "2", "3.015", "1" },    { "0", "0.25", "0" },     "2:3:1" },
    { { "3.02", "4", "1" },     { "0", "0.5", "0.25" },   "3:4:1" },
    { { "1", "2", "1.01" },     { "0", "0.25", "0.25" },  "1:2:1" },
    { { "3", "2.015", "1" },    { "0.125", "0", "0.375" },"3:2:1" },
    { { "4", "3", "2.01" },     { "0", "0.25", "0.125" }, "4:3:2" },
    { { "1", "1.05", "1" },     { "0", "0.25", "0.125" }, "1:1:1" },
};
#define NPAT ((uint32_t)(sizeof PATTERNS / sizeof PATTERNS[0]))

/* a < b for the positive finite values used here, via the sign
 * of a softfloat subtraction (a == b gives +0, never -0). */
static int f64_lt(f64 a, f64 b)
{
    return (f64_sub(a, b) & F64_SIGN) != 0;
}

/* Phases stay in [0,1): increments are < 0.02 cycles, so one
 * conditional subtract always suffices (and is exact by Sterbenz
 * for ph in [1,2)). */
static f64 wrap_cycle(f64 ph)
{
    f64 d = f64_sub(ph, F64_ONE);
    return (d & F64_SIGN) == 0 ? d : ph;
}

/* Pen position from phases and amplitudes. Softfloat; called
 * only from pattern load and step(), never from render(). */
static void pen_pos(const harm_state *s, f64 *x, f64 *y)
{
    f64 sx = f64_sin_pi(f64_mul(F64_TWO, s->ph[0]));
    f64 sy = f64_sin_pi(f64_mul(F64_TWO, s->ph[1]));
    f64 rx = f64_sin_pi(f64_mul(F64_TWO, s->ph[2]));
    f64 ry = f64_cos_pi(f64_mul(F64_TWO, s->ph[2]));
    *x = f64_fma(s->amp[2], rx, f64_mul(s->amp[0], sx));
    *y = f64_fma(s->amp[2], ry, f64_mul(s->amp[1], sy));
}

/* Load a pattern: fresh amplitudes, table phases and frequency
 * increments; pen lifted onto a fresh sheet (trail refilled with
 * the new pen position so no segment joins two patterns). */
static void load_pattern(harm_state *s, uint32_t idx)
{
    s->pattern = idx;
    s->pattern_steps = 0;
    s->amp[0] = f64_dec("0.62");
    s->amp[1] = f64_dec("0.60");
    s->amp[2] = f64_dec("0.28");
    for (int i = 0; i < NPEND; i++) {
        s->ph[i] = f64_dec(PATTERNS[idx].ph[i]);
        s->dph[i] = f64_mul(s->base_inc, f64_dec(PATTERNS[idx].f[i]));
    }
    pen_pos(s, &s->pen[0], &s->pen[1]);
    for (int t = 0; t < HTRAIL; t++) {
        s->trail[t][0] = s->pen[0];
        s->trail[t][1] = s->pen[1];
    }
    s->trail_head = 0;
    s->trail_count = 1;
}

static void harm_reset(void *st_)
{
    harm_state *s = (harm_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->base_inc = f64_dec("0.004");
    s->threshold = f64_dec("0.09");
    /* Per-pendulum damping: x reference, y slightly livelier,
     * rotary dies first. */
    s->decay[0] = f64_dec("0.99872");
    s->decay[1] = f64_dec("0.99878");
    s->decay[2] = f64_dec("0.99855");
    load_pattern(s, 0);
}

static void harm_step(void *st_)
{
    harm_state *s = (harm_state *)st_;
    for (int i = 0; i < NPEND; i++) {
        s->ph[i] = wrap_cycle(f64_add(s->ph[i], s->dph[i]));
        s->amp[i] = f64_mul(s->amp[i], s->decay[i]);
    }
    s->steps++;
    s->pattern_steps++;
    if (f64_lt(s->amp[0], s->threshold)) {
        load_pattern(s, (s->pattern + 1u) % NPAT);
        return;
    }
    pen_pos(s, &s->pen[0], &s->pen[1]);
    s->trail_head = (s->trail_head + 1u) % HTRAIL;
    s->trail[s->trail_head][0] = s->pen[0];
    s->trail[s->trail_head][1] = s->pen[1];
    if (s->trail_count < HTRAIL) s->trail_count++;
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

static void harm_render(const void *st_, toy_fb_t *fb)
{
    const harm_state *s = (const harm_state *)st_;
    const uint16_t ink[6] = {
        toy_rgb565(255, 196, 92),  /* gold */
        toy_rgb565(92, 230, 255),  /* cyan */
        toy_rgb565(120, 255, 170), /* mint */
        toy_rgb565(255, 120, 200), /* pink */
        toy_rgb565(170, 150, 255), /* violet */
        toy_rgb565(255, 140, 110), /* coral */
    };
    int cx = toy_cx(fb), cy = toy_cy(fb);
    double scale = (double)toy_r(fb) * 0.72; /* pen units -> px */
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, toy_r(fb) - 2, toy_rgb565(28, 34, 54));

    /* Fading polyline, oldest to newest. Fade is by ring age
     * only (drawing); stored points are read, never changed. */
    unsigned base = ink[s->pattern % NPAT];
    int prevx = 0, prevy = 0, have = 0;
    for (uint32_t n = s->trail_count; n > 0; n--) {
        uint32_t age = n - 1u; /* 0 = newest */
        uint32_t idx = (s->trail_head + HTRAIL - age) % HTRAIL;
        int x = cx + (int)(d64(s->trail[idx][0]) * scale);
        int y = cy - (int)(d64(s->trail[idx][1]) * scale);
        if (have) {
            unsigned fade = 255u - (age * 200u) / HTRAIL;
            uint16_t c = (uint16_t)(((((base >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((base >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((base & 31u) * fade) >> 8));
            toy_fb_line(fb, prevx, prevy, x, y, c);
        }
        prevx = x; prevy = y; have = 1;
    }

    int px = cx + (int)(d64(s->pen[0]) * scale);
    int py = cy - (int)(d64(s->pen[1]) * scale);
    toy_fb_disc(fb, px, py, toy_r(fb) / 60 + 1, (uint16_t)base);
    toy_fb_disc(fb, px, py, toy_r(fb) / 120 + 1, 0xffffu);

    int sc = toy_r(fb) >= 150 ? 2 : 1;
    toy_fb_text(fb, cx - 24 * sc, cy - toy_r(fb) + 10 * sc, sc,
                toy_rgb565(150, 170, 200), "HARMONOGRAPH");
    toy_fb_text(fb, cx - 10 * sc, cy + toy_r(fb) - 16 * sc, sc,
                toy_rgb565(150, 170, 200), PATTERNS[s->pattern % NPAT].label);
}

const toy_cart_t toy_cart_harmonograph = {
    "harmonograph", "Harmonograph",
    "Three damped pendulums draw one decaying Lissajous rosette, then retune.",
    sizeof(harm_state), 16, 6000, 500, harm_reset, harm_step, harm_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
