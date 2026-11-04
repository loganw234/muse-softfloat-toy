/* cart_microscope.c - function microscope: scripted zoom to the ulp staircase.
 *
 * Plots y = sqrt(x) on windows centred at x = 2, shrinking the
 * half-width by a fixed factor 8 at every stage, from 1/2 down to
 * 2^-49, then cycling back to the widest window. sqrt is chosen
 * because msf64_sqrt is the correctly rounded IEEE operation (no
 * transcendental argument to defend) and it is monotone with a
 * moderate slope at the centre: f'(2) = 1/(2*sqrt(2)) ~= 0.3536.
 *
 * The centre x = 2 is the point: it is a binade boundary. Just
 * below 2 the binary64 spacing is 2^-52; at and above 2 it is
 * 2^-51. So the left half of every deep window contains twice as
 * many representable x values as the right half, and the treads
 * of the staircase are visibly narrower on the left of centre
 * than on the right. The image sqrt(2) ~= 1.414 lies strictly
 * inside [1,2), so y spacing (2^-52) does not change across the
 * window: the asymmetry seen is purely the x spacing change.
 * Per representable x step, y moves f'(2)*ulp(x)/ulp(y) ~= 0.354
 * ulps on the left and ~= 0.707 ulps on the right, so deep stages
 * have only a handful of distinct y levels across all columns.
 *
 * Columns: MCOLS = 129 samples. The offset of column i from the
 * centre is built in step() as an offset accumulator starting at
 * -hw and adding hw/64 (exact: hw is a power of two, 1/64 is a
 * power of two, every partial offset is an exact binary
 * fraction), then x = centre + offset is the single rounding
 * that quantises x onto the representable grid near 2. Keeping
 * the accumulator on the offset (magnitude <= hw) rather than on
 * x itself avoids stagnation, where x + tiny would round back to
 * x forever at the deepest stages. Column 64 is exactly centre.
 *
 * State size: cols 129 * 8 = 1032 bytes, plus 14 f64 scalars
 * (112) and 10 uint64 scalars (80) = 1224 bytes, far under the
 * 65536-byte cap. All members are 8-byte types: no padding.
 *
 * Width in ulps: at each stage begin, step-side code computes
 * ulp_up = next_up(centre) - centre and width/ulp_up in
 * softfloat (stored as width_ulps), then decodes that f64 to a
 * uint64 by integer bit arithmetic (ulps_across). It also stores
 * ulp_steps_exact = bits(x_hi) - bits(x_lo), the exact number of
 * representable steps across the window (larger, because the
 * left half is spaced at half the upward ulp). Render only
 * formats the stored integers; it computes no function values
 * and writes no state.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define MCOLS 129          /* columns per window; centre column 64 */
#define MCOLS_PER_STEP 8   /* bounded fill work per step */
#define MDWELL 13          /* dwell steps after a window completes */
#define MSTAGES 17         /* stages 0..16, hw 2^-1 down to 2^-49 */

typedef struct {
    f64 centre;      /* 2 */
    f64 yc;          /* sqrt(centre) */
    f64 hw;          /* current half-width (power of two) */
    f64 hw_wide;     /* 1/2, the widest half-width */
    f64 shrink;      /* 1/8 per stage */
    f64 off_scale;   /* 1/64: offset step = hw * off_scale */
    f64 x_lo, x_hi;  /* window ends for the current stage */
    f64 y_lo, y_hi;  /* sqrt(x_lo), sqrt(x_hi) */
    f64 off;         /* offset accumulator for the next column */
    f64 off_step;    /* hw/64 for the current stage */
    f64 width_ulps;  /* (2*hw)/ulp_up(centre), softfloat result */
    f64 prev_x;      /* previous column's x (distinct-x counting) */
    f64 cols[MCOLS]; /* function values, filled by step() only */
    uint64_t steps;
    uint64_t stage;          /* 0..MSTAGES-1 */
    uint64_t filled;         /* columns filled this window */
    uint64_t timer;          /* dwell countdown */
    uint64_t ulps_across;    /* width_ulps decoded to an integer */
    uint64_t ulp_steps_exact;/* bits(x_hi) - bits(x_lo) */
    uint64_t hw_exp_neg;     /* hw = 2^-hw_exp_neg */
    uint64_t distinct_x;     /* distinct x values among filled */
    uint64_t distinct_y;     /* distinct y values among filled */
    uint64_t stages_total;   /* MSTAGES, stored for the label */
} micro_state;

/* Decode an f64 that holds a non-negative integer < 2^64 to a
 * uint64 by bit arithmetic. Step-side only (begin_stage). */
static uint64_t f64_to_u64_by_bits(f64 a)
{
    uint64_t bits = (uint64_t)a;
    uint64_t exp = (bits >> 52) & 0x7ffu;
    if (exp == 0) return 0;
    int e = (int)exp - 1023;
    uint64_t mant = (bits & ((1ull << 52) - 1ull)) | (1ull << 52);
    if (e < 0) return 0;
    if (e >= 52) {
        if (e - 52 >= 64) return 0;
        return mant << (e - 52);
    }
    return mant >> (52 - e);
}

/* Set up the current stage's window. Called from reset and from
 * the stage-advance branch of step only (softfloat lives here). */
static void begin_stage(micro_state *s)
{
    s->filled = 0;
    s->timer = 0;
    s->distinct_x = 0;
    s->distinct_y = 0;
    s->prev_x = F64_ZERO;
    for (int i = 0; i < MCOLS; i++) s->cols[i] = F64_ZERO;
    s->x_lo = f64_sub(s->centre, s->hw);
    s->x_hi = f64_add(s->centre, s->hw);
    s->y_lo = f64_sqrt(s->x_lo);
    s->y_hi = f64_sqrt(s->x_hi);
    s->off = f64_neg(s->hw);
    s->off_step = f64_mul(s->hw, s->off_scale);
    f64 ulp_up = f64_sub(f64_next_up(s->centre), s->centre);
    f64 width = f64_add(s->hw, s->hw);
    s->width_ulps = f64_div(width, ulp_up);
    s->ulps_across = f64_to_u64_by_bits(s->width_ulps);
    /* Positive finite f64 bit patterns are monotone in value. */
    s->ulp_steps_exact = (uint64_t)s->x_hi - (uint64_t)s->x_lo;
    /* hw is a power of two below 1: unbiased exponent is <= -1. */
    s->hw_exp_neg = 1023u - (uint64_t)(((uint64_t)s->hw >> 52) & 0x7ffu);
}

static void micro_reset(void *st_)
{
    micro_state *s = (micro_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->centre = f64_dec("2");
    s->yc = f64_sqrt(s->centre);
    s->hw_wide = f64_dec("0.5");
    s->hw = s->hw_wide;
    s->shrink = f64_dec("0.125");
    s->off_scale = f64_dec("0.015625"); /* 1/64, exact */
    s->stages_total = MSTAGES;
    s->stage = 0;
    begin_stage(s);
}

static void micro_step(void *st_)
{
    micro_state *s = (micro_state *)st_;
    if (s->filled < MCOLS) {
        uint64_t left = MCOLS - s->filled;
        uint64_t n = left < MCOLS_PER_STEP ? left : MCOLS_PER_STEP;
        for (uint64_t j = 0; j < n; j++) {
            uint64_t i = s->filled;
            f64 x = f64_add(s->centre, s->off); /* the quantising rounding */
            f64 y = f64_sqrt(x);
            s->cols[i] = y;
            if (i == 0) {
                s->distinct_x = 1;
                s->distinct_y = 1;
            } else {
                if (x != s->prev_x) s->distinct_x++;
                if (y != s->cols[i - 1]) s->distinct_y++;
            }
            s->prev_x = x;
            s->off = f64_add(s->off, s->off_step); /* exact, see header */
            s->filled++;
        }
        if (s->filled >= MCOLS) s->timer = MDWELL;
    } else if (s->timer > 1) {
        s->timer--;
    } else {
        if (s->stage + 1 >= MSTAGES) {
            s->stage = 0;
            s->hw = s->hw_wide;
        } else {
            s->stage++;
            s->hw = f64_mul(s->hw, s->shrink);
        }
        begin_stage(s);
    }
    s->steps++;
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Render-side string building from stored integers only. */
static int put_str(char *buf, int pos, const char *t)
{
    for (; *t; t++) buf[pos++] = *t;
    buf[pos] = 0;
    return pos;
}
static int put_u64(char *buf, int pos, uint64_t v)
{
    char tmp[24];
    int n = 0;
    if (v == 0) tmp[n++] = '0';
    while (v > 0) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n > 0) buf[pos++] = tmp[--n];
    buf[pos] = 0;
    return pos;
}
static int str_len(const char *t) { int n = 0; while (t[n]) n++; return n; }

static void micro_render(const void *st_, toy_fb_t *fb)
{
    const micro_state *s = (const micro_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    int sc = r >= 150 ? 2 : 1;
    uint16_t bg = toy_rgb565(5, 7, 18);
    uint16_t grid = toy_rgb565(40, 52, 84);
    uint16_t axis = toy_rgb565(70, 88, 124);
    uint16_t curve = 0x07ffu;   /* cyan */
    uint16_t amber = 0xfc60u;
    uint16_t label = toy_rgb565(150, 170, 200);

    toy_fb_clear(fb, bg);
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));

    /* Plot box in centre/radius fractions (inscribed-circle safe). */
    int left = cx - (r * 70) / 100;
    int right = cx + (r * 70) / 100;
    int top = cy - (r * 52) / 100;
    int bot = cy + (r * 42) / 100;

    const char *title = "SQRT AT 2";
    toy_fb_text(fb, cx - (str_len(title) * 4 * sc) / 2 + sc,
                cy - r + 9 * sc, sc, label, title);

    /* Vertical scale from stored window ends, with display margin. */
    double ylo = d64(s->y_lo), yhi = d64(s->y_hi), yc = d64(s->yc);
    double span = yhi - ylo;
    if (!(span > 0.0)) { ylo = yc - 1.0; yhi = yc + 1.0; span = 2.0; }
    double pad = span * 0.08;
    ylo -= pad; yhi += pad; span = yhi - ylo;

    /* Axis lines: box, centre vertical (x = centre), y = yc line. */
    toy_fb_line(fb, left, top, right, top, grid);
    toy_fb_line(fb, left, bot, right, bot, grid);
    toy_fb_line(fb, left, top, left, bot, grid);
    toy_fb_line(fb, right, top, right, bot, grid);
    toy_fb_line(fb, cx, top, cx, bot, axis);
    int yc_py = bot - (int)(((yc - ylo) / span) * (double)(bot - top));
    if (yc_py > top && yc_py < bot)
        toy_fb_line(fb, left, yc_py, right, yc_py, axis);

    /* Plot stored columns only; join neighbours so equal values
     * draw as horizontal treads and jumps as near-vertical risers. */
    int prev_x = 0, prev_y = 0, have_prev = 0;
    uint64_t filled = s->filled;
    if (filled > MCOLS) filled = MCOLS;
    for (uint64_t i = 0; i < filled; i++) {
        double v = d64(s->cols[i]);
        int px = left + (int)(((double)i / (double)(MCOLS - 1)) * (double)(right - left));
        int py = bot - (int)(((v - ylo) / span) * (double)(bot - top));
        if (py < top) py = top;
        if (py > bot) py = bot;
        if (have_prev) toy_fb_line(fb, prev_x, prev_y, px, py, curve);
        toy_fb_px(fb, px, py, curve);
        prev_x = px; prev_y = py; have_prev = 1;
        if (i == (MCOLS - 1) / 2)
            toy_fb_disc(fb, px, py, r / 90 + 1, amber);
    }

    /* Stage text: every number is a stored state integer. */
    char line[48];
    int p = 0;
    p = put_str(line, p, "STAGE ");
    p = put_u64(line, p, s->stage);
    p = put_str(line, p, "/");
    p = put_u64(line, p, s->stages_total - 1);
    p = put_str(line, p, " LOG2HW-");
    p = put_u64(line, p, s->hw_exp_neg);
    (void)p;
    toy_fb_text(fb, cx - (str_len(line) * 4 * sc) / 2 + sc,
                bot + 7 * sc, sc, label, line);

    p = 0;
    p = put_str(line, p, "ULPS ");
    p = put_u64(line, p, s->ulps_across);
    (void)p;
    toy_fb_text(fb, cx - (str_len(line) * 4 * sc) / 2 + sc,
                bot + 17 * sc, sc, amber, line);

    p = 0;
    p = put_str(line, p, "X ");
    p = put_u64(line, p, s->distinct_x);
    p = put_str(line, p, " LEVELS ");
    p = put_u64(line, p, s->distinct_y);
    (void)p;
    toy_fb_text(fb, cx - (str_len(line) * 4 * sc) / 2 + sc,
                bot + 27 * sc, sc, label, line);
}

/* Pacing: one stage is ceil(129/8) + 13 = 30 steps, so the full
 * 17-stage cycle is 510 steps. At 13 steps/frame the whole dive
 * (widest smooth curve to deepest staircase) fits the standard
 * 40-frame host sheet, and the 13-step dwell guarantees a frame
 * lands on every completed window. Proof: two full cycles, so
 * the horizon includes the deepest staircase and the wrap. */
const toy_cart_t toy_cart_microscope = {
    "microscope", "Function Microscope",
    "Zoom into sqrt(x) at x=2 until binary64 steps appear.",
    sizeof(micro_state), 13, 1020, 102, micro_reset, micro_step, micro_render, 0, 0, 0, 0, 0, 0, 0
};
