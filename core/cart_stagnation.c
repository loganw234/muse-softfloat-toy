/* cart_stagnation.c - accumulation freeze, made visible.
 *
 * Four lanes each add 1 to a running sum, forever. Each plain lane
 * starts N units below its format's integer ceiling 2^p (p = total
 * precision including the implicit bit: binary16 p=11, binary32
 * p=24, binary64 p=53). The start lies in the binade [2^(p-1), 2^p)
 * where the spacing is exactly 1, so the first N additions are
 * exact and land on 2^p. Addition N+1 has exact result 2^p + 1,
 * the midpoint between 2^p and 2^p + 2 (spacing 2 in the next
 * binade); ties-to-even rounds it back to 2^p and the sum never
 * moves again: sum + 1 == sum. Predicted freeze steps (1-based
 * addition number, = N+1): binary16 257 (start 1792 = 2048-256),
 * binary32 1025 (start 16776192 = 16777216-1024), binary64 2049
 * (start 9007199254738944 = 9007199254740992-2048).
 *
 * The fourth lane repeats the binary64 lane (same start, same +1)
 * with IEEE 754-2019 augmented addition keeping the (head, tail)
 * pair: what the plain lane throws away at the ceiling, this lane
 * keeps in the tail and folds back in, so its total stays exact
 * (start + steps) and the head keeps advancing past 2^53.
 *
 * SANCTIONED EXCEPTION (Wave T1 stagnation parcel): this cartridge
 * calls msf16_*, msf32_* and msf64_* directly instead of the
 * binary64-only toy_math.h wrappers, because comparing the three
 * formats is the phenomenon. All floating-point in reset()/step()
 * still goes through muse-softfloat; no native FP touches state.
 * Constants come from decimal strings via msf*_from_decimal at
 * reset. Flags are discarded per call, as in toy_math.h.
 */
#include "toy_cart.h"
#include "toy_math.h" /* brings in muse_softfloat.h; wrappers unused here */
#include "toy_util.h"

/* All-uint64_t fields: POD, no padding, every byte defined by the
 * reset memset. Sum/start/inc fields hold raw IEEE bit patterns
 * (binary16/binary32 patterns zero-extended into 64 bits). */
typedef struct {
    uint64_t sum16, sum32, sum64;     /* plain-lane running sums */
    uint64_t aug_head, aug_tail;      /* augmented binary64 pair */
    uint64_t start16, start32, start64; /* reset sums (aug shares start64) */
    uint64_t inc16, inc32, inc64;     /* +1 in each format (aug uses inc64) */
    uint64_t steps;                   /* additions performed per lane */
    uint64_t frozen16, frozen32, frozen64, frozen_aug; /* 0 = not frozen yet */
} stag_state;

static msf16_t dec16(const char *s) { msf_flags_t f = 0; return msf16_from_decimal(s, 0, MSF_ROUND_TIES_TO_EVEN, &f); }
static msf32_t dec32(const char *s) { msf_flags_t f = 0; return msf32_from_decimal(s, 0, MSF_ROUND_TIES_TO_EVEN, &f); }
static msf64_t dec64(const char *s) { msf_flags_t f = 0; return msf64_from_decimal(s, 0, MSF_ROUND_TIES_TO_EVEN, &f); }
static msf16_t add16(msf16_t a, msf16_t b) { msf_flags_t f = 0; return msf16_add(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }
static msf32_t add32(msf32_t a, msf32_t b) { msf_flags_t f = 0; return msf32_add(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }
static msf64_t add64(msf64_t a, msf64_t b) { msf_flags_t f = 0; return msf64_add(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }

static void stag_reset(void *st_)
{
    stag_state *s = (stag_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->start16 = dec16("1792");              /* 2^11 - 256 */
    s->start32 = dec32("16776192");          /* 2^24 - 1024 */
    s->start64 = dec64("9007199254738944");  /* 2^53 - 2048 */
    s->inc16 = dec16("1");
    s->inc32 = dec32("1");
    s->inc64 = dec64("1");
    s->sum16 = s->start16;
    s->sum32 = s->start32;
    s->sum64 = s->start64;
    s->aug_head = s->start64;
    s->aug_tail = 0; /* +0.0 binary64 bit pattern: structural zero */
}

static void stag_step(void *st_)
{
    stag_state *s = (stag_state *)st_;
    s->steps++; /* this addition's 1-based number, shared by all lanes */

    uint64_t before16 = s->sum16;
    s->sum16 = add16((msf16_t)s->sum16, (msf16_t)s->inc16);
    if (s->frozen16 == 0 && s->sum16 == before16) s->frozen16 = s->steps;

    uint64_t before32 = s->sum32;
    s->sum32 = add32((msf32_t)s->sum32, (msf32_t)s->inc32);
    if (s->frozen32 == 0 && s->sum32 == before32) s->frozen32 = s->steps;

    uint64_t before64 = s->sum64;
    s->sum64 = add64(s->sum64, s->inc64);
    if (s->frozen64 == 0 && s->sum64 == before64) s->frozen64 = s->steps;

    /* Augmented lane: (head, tail) stays a normalised exact pair.
     * Add the increment to the head, fold the new tail into the old
     * one, then renormalise. Past 2^53 the tail alternates 0/1 and
     * the head advances 2 every other step; the pair never freezes
     * in any reachable horizon. Freeze test compares the whole pair. */
    uint64_t before_head = s->aug_head, before_tail = s->aug_tail;
    msf_flags_t fl = 0;
    msf64_pair_t p1 = msf64_augmented_add(s->aug_head, s->inc64, &fl);
    msf64_t folded = add64(s->aug_tail, p1.tail);
    fl = 0;
    msf64_pair_t p2 = msf64_augmented_add(p1.head, folded, &fl);
    s->aug_head = p2.head;
    s->aug_tail = p2.tail;
    if (s->frozen_aug == 0 && s->aug_head == before_head && s->aug_tail == before_tail)
        s->frozen_aug = s->steps;
}

/* Display conversion only (never feeds state). Widening binary16/32
 * to binary64 is exact, so growth values below are exact integers. */
static double d64(msf64_t a) { union { msf64_t u; double d; } v; v.u = a; return v.d; }
static double widen16(uint64_t bits)
{
    msf_flags_t f = 0;
    return d64(msf64_from_f16((msf16_t)bits, MSF_ROUND_TIES_TO_EVEN, &f));
}
static double widen32(uint64_t bits)
{
    msf_flags_t f = 0;
    return d64(msf64_from_f32((msf32_t)bits, MSF_ROUND_TIES_TO_EVEN, &f));
}

/* Format v (optionally prefixed) into buf; returns the string. */
static char *fmt_u64(char *buf, char prefix, uint64_t v)
{
    char rev[20];
    int n = 0, i = 0;
    if (v == 0) rev[n++] = '0';
    while (v) { rev[n++] = (char)('0' + (int)(v % 10u)); v /= 10u; }
    if (prefix) buf[i++] = prefix;
    while (n) buf[i++] = rev[--n];
    buf[i] = '\0';
    return buf;
}

static int text_w(const char *s, int sc)
{
    int n = 0;
    while (s[n]) n++;
    return n ? n * 4 * sc - sc : 0;
}

static void stag_render(const void *st_, toy_fb_t *fb)
{
    const stag_state *s = (const stag_state *)st_;
    /* Display-only design constants: units from each start to its
     * 2^p ceiling (the plain lane's maximum growth), and the plot
     * scale in the same units. One shared scale, so the augmented
     * lane passing the binary64 ceiling is directly comparable. */
    static const char *const label[4] = { "F16", "F32", "F64", "AUG" };
    static const double ceiling[4] = { 256.0, 1024.0, 2048.0, 2048.0 };
    const double plot_units = 4096.0;
    const uint64_t frozen[4] = { s->frozen16, s->frozen32, s->frozen64, s->frozen_aug };
    uint16_t live[4];
    live[0] = toy_rgb565(0, 220, 255);   /* cyan */
    live[1] = toy_rgb565(80, 230, 120);  /* green */
    live[2] = toy_rgb565(255, 180, 40);  /* amber */
    live[3] = toy_rgb565(240, 100, 255); /* magenta */
    const uint16_t frozen_col = toy_rgb565(235, 60, 50);
    const uint16_t track_col = toy_rgb565(40, 48, 72);
    const uint16_t ceil_col = toy_rgb565(150, 170, 200);
    const uint16_t text_col = toy_rgb565(150, 170, 200);

    double growth[4];
    growth[0] = widen16(s->sum16) - widen16(s->start16);
    growth[1] = widen32(s->sum32) - widen32(s->start32);
    growth[2] = d64(s->sum64) - d64(s->start64);
    growth[3] = (d64(s->aug_head) - d64(s->start64)) + d64(s->aug_tail);

    int cx = toy_cx(fb), cy = toy_cy(fb), r = toy_r(fb);
    int sc = r >= 150 ? 2 : 1;
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, r - 2, toy_rgb565(28, 34, 54));
    toy_fb_text(fb, cx - text_w("STAGNATION", sc) / 2, cy - r + 10 * sc, sc, text_col, "STAGNATION");
    char buf[24];
    fmt_u64(buf, 0, s->steps);
    int sx = toy_fb_text(fb, cx - (text_w("STEP ", sc) + text_w(buf, sc)) / 2,
                         cy - r + 19 * sc, sc, text_col, "STEP ");
    toy_fb_text(fb, sx, cy - r + 19 * sc, sc, text_col, buf);

    int baseline = cy + (int)(0.44 * (double)r);
    int plot_h = (int)(0.92 * (double)r);
    int bw = (int)(0.18 * (double)r);
    static const double xoff[4] = { -0.54, -0.18, 0.18, 0.54 };
    for (int i = 0; i < 4; i++) {
        int xc = cx + (int)(xoff[i] * (double)r);
        int x0 = xc - bw / 2, x1 = x0 + bw;
        /* Track outline up to the common plot top. */
        toy_fb_line(fb, x0, baseline, x0, baseline - plot_h, track_col);
        toy_fb_line(fb, x1, baseline, x1, baseline - plot_h, track_col);
        toy_fb_line(fb, x0, baseline - plot_h, x1, baseline - plot_h, track_col);
        /* Ceiling tick: where a plain lane of this design freezes. */
        int cy_ceil = baseline - (int)(ceiling[i] / plot_units * (double)plot_h);
        toy_fb_line(fb, x0 - 3 * sc, cy_ceil, x1 + 3 * sc, cy_ceil, ceil_col);
        /* Bar, clamped to the plot: the augmented lane passes its
         * ceiling and eventually the plot top in a long live run. */
        int bh = (int)(growth[i] / plot_units * (double)plot_h);
        if (bh > plot_h) bh = plot_h;
        if (bh < 0) bh = 0;
        uint16_t col = frozen[i] ? frozen_col : live[i];
        for (int x = x0; x <= x1; x++)
            toy_fb_line(fb, x, baseline, x, baseline - bh, col);
        /* Lane label under the baseline; freeze step under it. */
        toy_fb_text(fb, xc - text_w(label[i], sc) / 2, baseline + 4 * sc, sc, text_col, label[i]);
        if (frozen[i]) {
            fmt_u64(buf, 'F', frozen[i]);
            toy_fb_text(fb, xc - text_w(buf, sc) / 2, baseline + 12 * sc, sc, frozen_col, buf);
        }
    }
    toy_fb_line(fb, cx - (int)(0.68 * (double)r), baseline, cx + (int)(0.68 * (double)r), baseline, track_col);
}

const toy_cart_t toy_cart_stagnation = {
    "stagnation", "Stagnation",
    "Adding 1 forever: binary16/32/64 freeze at 2^p. The augmented lane keeps the change.",
    sizeof(stag_state), 100, 4000, 500, stag_reset, stag_step, stag_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
