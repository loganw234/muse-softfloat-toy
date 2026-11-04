/* cart_orbitspec.c - the figure-8's line spectrum, live.
 *
 * The Chenciner-Montgomery choreography (same ICs and Yoshida-4
 * integrator as cart_fig8.c, copied here so the cart is
 * self-contained) runs at dt = 0.01. Every 21 steps the cart
 * samples r0 = |x0|, body 0's distance from the barycentre,
 * into a 256-sample ring; every 210 steps it runs
 * toy_fft_forward over the ring (oldest first, zero-padded
 * while filling) and stores the magnitudes and the argmax bin.
 *
 * Why r0 and not a coordinate: x0(t) is nearly a pure sinusoid
 * (offline check: harmonics below 2% of the fundamental), so
 * its spectrum is one line and little else. r0 pulsates as
 * body 0 rounds each lobe of the 8, and by the choreography's
 * central symmetry, x(t + T/2) = -x(t), so r0's own period is
 * T/2 ~= 3.163 with a decaying harmonic ladder (measured,
 * offline: 1, 0.32, 0.11, 0.056, 0.036, ...). The window spans
 * 256*21*0.01 = 53.76 time units = 17.00 r0 periods (8.50
 * choreography periods), so the expected dominant bin is 17,
 * with harmonics at 34, 51, 68, 85, 102. The near-integer
 * period count is deliberate: it keeps rectangular-window
 * leakage low when the window is full.
 *
 * Windowing: rectangular, plus subtraction of the filled
 * window's mean (softfloat, at FFT time) so the large DC of a
 * distance does not sit in bin 0. Rectangular means leakage
 * skirts around each line are real and visible on the log
 * bars, worst while the window is still filling (a short
 * aperture is a wide line - that sharpening as the window
 * fills is the phenomenon). A Hann window would lower the
 * skirts at twice the line width; with 17-bin line spacing
 * resolution is not the constraint, but rectangular keeps the
 * partially-filled aperture honest and matches the study-fft
 * plan for this cart. Cost of honesty: r0 harmonics above the
 * 7th (of T/2) alias at this sample stride; they are at the
 * ~1% level and fold into the upper displayed bins.
 *
 * FFT cadence: one 256-point transform every 210 steps (every
 * 10 new samples). The transform step is a deliberate,
 * measured spike (see parcel report); between spikes a step
 * is one Yoshida-4 step plus, every 21st step, one sqrt.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"
#include "toy_fft.h"

#define OS_N 256
#define OS_BINS 129 /* magnitudes kept: bins 0..N/2 */
#define SAMPLE_STRIDE 21u
#define FFT_EVERY 210u
#define TRAIL 192

typedef struct {
    f64 x[3][2], v[3][2];
    f64 w1, w0, dt;
    f64 hist[OS_N];   /* r0 sample ring */
    f64 re[OS_N];     /* FFT buffers (transformed in place) */
    f64 im[OS_N];
    f64 mag[OS_BINS]; /* |X[k]| for k = 0..128, from step() */
    f64 filled_f;     /* sample count as f64, for the mean */
    f64 trail[3][TRAIL][2];
    uint64_t steps;
    uint64_t sample_head; /* next ring write slot */
    uint64_t filled;      /* samples in ring, capped at OS_N */
    uint64_t peak_bin;    /* argmax over bins 1..128 */
    uint64_t fft_count;
} orbitspec_state;

static void accel(const orbitspec_state *s, f64 a[3][2])
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

static void leapfrog(orbitspec_state *s, f64 h)
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

/* One spectrum epoch: fill re/im from the ring oldest-first
 * (zero-padded past the filled count), subtract the filled
 * mean, transform, store magnitudes and the argmax bin. All
 * softfloat; magnitudes are >= +0, so comparing their bit
 * patterns as integers is exact numeric comparison. */
static void run_spectrum(orbitspec_state *s)
{
    uint64_t filled = s->filled;
    if (filled == 0) return;
    uint64_t start = (s->sample_head + OS_N - filled) % OS_N;
    f64 sum = F64_ZERO;
    for (uint64_t i = 0; i < filled; i++)
        sum = f64_add(sum, s->hist[(start + i) % OS_N]);
    f64 mean = f64_div(sum, s->filled_f);
    for (uint64_t i = 0; i < OS_N; i++)
        s->re[i] = (i < filled)
            ? f64_sub(s->hist[(start + i) % OS_N], mean)
            : F64_ZERO;
    toy_memset(s->im, 0, sizeof s->im); /* bit pattern 0 is +0.0 */
    (void)toy_fft_forward(s->re, s->im, OS_N);
    for (int k = 0; k < OS_BINS; k++)
        s->mag[k] = f64_sqrt(f64_fma(s->re[k], s->re[k],
                                     f64_mul(s->im[k], s->im[k])));
    uint64_t best = 1;
    for (uint64_t k = 2; k < OS_BINS; k++)
        if (s->mag[k] > s->mag[best]) best = k;
    s->peak_bin = best;
    s->fft_count++;
}

static void orbitspec_reset(void *st_)
{
    orbitspec_state *s = (orbitspec_state *)st_;
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

static void orbitspec_step(void *st_)
{
    orbitspec_state *s = (orbitspec_state *)st_;
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
    if ((s->steps % SAMPLE_STRIDE) == 0) {
        f64 r0 = f64_sqrt(f64_fma(s->x[0][0], s->x[0][0],
                                  f64_mul(s->x[0][1], s->x[0][1])));
        s->hist[s->sample_head] = r0;
        s->sample_head = (s->sample_head + 1u) % OS_N;
        if (s->filled < OS_N) {
            s->filled++;
            s->filled_f = f64_add(s->filled_f, F64_ONE); /* exact <= 256 */
        }
    }
    if ((s->steps % FFT_EVERY) == 0) run_spectrum(s);
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Monotone piecewise-linear log2 from the bit pattern:
 * exponent + mantissa fraction. Display scaling only; no
 * libm, so the same renderer builds freestanding. */
static double log2_display(f64 a)
{
    uint64_t u = (uint64_t)a;
    if (u == 0) return -1.0e9;
    double e = (double)((int)((u >> 52) & 0x7ffu) - 1023);
    double frac = (double)(u & 0xfffffffffffffull) / 4503599627370496.0;
    return e + frac;
}

static int append_u64(char *buf, int pos, uint64_t v)
{
    char dig[20];
    int n = 0;
    if (v == 0) dig[n++] = '0';
    while (v > 0) { dig[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n > 0) buf[pos++] = dig[--n];
    return pos;
}

static int append_str(char *buf, int pos, const char *t)
{
    while (*t) buf[pos++] = *t++;
    return pos;
}

static void orbitspec_render(const void *st_, toy_fb_t *fb)
{
    const orbitspec_state *s = (const orbitspec_state *)st_;
    static const uint16_t body_col[3] = { 0x07ffu, 0xfc60u, 0xf81fu }; /* cyan amber magenta */
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    int sc = R >= 150 ? 2 : 1;
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));

    /* Spectrum bars, bins 1..110 (harmonics to bin 102).
     * Heights: log display range of 2^10 below the stored
     * peak magnitude, from stored magnitudes only. */
    int x0 = cx - (R * 66) / 100, x1 = cx + (R * 66) / 100;
    int yb = cy + (R * 56) / 100;
    int maxh = (R * 72) / 100;
    toy_fb_line(fb, x0 - 2, yb, x1 + 2, yb, toy_rgb565(70, 85, 120));
    if (s->fft_count > 0 && s->peak_bin >= 1 && s->peak_bin < OS_BINS
        && s->mag[s->peak_bin] != F64_ZERO) {
        double top = log2_display(s->mag[s->peak_bin]);
        uint16_t bar = toy_rgb565(40, 210, 190);
        uint16_t peakc = toy_rgb565(255, 190, 60);
        for (int k = 1; k <= 110; k++) {
            double fr = (log2_display(s->mag[k]) - (top - 10.0)) / 10.0;
            if (fr <= 0.0) continue;
            if (fr > 1.0) fr = 1.0;
            int h = (int)(fr * (double)maxh);
            if (h <= 0) continue;
            int bx = x0 + ((k - 1) * (x1 - x0)) / 109;
            toy_fb_line(fb, bx, yb - 1, bx, yb - h,
                        (uint64_t)k == s->peak_bin ? peakc : bar);
        }
    }

    /* Orbit inset above the bars: the thing being analysed.
     * The line from the inset centre to body 0 is r0 itself. */
    int icx = cx, icy = cy - (R * 52) / 100;
    double iscale = (double)R * 0.34 / 1.35;
    size_t head = (size_t)((s->steps / 4u) % TRAIL);
    int bx[3], by[3];
    for (int i = 0; i < 3; i++) {
        for (int t = 0; t < TRAIL; t++) {
            size_t idx = (head + TRAIL - (size_t)t) % TRAIL;
            int x = icx + (int)(d64(s->trail[i][idx][0]) * iscale);
            int y = icy - (int)(d64(s->trail[i][idx][1]) * iscale);
            unsigned fade = 255u - (unsigned)(t * 200 / TRAIL);
            unsigned p = body_col[i];
            uint16_t c = (uint16_t)(((((p >> 11) & 31u) * fade) >> 8 << 11) |
                                    ((((p >> 5) & 63u) * fade) >> 8 << 5) |
                                    (((p & 31u) * fade) >> 8));
            toy_fb_px(fb, x, y, c);
        }
        bx[i] = icx + (int)(d64(s->x[i][0]) * iscale);
        by[i] = icy - (int)(d64(s->x[i][1]) * iscale);
    }
    toy_fb_line(fb, icx, icy, bx[0], by[0], toy_rgb565(90, 150, 170));
    toy_fb_px(fb, icx, icy, toy_rgb565(150, 170, 200));
    for (int i = 0; i < 3; i++) {
        toy_fb_disc(fb, bx[i], by[i], R / 60 + 1, body_col[i]);
        toy_fb_disc(fb, bx[i], by[i], 1, 0xffffu);
    }

    /* Labels, from stored integers only. */
    const char *title = "ORBITSPEC";
    int tlen = 0;
    while (title[tlen]) tlen++;
    toy_fb_text(fb, cx - (tlen * 4 * sc - sc) / 2, cy - R + 10 * sc, sc,
                toy_rgb565(150, 170, 200), title);
    char buf[32];
    int n = 0;
    if (s->fft_count == 0) {
        n = append_str(buf, n, "FILLING");
    } else {
        n = append_str(buf, n, "PEAK BIN ");
        n = append_u64(buf, n, s->peak_bin);
    }
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, yb + 5 * sc, sc,
                toy_rgb565(200, 215, 240), buf);
    n = 0;
    n = append_str(buf, n, "R0 FILL ");
    n = append_u64(buf, n, s->filled);
    n = append_str(buf, n, "/256");
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, yb + 13 * sc, sc,
                toy_rgb565(110, 125, 155), buf);
}

const toy_cart_t toy_cart_orbitspec = {
    "orbitspec", "Orbital Spectrum",
    "The figure-8's radius, Fourier-read: lines at whole multiples.",
    sizeof(orbitspec_state), 4, 6000, 500,
    orbitspec_reset, orbitspec_step, orbitspec_render, 0, 0, 0, 0, 0, 0, 0, 0, { 0 }
};
