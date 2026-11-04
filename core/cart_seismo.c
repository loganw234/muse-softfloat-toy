/* cart_seismo.c - a desk seismometer on the synthetic IMU6 stream.
 *
 * Signal binding (descriptor tail): IMU6, SYNTHETIC proof, scenario 1
 * ("desk"), 100 Hz, seed 0x5eed, 10 samples per step. One step is
 * therefore exactly 0.1 s of stream time (10 samples / 100 Hz), an
 * index-derived count, never clock-derived. 10 samples/step makes
 * the t=30 s desk tap (sample 3000, core/toy_signal.c) the first
 * sample of step 301, so a 400-step proof (40 s, 4000 samples)
 * pins pre-tap hashes (reports to step 300), the tap step itself
 * (detector fields, hashed from the step-310 report on), and 10 s
 * of post-tap ringing return, in 400 steps instead of 4000.
 *
 * Seismometer quantity: the vertical acceleration az (IMU6 channel
 * 2), gravity-removed. The device sits flat on a desk, so z is the
 * vertical axis a desk seismometer reads; in scenario 1 az carries
 * the constant gravity component (15864 LSB), the ~5 mg band-
 * limited tremor, and the tap at full amplitude (~0.49 g), while
 * ax/ay additionally carry the 0.25 g slow tilt circle that would
 * swamp a horizontal channel, and ax carries only a quarter of the
 * tap. feed() derives, per sample, in softfloat:
 *   x    = az_counts / 16384            (g; scale from f64_dec)
 *   dyn  = x - baseline                 (the displayed signal)
 *   baseline += alpha * dyn             (one-pole detrending)
 * with alpha = f64_dec("0.00390625") = 1/256, i.e. a 256-sample
 * (2.56 s) time constant, ~0.06 Hz cutoff: gravity and slow tilt
 * drift are tracked out, tremor and the tap pass. The first fed
 * sample initialises baseline exactly (dyn = 0 for it), so there
 * is no synthetic startup transient. All of this lives in state.
 * There is no step-0 feed: before the first feed the embedded
 * label says KIND_NONE and render() shows NO SIG, never a flat
 * trace. Provenance is rendered from that hashed label.
 *
 * Event detector (exact): threshold thr = f64_dec("0.05") g on
 * |dyn|, rising edge only (|dyn_prev| <= thr < |dyn| on absolute
 * patterns, exact for non-negative values), with a 50-sample
 * (0.5 s) refractory after the previous event so the tap's 20 Hz
 * ring counts once, not once per half-cycle. An event records the
 * sample index and the step it precedes (steps completed + 1) in
 * state, plus a running event count and |dyn| peak.
 *
 * Spectrum: toy_fft binary64, N = 256 over the 256-sample dyn
 * history ring (2.56 s at 100 Hz; bin width 0.390625 Hz), Hann
 * windowed cart-side (weights 0.5 - 0.5*cosPi(2n/255) computed in
 * reset() into state, per docs/study-fft.md rule-of-two: this is
 * the second FFT consumer and the first to window). The transform
 * runs in step() every 10 steps (1.0 s of stream, 100 new samples)
 * once the ring is full - an instrument-rate refresh, not per-step.
 * The FFT step is a deliberate, costed spike (study-fft section 5;
 * see the parcel report for this cart's measured split); the
 * proof run contains exactly 38 transforms (steps 30,40,..,400;
 * steps 10/20 are still filling and run none). Magnitudes for
 * bins 0..128 and the argmax bin are state, so the proof hash
 * covers the spectrum.
 *
 * The tap (20 Hz ringing, bin ~51) enters the spectrum window at
 * the step-310 transform (window samples 2844..3099), perturbs
 * that and the step-320 epoch, and has aged out of the 256-sample
 * window by step 330. Display shows bins 1..64 (to 25 Hz).
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"
#include "toy_fft.h"

#define SE_N 256          /* history / FFT length, samples (2.56 s) */
#define SE_BINS 129       /* magnitudes kept: bins 0..N/2          */
#define SE_FFT_EVERY 10u  /* steps between transforms (1.0 s)      */
#define SE_REFRACTORY 50u /* samples; one count per ring burst     */

typedef struct {
    toy_sig_label_t label; /* provenance, hashed (32 bytes)         */
    f64 hist[SE_N];        /* dyn ring, g, oldest..newest via head  */
    f64 win[SE_N];         /* Hann weights, from reset()            */
    f64 re[SE_N];          /* FFT buffers (transformed in place)    */
    f64 im[SE_N];
    f64 mag[SE_BINS];      /* |X[k]| for k = 0..128, from step()    */
    f64 baseline;          /* one-pole gravity estimate, g          */
    f64 alpha;             /* 1/256, from f64_dec at reset          */
    f64 scale_g;           /* 16384 LSB/g divisor, from f64_dec     */
    f64 thr_g;             /* detector threshold 0.05 g, f64_dec    */
    f64 last_dyn;          /* newest gravity-removed sample, g      */
    f64 prev_abs;          /* |dyn| of the previous sample          */
    f64 peak_abs;          /* max |dyn| since reset, g              */
    uint64_t steps;        /* steps completed                       */
    uint64_t sample_seq;   /* next sample index to be fed           */
    uint64_t hist_head;    /* next ring write slot                  */
    uint64_t filled;       /* samples in ring, capped at SE_N       */
    uint64_t peak_bin;     /* argmax over bins 1..128               */
    uint64_t fft_count;
    uint64_t event_count;  /* detector events so far                */
    uint64_t last_event_step;   /* step the newest event preceded   */
    uint64_t last_event_sample; /* its 0-based sample index         */
    uint64_t feed_calls;
    int32_t last_az;       /* newest raw az counts                  */
    uint32_t baseline_init;
    uint32_t pad0;
} seismo_state;

static f64 se_from_i32(int32_t v) { return (f64)toy_f64_bits_from_i32(v); }

static void seismo_reset(void *st_)
{
    seismo_state *s = (seismo_state *)st_;
    toy_memset(s, 0, sizeof *s);
    toy_sig_label_reset(&s->label); /* NONE: NO SIG until fed */
    s->alpha = f64_dec("0.00390625");
    s->scale_g = f64_dec("16384");
    s->thr_g = f64_dec("0.05");
    /* Hann window 0.5 - 0.5*cosPi(2n/(N-1)), softfloat, in state. */
    f64 denom = se_from_i32(SE_N - 1);
    for (int n = 0; n < SE_N; n++) {
        f64 ang = f64_div(se_from_i32(2 * n), denom);
        s->win[n] = f64_sub(F64_HALF, f64_mul(F64_HALF, f64_cos_pi(ang)));
    }
}

static void seismo_feed(void *st_, const int16_t *samples, uint32_t n,
                        uint32_t channels, const toy_sig_info_t *info,
                        uint32_t flags)
{
    seismo_state *s = (seismo_state *)st_;
    toy_sig_label_feed(&s->label, info, n, flags);
    s->feed_calls++;
    if (!samples || channels < 3u) return;
    for (uint32_t i = 0; i < n; i++) {
        int32_t az = samples[(size_t)i * channels + 2u];
        s->last_az = az;
        f64 x = f64_div(se_from_i32(az), s->scale_g);
        f64 dyn;
        if (!s->baseline_init) {
            s->baseline = x;
            s->baseline_init = 1u;
            dyn = F64_ZERO;
        } else {
            dyn = f64_sub(x, s->baseline);
            s->baseline = f64_fma(s->alpha, dyn, s->baseline);
        }
        s->last_dyn = dyn;
        /* Detector: rising edge over thr on |dyn| (non-negative
         * patterns compare as integers exactly), refractory so one
         * ringing burst counts once. */
        f64 adyn = f64_abs(dyn);
        if (adyn > s->peak_abs) s->peak_abs = adyn;
        if (s->prev_abs <= s->thr_g && adyn > s->thr_g &&
            (s->event_count == 0 ||
             s->sample_seq - s->last_event_sample >= SE_REFRACTORY)) {
            s->event_count++;
            s->last_event_sample = s->sample_seq;
            s->last_event_step = s->steps + 1u; /* step this feed precedes */
        }
        s->prev_abs = adyn;
        s->hist[s->hist_head] = dyn;
        s->hist_head = (s->hist_head + 1u) % SE_N;
        if (s->filled < SE_N) s->filled++;
        s->sample_seq++;
    }
}

/* One spectrum epoch: Hann-window the full ring oldest-first,
 * transform, store magnitudes and the argmax bin. All softfloat;
 * magnitudes are >= +0, so integer comparison of their patterns
 * is exact numeric comparison. */
static void run_spectrum(seismo_state *s)
{
    uint64_t start = s->hist_head; /* ring full: head is oldest */
    for (uint64_t i = 0; i < SE_N; i++)
        s->re[i] = f64_mul(s->hist[(start + i) % SE_N], s->win[i]);
    toy_memset(s->im, 0, sizeof s->im); /* bit pattern 0 is +0.0 */
    (void)toy_fft_forward(s->re, s->im, SE_N);
    for (int k = 0; k < SE_BINS; k++)
        s->mag[k] = f64_sqrt(f64_fma(s->re[k], s->re[k],
                                     f64_mul(s->im[k], s->im[k])));
    uint64_t best = 1;
    for (uint64_t k = 2; k < SE_BINS; k++)
        if (s->mag[k] > s->mag[best]) best = k;
    s->peak_bin = best;
    s->fft_count++;
}

static void seismo_step(void *st_)
{
    seismo_state *s = (seismo_state *)st_;
    s->steps++;
    if ((s->steps % SE_FFT_EVERY) == 0 && s->filled >= SE_N)
        run_spectrum(s);
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

/* Monotone piecewise-linear log2 from the bit pattern (display
 * scaling only; no libm, freestanding-safe, as cart_orbitspec). */
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

static int text_len(const char *t)
{
    int n = 0;
    while (t[n]) n++;
    return n;
}

/* Provenance from the hashed label (study-signals section 7):
 * synthetic is named synthetic, never dressed as measured. */
static const char *provenance(const seismo_state *s)
{
    if (s->label.kind == TOY_SIG_KIND_SYNTHETIC) return "SYNTH";
    if (s->label.kind == TOY_SIG_KIND_LIVE) return "LIVE";
    if (s->label.kind == TOY_SIG_KIND_REPLAY)
        return s->label.orig_kind == TOY_SIG_KIND_LIVE ? "REPLAY-LIVE"
                                                       : "REPLAY-SYN";
    return "NO SIG";
}

static void seismo_render(const void *st_, toy_fb_t *fb)
{
    const seismo_state *s = (const seismo_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    int sc = R >= 150 ? 2 : 1;
    uint16_t dim = toy_rgb565(110, 125, 155);
    uint16_t bright = toy_rgb565(200, 215, 240);
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));

    const char *title = "SEISMO";
    toy_fb_text(fb, cx - (text_len(title) * 4 * sc - sc) / 2, cy - R + 10 * sc,
                sc, toy_rgb565(150, 170, 200), title);
    char buf[40];
    int n = 0;
    n = append_str(buf, n, provenance(s));
    if (s->label.kind != TOY_SIG_KIND_NONE) n = append_str(buf, n, " 100HZ AZ");
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, cy - R + 19 * sc, sc,
                toy_rgb565(255, 190, 60), buf);

    int x0 = cx - (R * 68) / 100, x1 = cx + (R * 68) / 100;
    int ymid = cy - (R * 28) / 100;
    int halfh = (R * 26) / 100;
    int yb = cy + (R * 56) / 100;
    /* Axes are furniture, not data: drawn in every state. */
    toy_fb_line(fb, x0 - 2, ymid, x1 + 2, ymid, toy_rgb565(70, 85, 120));
    toy_fb_line(fb, x0 - 2, yb, x1 + 2, yb, toy_rgb565(70, 85, 120));

    if (s->label.kind == TOY_SIG_KIND_NONE) {
        /* No source has fed yet: say so, draw no fake trace. */
        const char *msg = "NO SIG";
        toy_fb_text(fb, cx - (text_len(msg) * 4 * sc - sc) / 2, ymid - 3 * sc,
                    sc, toy_rgb565(255, 120, 90), msg);
        return;
    }

    /* Trace gain (display only): full scale +/-0.1 g, so the
     * ~5 mg tremor reads as a visible band and the detector
     * threshold sits at half height; the 0.42 g tap peak clips
     * at the rails like a saturated seismogram - state, detector
     * and spectrum keep the unclipped signal. */
    const double trace_fs = 0.1;
    int thr_px = (int)(d64(s->thr_g) / trace_fs * (double)halfh);
    uint16_t rail = toy_rgb565(60, 70, 95);
    for (int x = x0; x <= x1; x += 4) {
        toy_fb_px(fb, x, ymid - thr_px, rail);
        toy_fb_px(fb, x, ymid + thr_px, rail);
    }

    /* Scrolling tremor trace: the 256-sample ring, oldest first,
     * linear across halfh at the trace gain above, clamped. */
    if (s->filled > 1) {
        uint64_t start = (s->hist_head + SE_N - s->filled) % SE_N;
        int px = x0, py = ymid;
        for (uint64_t i = 0; i < s->filled; i++) {
            double v = d64(s->hist[(start + i) % SE_N]) / trace_fs;
            if (v > 1.0) v = 1.0;
            if (v < -1.0) v = -1.0;
            int x = x0 + (int)((i * (uint64_t)(x1 - x0)) / (s->filled - 1));
            int y = ymid - (int)(v * (double)halfh);
            if (i > 0) toy_fb_line(fb, px, py, x, y, toy_rgb565(60, 230, 150));
            px = x;
            py = y;
        }
        toy_fb_disc(fb, px, py, 1, 0xffffu);
        /* Event marker while the event sample is in the window. */
        if (s->event_count > 0) {
            uint64_t oldest = s->sample_seq - s->filled;
            if (s->last_event_sample >= oldest &&
                s->last_event_sample < s->sample_seq) {
                uint64_t pos = s->last_event_sample - oldest;
                int x = x0 + (int)((pos * (uint64_t)(x1 - x0)) / (s->filled - 1));
                toy_fb_line(fb, x, ymid - halfh, x, ymid + halfh,
                            toy_rgb565(255, 190, 60));
            }
        }
    }
    toy_fb_text(fb, x0, ymid - halfh - 8 * sc, sc, dim, "AZ DYN FS 0.1G");

    /* Spectrum bars, bins 1..64 (0.39 Hz/bin, to 25 Hz; the tap
     * rings at 20 Hz, bin ~51). Log range 2^10 below the peak. */
    int maxh = (R * 42) / 100;
    if (s->fft_count > 0 && s->peak_bin >= 1 && s->peak_bin < SE_BINS &&
        s->mag[s->peak_bin] != F64_ZERO) {
        double top = log2_display(s->mag[s->peak_bin]);
        uint16_t bar = toy_rgb565(40, 210, 190);
        uint16_t peakc = toy_rgb565(255, 190, 60);
        for (int k = 1; k <= 64; k++) {
            double fr = (log2_display(s->mag[k]) - (top - 10.0)) / 10.0;
            if (fr <= 0.0) continue;
            if (fr > 1.0) fr = 1.0;
            int h = (int)(fr * (double)maxh);
            if (h <= 0) continue;
            int bx = x0 + ((k - 1) * (x1 - x0)) / 63;
            toy_fb_line(fb, bx, yb - 1, bx, yb - h,
                        (uint64_t)k == s->peak_bin ? peakc : bar);
        }
    }
    toy_fb_text(fb, x0, yb - maxh - 8 * sc, sc, dim, "FFT 256 HANN");

    /* Labels, from stored integers only. */
    n = 0;
    if (s->fft_count == 0) {
        n = append_str(buf, n, "FILL ");
        n = append_u64(buf, n, s->filled);
        n = append_str(buf, n, "/256");
    } else {
        uint64_t ft = (s->peak_bin * 1000u) / 256u; /* Hz x10, trunc */
        n = append_str(buf, n, "BIN ");
        n = append_u64(buf, n, s->peak_bin);
        buf[n++] = ' ';
        n = append_u64(buf, n, ft / 10u);
        buf[n++] = '.';
        n = append_u64(buf, n, ft % 10u);
        n = append_str(buf, n, "HZ");
    }
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, yb + 5 * sc, sc, bright, buf);
    n = 0;
    n = append_str(buf, n, "EV ");
    n = append_u64(buf, n, s->event_count);
    if (s->event_count > 0) {
        n = append_str(buf, n, " AT ");
        n = append_u64(buf, n, s->last_event_step);
    }
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, yb + 13 * sc, sc,
                s->event_count > 0 ? toy_rgb565(255, 190, 60) : dim, buf);
}

/* demo 1 step/frame: at the nominal 30 fps the trace plays at
 * 3x stream rate (0.1 s per frame) - a readable seismogram
 * time-lapse, spectrum refreshing every 10 frames; the slowest
 * target's FFT-step spike is costed in the parcel report.
 * proof 400 steps x 10 samples = 40 s: tap sample 3000 is the
 * first sample of step 301; reports every 10 steps pin step 300
 * (pre-tap), 310 (tap in window + detector fired), 320/330
 * (spectrum perturbed, then tap aged out) and 400 (settled). */
const toy_cart_t toy_cart_seismo = {
    "seismo", "Desk Seismometer",
    "Desk tremor and one tap at 30 s, Fourier-read. Synthetic feed.",
    sizeof(seismo_state), 1, 400, 10,
    seismo_reset, seismo_step, seismo_render,
    0, 0, 0, 0, 0, 0,
    0, /* state_version */
    seismo_feed,
    { TOY_SIG_DEVICE_IMU6, TOY_SIG_KIND_SYNTHETIC, 1u, 100u, 10u,
      UINT64_C(0x5eed) }
};
