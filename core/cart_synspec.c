/* cart_synspec.c - a parametric synth, Fourier-read live.
 *
 * Signal-deaf by design (no feed): the tone is synthesised
 * inside state, so the whole instrument is re-derivable from
 * reset + the proof script alone. A MIC feed scenario is a
 * fixed 440 Hz tone a user cannot steer; a controllable
 * synth is the gallery piece, and a live microphone is kit
 * work behind the same feed seam later.
 *
 * Synthesis maths. Conceptual sample rate Fs = 8192 Hz,
 * FFT length N = 256, so the bin width is exactly 32 Hz.
 * Each step() generates one block of 32 samples into a
 * 256-sample ring. The oscillator is a phase accumulator
 * held in cycles, phase in [0,1): per sample the value is
 * taken at the current phase, then
 *     phase += dphase;  if (phase >= 1) phase -= 1;
 * with dphase = f / Fs, all binary64 via toy_math.h.
 * dphase is recomputed in apply() when the frequency
 * changes (one softfloat divide); step() only adds.
 * Frequency changes are phase-continuous: apply() never
 * resets phase, like a real VCO being retuned.
 *
 * Coherence. A tone is bin-coherent when f is a multiple
 * of 32 Hz (an integer number of cycles in the window).
 * The default 256 Hz (bin 8) and every proof tone are
 * coherent by construction. The dial, however, steps in
 * 0.256 Hz increments (below), so live tones are usually
 * NOT coherent: a cart-side Hann window (own copy, as in
 * cart_seismo; rule-of-two -- no shared helper added)
 * trades a 4-bin main lobe for low leakage skirts, which
 * is the right trade for arbitrary dial frequencies and
 * for reading harmonic ratios off neighbouring bins.
 * With a coherent tone the Hann coherent gain is the
 * same at every harmonic bin, so magnitude ratios read
 * the recipe ratios directly.
 *
 * Waveforms (ENUM param, harmonic recipes, softfloat):
 *   SINE:   sin(2pi p)
 *   SQUARE: sin + (1/3) sin(3.) + (1/5) sin(5.)  (odd)
 *   SAW:    sin - (1/2) sin(2.) + (1/3) sin(3.)
 *                 - (1/4) sin(4.)
 * where sin(k.) means f64_sin_pi(2*k*p) and the 1/k
 * coefficients are softfloat divisions stored at reset.
 * Truncated Fourier sums, not time-domain switching: no
 * aliasing from a discontinuous edge, and the spectrum
 * shows exactly the harmonics in the recipe (Gibbs ripple
 * in the trace is the truncation, honestly visible).
 * Fundamental is capped at 1024 Hz (bin 32) so the SQUARE
 * 3rd harmonic (3072 Hz) stays below Nyquist (4096 Hz);
 * its 5th can alias only above ~819 Hz fundamental.
 *
 * Dial semantics (FIRST dial consumer; defined exactly).
 * TOY_EV_DIAL, id 0 only (the device itself); other ids
 * are ignored. v is a signed milliturn delta (1000 = one
 * full turn). In apply(), integer arithmetic only:
 *     freq_mhz = clamp(freq_mhz + (int64)v * 256,
 *                       32000, 1024000)
 * i.e. one full turn = +256 Hz = +8 bins, one milliturn
 * = 0.256 Hz; range 32..1024 Hz (bins 1..32). The new
 * millihertz integer is stored in state and dphase is
 * re-derived from it through toy_f64_bits_from_milli;
 * the renderer's frequency readout formats that same
 * integer, never a float round-trip. Linear (not
 * exponential/octave) so the mapping is one exact
 * integer multiply, trivially replayable; coarse musical
 * tuning via multiple turns is the live producer's job.
 *
 * Spectrum: toy_fft N=256 over the ring, Hann-windowed,
 * every 8 steps once full (8*32 = 256 fresh samples --
 * non-overlapping epochs, each window wholly one tone
 * after any change settles within one epoch). Magnitudes
 * for bins 0..128 and the argmax bin are state.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"
#include "toy_fft.h"

#define SYN_N 256
#define SYN_BINS 129
#define SYN_PER_STEP 32u
#define SYN_FFT_EVERY 8u
#define SYN_FS_HZ 8192
#define SYN_FREQ_MIN_MHZ 32000
#define SYN_FREQ_MAX_MHZ 1024000
#define SYN_FREQ_DEF_MHZ 256000
#define SYN_MHZ_PER_MILLITURN 256
#define SYN_PARAM_WAVE 0u
#define SYN_PARAM_LEVEL 1u
#define SYN_LEVEL_MIN 10
#define SYN_LEVEL_MAX 100

/* State bytes: 4 arrays of 256 f64 (8192) + mag 129*8
 * (1032) + 12 f64 scalars (96) + 7 uint64 (56) + 4 int32/
 * uint32 (16) = 9392 bytes, a multiple of 8, fields in
 * descending-alignment order; 14% of TOY_MAX_STATE_SIZE. */
typedef struct {
    f64 hist[SYN_N];
    f64 win[SYN_N];
    f64 re[SYN_N];
    f64 im[SYN_N];
    f64 mag[SYN_BINS];
    f64 phase;   /* cycles, in [0,1) */
    f64 dphase;  /* f / 8192, cycles per sample */
    f64 amp;     /* level param effect, 0.10..1.00 */
    f64 h2, h3, h4, h5;       /* harmonic numbers as f64 */
    f64 inv2, inv3, inv4, inv5; /* 1/k recipe coefficients */
    f64 last_sample;
    uint64_t steps;
    uint64_t sample_seq;
    uint64_t hist_head;
    uint64_t filled;
    uint64_t peak_bin;
    uint64_t fft_count;
    uint64_t dial_events;
    int32_t freq_mhz;  /* stored frequency, millihertz */
    int32_t level_pct; /* level param effect, 10..100 */
    uint32_t wave;     /* waveform enum effect, 0..2 */
    uint32_t pad0;
} synspec_state;

static f64 syn_from_i32(int32_t v) { return (f64)toy_f64_bits_from_i32(v); }
static int syn_ge(f64 a, f64 b)
{
    msf_flags_t f = 0;
    return msf64_compare_quiet_greater_equal(a, b, &f);
}

/* Store a millihertz frequency and re-derive dphase. */
static void syn_set_freq(synspec_state *s, int32_t freq_mhz)
{
    s->freq_mhz = freq_mhz;
    f64 f_hz = (f64)toy_f64_bits_from_milli(freq_mhz);
    s->dphase = f64_div(f_hz, syn_from_i32(SYN_FS_HZ));
}

static const char *const syn_wave_names[] = { "SINE", "SQUARE", "SAW" };

/* apply(): events -> state effects only (seam rules). */
static void synspec_apply(void *st_, const toy_event_t *ev)
{
    synspec_state *s = (synspec_state *)st_;
    switch (ev->type) {
    case TOY_EV_DIAL: {
        if (ev->id != 0u) break; /* only the device dial */
        /* Integer mapping (header): 1 milliturn = 256 mHz. */
        int64_t nf = (int64_t)s->freq_mhz +
                     (int64_t)ev->v * SYN_MHZ_PER_MILLITURN;
        if (nf < SYN_FREQ_MIN_MHZ) nf = SYN_FREQ_MIN_MHZ;
        if (nf > SYN_FREQ_MAX_MHZ) nf = SYN_FREQ_MAX_MHZ;
        s->dial_events++;
        if ((int32_t)nf != s->freq_mhz)
            syn_set_freq(s, (int32_t)nf); /* phase kept */
        break;
    }
    case TOY_EV_SET_PARAM:
        if (ev->id == SYN_PARAM_WAVE) {
            int32_t v = ev->v;
            if (v < 0) v = 0;
            if (v > 2) v = 2;
            s->wave = (uint32_t)v;
        } else if (ev->id == SYN_PARAM_LEVEL) {
            int32_t v = ev->v;
            if (v < SYN_LEVEL_MIN) v = SYN_LEVEL_MIN;
            if (v > SYN_LEVEL_MAX) v = SYN_LEVEL_MAX;
            s->level_pct = v;
            /* percent -> amplitude: v*10 milli = v/100. */
            s->amp = (f64)toy_f64_bits_from_milli(v * 10);
        }
        break; /* out-of-range ids ignored, per the seam */
    default:
        break; /* unknown types ignored, per the seam */
    }
}

static void synspec_reset(void *st_)
{
    synspec_state *s = (synspec_state *)st_;
    toy_memset(s, 0, sizeof *s);
    s->h2 = syn_from_i32(2);
    s->h3 = syn_from_i32(3);
    s->h4 = syn_from_i32(4);
    s->h5 = syn_from_i32(5);
    s->inv2 = f64_div(F64_ONE, s->h2);
    s->inv3 = f64_div(F64_ONE, s->h3);
    s->inv4 = f64_div(F64_ONE, s->h4);
    s->inv5 = f64_div(F64_ONE, s->h5);
    syn_set_freq(s, SYN_FREQ_DEF_MHZ); /* 256 Hz, bin 8 */
    /* Hann window 0.5 - 0.5*cosPi(2n/(N-1)), own copy. */
    f64 denom = syn_from_i32(SYN_N - 1);
    for (int n = 0; n < SYN_N; n++) {
        f64 ang = f64_div(syn_from_i32(2 * n), denom);
        s->win[n] = f64_sub(F64_HALF, f64_mul(F64_HALF, f64_cos_pi(ang)));
    }
    /* Param defaults through the step-0 sequence a runner
     * uses for user settings (julia pattern): reset()
     * provably means the declared defaults. */
    for (uint32_t i = 0; i < toy_cart_synspec.param_count; i++) {
        toy_event_t ev;
        if (toy_param_event(&toy_cart_synspec, i,
                            toy_cart_synspec.params[i].i_def, 0, &ev) == 0)
            synspec_apply(s, &ev);
    }
    /* phase, counters: zero from the memset. */
}

/* One sample at the current phase, per the recipe. */
static f64 syn_sample(const synspec_state *s)
{
    f64 two_p = f64_mul(s->phase, F64_TWO); /* sinPi arg */
    f64 fund = f64_sin_pi(two_p);
    f64 val = fund;
    if (s->wave == 1u) { /* SQUARE: odd harmonics 1, 1/3, 1/5 */
        f64 h3v = f64_sin_pi(f64_mul(two_p, s->h3));
        f64 h5v = f64_sin_pi(f64_mul(two_p, s->h5));
        val = f64_add(val, f64_mul(s->inv3, h3v));
        val = f64_add(val, f64_mul(s->inv5, h5v));
    } else if (s->wave == 2u) { /* SAW: 1, -1/2, 1/3, -1/4 */
        f64 h2v = f64_sin_pi(f64_mul(two_p, s->h2));
        f64 h3v = f64_sin_pi(f64_mul(two_p, s->h3));
        f64 h4v = f64_sin_pi(f64_mul(two_p, s->h4));
        val = f64_sub(val, f64_mul(s->inv2, h2v));
        val = f64_add(val, f64_mul(s->inv3, h3v));
        val = f64_sub(val, f64_mul(s->inv4, h4v));
    }
    return f64_mul(val, s->amp);
}

static void run_spectrum(synspec_state *s)
{
    uint64_t start = s->hist_head; /* ring full: head is oldest */
    for (uint64_t i = 0; i < SYN_N; i++)
        s->re[i] = f64_mul(s->hist[(start + i) % SYN_N], s->win[i]);
    toy_memset(s->im, 0, sizeof s->im); /* bit pattern 0 is +0.0 */
    (void)toy_fft_forward(s->re, s->im, SYN_N);
    for (int k = 0; k < SYN_BINS; k++)
        s->mag[k] = f64_sqrt(f64_fma(s->re[k], s->re[k],
                                     f64_mul(s->im[k], s->im[k])));
    uint64_t best = 1;
    for (uint64_t k = 2; k < SYN_BINS; k++)
        if (s->mag[k] > s->mag[best]) best = k;
    s->peak_bin = best;
    s->fft_count++;
}

static void synspec_step(void *st_)
{
    synspec_state *s = (synspec_state *)st_;
    for (uint32_t i = 0; i < SYN_PER_STEP; i++) {
        f64 v = syn_sample(s);
        s->last_sample = v;
        s->hist[s->hist_head] = v;
        s->hist_head = (s->hist_head + 1u) % SYN_N;
        if (s->filled < SYN_N) s->filled++;
        s->sample_seq++;
        s->phase = f64_add(s->phase, s->dphase);
        if (syn_ge(s->phase, F64_ONE))
            s->phase = f64_sub(s->phase, F64_ONE);
    }
    s->steps++;
    if ((s->steps % SYN_FFT_EVERY) == 0 && s->filled >= SYN_N)
        run_spectrum(s);
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

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

static void synspec_render(const void *st_, toy_fb_t *fb)
{
    const synspec_state *s = (const synspec_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    int sc = R >= 150 ? 2 : 1;
    uint16_t dim = toy_rgb565(110, 125, 155);
    uint16_t bright = toy_rgb565(200, 215, 240);
    toy_fb_clear(fb, toy_rgb565(5, 7, 18));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(28, 34, 54));

    const char *title = "SYNSPEC";
    toy_fb_text(fb, cx - (text_len(title) * 4 * sc - sc) / 2, cy - R + 10 * sc,
                sc, toy_rgb565(150, 170, 200), title);
    /* Frequency readout from the stored millihertz integer:
     * "F 256.0HZ SINE" (waveform name from state enum). */
    char buf[40];
    int n = 0;
    uint32_t fm = s->freq_mhz < 0 ? 0u : (uint32_t)s->freq_mhz;
    n = append_str(buf, n, "F ");
    n = append_u64(buf, n, fm / 1000u);
    buf[n++] = '.';
    n = append_u64(buf, n, (fm / 100u) % 10u);
    n = append_str(buf, n, "HZ ");
    n = append_str(buf, n, syn_wave_names[s->wave > 2u ? 0u : s->wave]);
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, cy - R + 19 * sc, sc,
                toy_rgb565(255, 190, 60), buf);

    int x0 = cx - (R * 68) / 100, x1 = cx + (R * 68) / 100;
    int ymid = cy - (R * 28) / 100;
    int halfh = (R * 26) / 100;
    int yb = cy + (R * 56) / 100;
    toy_fb_line(fb, x0 - 2, ymid, x1 + 2, ymid, toy_rgb565(70, 85, 120));
    toy_fb_line(fb, x0 - 2, yb, x1 + 2, yb, toy_rgb565(70, 85, 120));

    /* Waveform trace: the 256-sample ring, oldest first,
     * full scale +/-1.0, clamped for display only. */
    if (s->filled > 1) {
        uint64_t start = (s->hist_head + SYN_N - s->filled) % SYN_N;
        int px = x0, py = ymid;
        for (uint64_t i = 0; i < s->filled; i++) {
            double v = d64(s->hist[(start + i) % SYN_N]);
            if (v > 1.0) v = 1.0;
            if (v < -1.0) v = -1.0;
            int x = x0 + (int)((i * (uint64_t)(x1 - x0)) / (s->filled - 1));
            int y = ymid - (int)(v * (double)halfh);
            if (i > 0) toy_fb_line(fb, px, py, x, y, toy_rgb565(60, 230, 150));
            px = x;
            py = y;
        }
        toy_fb_disc(fb, px, py, 1, 0xffffu);
    }
    toy_fb_text(fb, x0, ymid - halfh - 8 * sc, sc, dim, "WAVE FS 8192");

    /* Spectrum bars, bins 1..96 (32 Hz/bin, to 3072 Hz).
     * Log range 2^10 below the stored peak magnitude. */
    int maxh = (R * 42) / 100;
    if (s->fft_count > 0 && s->peak_bin >= 1 && s->peak_bin < SYN_BINS &&
        s->mag[s->peak_bin] != F64_ZERO) {
        double top = log2_display(s->mag[s->peak_bin]);
        uint16_t bar = toy_rgb565(40, 210, 190);
        uint16_t peakc = toy_rgb565(255, 190, 60);
        for (int k = 1; k <= 96; k++) {
            double fr = (log2_display(s->mag[k]) - (top - 10.0)) / 10.0;
            if (fr <= 0.0) continue;
            if (fr > 1.0) fr = 1.0;
            int h = (int)(fr * (double)maxh);
            if (h <= 0) continue;
            int bx = x0 + ((k - 1) * (x1 - x0)) / 95;
            toy_fb_line(fb, bx, yb - 1, bx, yb - h,
                        (uint64_t)k == s->peak_bin ? peakc : bar);
        }
    }
    toy_fb_text(fb, x0, yb - maxh - 8 * sc, sc, dim, "FFT 256 HANN");

    n = 0;
    if (s->fft_count == 0) {
        n = append_str(buf, n, "FILL ");
        n = append_u64(buf, n, s->filled);
        n = append_str(buf, n, "/256");
    } else {
        n = append_str(buf, n, "PEAK ");
        n = append_u64(buf, n, s->peak_bin);
        buf[n++] = ' ';
        n = append_u64(buf, n, s->peak_bin * 32u); /* bin*32 Hz */
        n = append_str(buf, n, "HZ");
    }
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, yb + 5 * sc, sc, bright, buf);
    n = 0;
    n = append_str(buf, n, "LV ");
    n = append_u64(buf, n, s->level_pct < 0 ? 0u : (uint64_t)s->level_pct);
    n = append_str(buf, n, " DIAL ");
    n = append_u64(buf, n, s->dial_events);
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, yb + 13 * sc, sc, dim, buf);
}

static const toy_param_t synspec_params[] = {
    { "wave", "Waveform", TOY_PARAM_ENUM, 0,
      0, 2, 0, 0, 0, 0, syn_wave_names },
    { "level", "Level %", TOY_PARAM_I32, 0,
      SYN_LEVEL_MIN, SYN_LEVEL_MAX, 80, 0, 0, 0, 0 },
};

/* Proof: four 40-step phases (5 FFT epochs each).
 *   1..40    idle default: 256 Hz sine, peak bin 8.
 *   41       DIAL +500 milliturns (+128 Hz) -> 384 Hz
 *            sine, peak bin 12 (steps 41..80).
 *   81       SET_PARAM wave=SQUARE at 384 Hz: peak stays
 *            12, harmonics appear at 36 and 60 (81..120).
 *   121      DIAL -1000 (-256 Hz) -> 128 Hz square: peak
 *            bin 4, harmonics at 12 and 20 (121..160).
 * Each change lands one full epoch before its phase's
 * first report at +20, so reported hashes pin spectra
 * from windows wholly of the new tone. */
static const toy_proof_input_t synspec_proof[] = {
    {  41, { TOY_EV_DIAL,      0, 0, 0,  500, 0, 0 } },
    {  81, { TOY_EV_SET_PARAM, SYN_PARAM_WAVE, 0, 0, 1, 0, 0 } },
    { 121, { TOY_EV_DIAL,      0, 0, 0, -1000, 0, 0 } },
};

/* demo 4 steps/frame: 40 host frames replay the whole
 * 160-step proof, spectrum refreshing every 2 frames.
 * proof 160 steps, report every 20 (mid- and end-phase).
 * Board cost is estimated in the parcel report from
 * host synth-only vs FFT-step timings. */
const toy_cart_t toy_cart_synspec = {
    "synspec", "Synth Spectrum",
    "Turn the toy to tune. The harmonics are the recipe, Fourier-read.",
    sizeof(synspec_state), 4, 160, 20,
    synspec_reset, synspec_step, synspec_render,
    synspec_apply, synspec_params, 2, synspec_proof, 3,
    TOY_CAP_DIAL | TOY_CAP_PARAMS, 0, 0, { 0 }
};



