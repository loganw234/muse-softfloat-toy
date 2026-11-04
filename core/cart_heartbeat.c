/* cart_heartbeat.c - a seismocardiography toy on the synthetic
 * chest stream. TOY, NOT A MEDICAL DEVICE: nothing here measures
 * a person; v1 proves the detector chain against scenario 2
 * "chest" (core/toy_signal.c), whose magnitudes are modelled on
 * published SCG numbers (parcel report has the arithmetic).
 *
 * Signal binding: IMU6, SYNTHETIC proof, scenario 2, 100 Hz,
 * seed 0xbea75, 5 samples per step - one step is exactly 50 ms
 * of stream time, index-derived. 5/step (not seismo's 10) keeps
 * the pulse envelope's per-render decay steps fine-grained at
 * demo rate while the proof stays short: 800 steps = 4000
 * samples = 40 s = 50 nominal beats at the generator's exact
 * 75 BPM (first systolic lands one period in plus seed phase).
 *
 * Axis: az (IMU6 channel 2). The puck lies flat on the sternum
 * screen-up, so the chest wall's out-of-plane micro-acceleration
 * - the SCG direction - is z; in this frame's convention gravity
 * rests in +az (16384 LSB) and ax/ay carry only spillover at a
 * quarter/sixth of the pulse. Reading ax/ay would throw away
 * most of an already ~5 mg signal.
 *
 * Chain (per sample, in feed(), softfloat like cart_seismo):
 *   x    = az_counts / 16384            (g)
 *   dyn  = x - baseline                 (filtered signal)
 *   baseline += alpha * dyn             (one-pole detrend)
 * alpha = f64_dec("0.125") = 1/8: an 8-sample (80 ms) time
 * constant, high-pass corner ~2 Hz. That corner is the breathing
 * argument: breathing at 0.25 Hz sits ~3 octaves below it, so
 * the one-pole leaves only ~1/9 of the modelled 12 mg breathing
 * swing in dyn (~1.3 mg), while the ~120 ms systolic complex
 * (energy well above 2 Hz) keeps most of its 5 mg peak. A slower
 * baseline (seismo's 1/256) would pass breathing almost whole
 * and swamp the pulse; a much faster one would differentiate
 * the systolic peak away. First sample initialises baseline.
 *
 * Detector (exact): rising edge of |dyn| over thr (default
 * 2.0 mg = 0.002 g, SET_PARAM "thr" in 0.1 mg units), with a
 * 40-sample (0.4 s) refractory after the previous beat. The
 * refractory is the doublet argument: the diastolic bump rides
 * at +30 samples at ~40% amplitude and, summed with breathing
 * residual, CAN cross a 2 mg threshold - refractory 40 covers
 * its whole +30..+37 span while still passing any plausible
 * resting rate (it caps detection at 150 BPM). On a beat: count,
 * last-beat sample/step, inter-beat interval into an 8-ring,
 * BPM = 6000 / median-interval-samples (integer; rate is the
 * descriptor's 100 Hz), pulse envelope set to 1.0. The envelope
 * decays x0.98 per SAMPLE in feed(), so the heart's rendered
 * size is a function of stream time, never wall time.
 *
 * Before the first feed the label says KIND_NONE and render
 * shows NO SIG - never a flat trace dressed as a chest.
 */
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

#define HB_N 200           /* graph ring, samples (2.0 s at 100 Hz) */
#define HB_REFRACTORY 40u  /* samples; covers diastolic at +30..37  */
#define HB_IBI_N 8u        /* inter-beat intervals kept for median  */
#define HB_PARAM_THR 0u    /* threshold, 0.1 mg units               */
#define HB_THR_MIN 5       /* 0.5 mg                                */
#define HB_THR_MAX 100     /* 10.0 mg                               */
#define HB_THR_DEF 20      /* 2.0 mg                                */

typedef struct {
    toy_sig_label_t label; /* provenance, hashed (32 bytes)         */
    f64 hist[HB_N];        /* dyn ring, g                           */
    f64 baseline;          /* one-pole gravity+breathing estimate   */
    f64 alpha;             /* 0.125, from f64_dec at reset          */
    f64 scale_g;           /* 16384 LSB/g divisor, from f64_dec     */
    f64 thr_g;             /* detector threshold, g (from param)    */
    f64 env;               /* pulse envelope 0..1, decays/sample    */
    f64 env_decay;         /* 0.98 per sample, f64_dec              */
    f64 last_dyn;          /* newest filtered sample, g             */
    f64 prev_abs;          /* |dyn| of the previous sample          */
    f64 peak_abs;          /* max |dyn| since reset, g              */
    uint64_t ibi[HB_IBI_N];/* inter-beat intervals, samples (ring)  */
    uint64_t steps;        /* steps completed                       */
    uint64_t sample_seq;   /* next sample index to be fed           */
    uint64_t hist_head;    /* next ring write slot                  */
    uint64_t filled;       /* samples in ring, capped at HB_N       */
    uint64_t beat_count;   /* detector beats so far                 */
    uint64_t last_beat_sample; /* 0-based sample index of last beat */
    uint64_t last_beat_step;   /* step the last beat preceded       */
    uint64_t ibi_head;     /* next interval ring slot               */
    uint64_t ibi_count;    /* intervals stored, capped at HB_IBI_N  */
    uint64_t feed_calls;
    uint32_t tick[HB_N];   /* 1 at ring slots where a beat fired    */
    int32_t last_az;       /* newest raw az counts                  */
    int32_t thr_tenth_mg;  /* threshold param, 0.1 mg units         */
    uint32_t baseline_init;
    uint32_t bpm;          /* integer BPM readout from median IBI   */
    uint32_t median_ibi;   /* median inter-beat interval, samples   */
    uint32_t pad0;
} hb_state;

static f64 hb_from_i32(int32_t v) { return (f64)toy_f64_bits_from_i32(v); }

/* threshold: v in 0.1 mg -> g is v / 10000. */
static void hb_set_thr(hb_state *s, int32_t tenth_mg)
{
    if (tenth_mg < HB_THR_MIN) tenth_mg = HB_THR_MIN;
    if (tenth_mg > HB_THR_MAX) tenth_mg = HB_THR_MAX;
    s->thr_tenth_mg = tenth_mg;
    s->thr_g = f64_div(hb_from_i32(tenth_mg), f64_dec("10000"));
}

static void hb_reset(void *st_)
{
    hb_state *s = (hb_state *)st_;
    toy_memset(s, 0, sizeof *s);
    toy_sig_label_reset(&s->label); /* NONE: NO SIG until fed */
    s->alpha = f64_dec("0.125");
    s->scale_g = f64_dec("16384");
    s->env_decay = f64_dec("0.98");
    hb_set_thr(s, HB_THR_DEF);
}

/* Median of the stored intervals (insertion sort of a copy,
 * average of the two middles for even counts - integer). */
static uint32_t hb_median_ibi(const hb_state *s)
{
    uint64_t v[HB_IBI_N];
    uint32_t n = (uint32_t)s->ibi_count;
    if (n == 0u) return 0u;
    for (uint32_t i = 0; i < n; i++) v[i] = s->ibi[i];
    for (uint32_t i = 1; i < n; i++) {
        uint64_t x = v[i];
        uint32_t j = i;
        while (j > 0u && v[j - 1] > x) { v[j] = v[j - 1]; j--; }
        v[j] = x;
    }
    if (n & 1u) return (uint32_t)v[n / 2];
    return (uint32_t)((v[n / 2 - 1] + v[n / 2]) / 2u);
}

static void hb_feed(void *st_, const int16_t *samples, uint32_t n,
                    uint32_t channels, const toy_sig_info_t *info,
                    uint32_t flags)
{
    hb_state *s = (hb_state *)st_;
    toy_sig_label_feed(&s->label, info, n, flags);
    s->feed_calls++;
    if (!samples || channels < 3u) return;
    for (uint32_t i = 0; i < n; i++) {
        int32_t az = samples[(size_t)i * channels + 2u];
        s->last_az = az;
        f64 x = f64_div(hb_from_i32(az), s->scale_g);
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
        s->env = f64_mul(s->env, s->env_decay); /* per-sample decay */
        f64 adyn = f64_abs(dyn);
        if (adyn > s->peak_abs) s->peak_abs = adyn;
        uint32_t beat = 0u;
        if (s->prev_abs <= s->thr_g && adyn > s->thr_g &&
            (s->beat_count == 0 ||
             s->sample_seq - s->last_beat_sample >= HB_REFRACTORY)) {
            beat = 1u;
            if (s->beat_count > 0) {
                s->ibi[s->ibi_head] = s->sample_seq - s->last_beat_sample;
                s->ibi_head = (s->ibi_head + 1u) % HB_IBI_N;
                if (s->ibi_count < HB_IBI_N) s->ibi_count++;
                s->median_ibi = hb_median_ibi(s);
                /* BPM = 60 s * 100 samples/s / samples-per-beat. */
                s->bpm = s->median_ibi ? 6000u / s->median_ibi : 0u;
            }
            s->beat_count++;
            s->last_beat_sample = s->sample_seq;
            s->last_beat_step = s->steps + 1u; /* step this feed precedes */
            s->env = F64_ONE;
        }
        s->prev_abs = adyn;
        s->hist[s->hist_head] = dyn;
        s->tick[s->hist_head] = beat;
        s->hist_head = (s->hist_head + 1u) % HB_N;
        if (s->filled < HB_N) s->filled++;
        s->sample_seq++;
    }
}

static void hb_step(void *st_)
{
    hb_state *s = (hb_state *)st_;
    s->steps++;
}

static void hb_apply(void *st_, const toy_event_t *ev)
{
    hb_state *s = (hb_state *)st_;
    if (ev->type == TOY_EV_SET_PARAM && ev->id == HB_PARAM_THR)
        hb_set_thr(s, ev->v); /* clamped inside, per the seam */
    /* out-of-range ids and unknown types ignored, per the seam */
}

/* Display conversion only (never feeds state). */
static double d64(f64 a) { union { f64 u; double d; } v; v.u = a; return v.d; }

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

/* Provenance from the hashed label: synthetic named synthetic. */
static const char *provenance(const hb_state *s)
{
    if (s->label.kind == TOY_SIG_KIND_SYNTHETIC) return "SYNTH CHEST";
    if (s->label.kind == TOY_SIG_KIND_LIVE) return "LIVE CHEST";
    if (s->label.kind == TOY_SIG_KIND_REPLAY)
        return s->label.orig_kind == TOY_SIG_KIND_LIVE ? "REPLAY-LIVE"
                                                       : "REPLAY-SYN";
    return "NO SIG";
}

/* A recognisable heart from fb primitives: two discs for the
 * lobes over a tapering V of scanlines. u = lobe offset unit. */
static void draw_heart(toy_fb_t *fb, int cx, int cy, int u, uint16_t col)
{
    int lobe_y = cy - (u * 35) / 100;
    int lobe_r = (u * 58) / 100;
    int off = (u * 55) / 100;
    toy_fb_disc(fb, cx - off, lobe_y, lobe_r, col);
    toy_fb_disc(fb, cx + off, lobe_y, lobe_r, col);
    int top = lobe_y;
    int bottom = cy + (u * 105) / 100;
    int half_top = off + lobe_r;
    for (int y = top; y <= bottom; y++) {
        int span = bottom - top;
        int hw = span > 0 ? (half_top * (bottom - y)) / span : 0;
        if (hw > 0) toy_fb_line(fb, cx - hw, y, cx + hw, y, col);
    }
}

static void hb_render(const void *st_, toy_fb_t *fb)
{
    const hb_state *s = (const hb_state *)st_;
    int cx = toy_cx(fb), cy = toy_cy(fb), R = toy_r(fb);
    int sc = R >= 150 ? 2 : 1;
    uint16_t dim = toy_rgb565(110, 125, 155);
    uint16_t bright = toy_rgb565(200, 215, 240);
    uint16_t amber = toy_rgb565(255, 190, 60);
    toy_fb_clear(fb, toy_rgb565(14, 6, 12));
    toy_fb_ring(fb, cx, cy, R - 2, toy_rgb565(54, 30, 44));

    const char *title = "HEARTBEAT";
    toy_fb_text(fb, cx - (text_len(title) * 4 * sc - sc) / 2, cy - R + 10 * sc,
                sc, toy_rgb565(220, 150, 170), title);
    const char *prov = provenance(s);
    toy_fb_text(fb, cx - (text_len(prov) * 4 * sc - sc) / 2, cy - R + 19 * sc,
                sc, amber, prov);
    /* The label the brief insists on, in every state. */
    const char *foot = "TOY NOT MEDICAL";
    toy_fb_text(fb, cx - (text_len(foot) * 4 - 1) / 2, cy + R - 12, 1,
                toy_rgb565(255, 120, 90), foot);

    int x0 = cx - (R * 68) / 100, x1 = cx + (R * 68) / 100;
    int ymid = cy + (R * 40) / 100;
    int halfh = (R * 16) / 100;
    toy_fb_line(fb, x0 - 2, ymid, x1 + 2, ymid, toy_rgb565(90, 70, 95));

    if (s->label.kind == TOY_SIG_KIND_NONE) {
        const char *msg = "NO SIG";
        toy_fb_text(fb, cx - (text_len(msg) * 4 * sc - sc) / 2,
                    cy - (R * 10) / 100, sc, toy_rgb565(255, 120, 90), msg);
        draw_heart(fb, cx, cy - (R * 28) / 100, (R * 22) / 100,
                   toy_rgb565(90, 45, 60));
        return;
    }

    /* Heart scales with the pulse envelope: visibly larger on
     * the beat, relaxing between beats (env 1.0 -> ~0.2). */
    double env = d64(s->env);
    if (env < 0.0) env = 0.0;
    if (env > 1.0) env = 1.0;
    int u = (R * 22) / 100 + (int)(env * (double)((R * 10) / 100));
    uint16_t heart_col = env > 0.55 ? toy_rgb565(255, 70, 95)
                                    : toy_rgb565(200, 45, 70);
    draw_heart(fb, cx, cy - (R * 30) / 100, u, heart_col);

    /* BPM + beat count, from stored integers only. */
    char buf[40];
    int n = 0;
    if (s->bpm > 0u) {
        n = append_u64(buf, n, s->bpm);
        n = append_str(buf, n, " BPM");
    } else {
        n = append_str(buf, n, "-- BPM");
    }
    n = append_str(buf, n, "  BEATS ");
    n = append_u64(buf, n, s->beat_count);
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, cy + (R * 8) / 100, sc,
                bright, buf);
    n = 0;
    n = append_str(buf, n, "THR ");
    n = append_u64(buf, n, (uint64_t)(s->thr_tenth_mg / 10));
    buf[n++] = '.';
    n = append_u64(buf, n, (uint64_t)(s->thr_tenth_mg % 10));
    n = append_str(buf, n, "MG AZ 100HZ");
    buf[n] = '\0';
    toy_fb_text(fb, cx - (n * 4 * sc - sc) / 2, cy + (R * 8) / 100 + 8 * sc,
                sc, dim, buf);

    /* Scrolling filtered waveform, full scale +/-6 mg, with an
     * amber tick at every detected beat in the window. */
    toy_fb_text(fb, x0, ymid + halfh + 3 * sc, sc, dim, "AZ DYN FS 6MG");
    if (s->filled > 1) {
        const double fs = 0.006; /* g */
        uint64_t start = (s->hist_head + HB_N - s->filled) % HB_N;
        int px = x0, py = ymid;
        for (uint64_t i = 0; i < s->filled; i++) {
            uint64_t slot = (start + i) % HB_N;
            double v = d64(s->hist[slot]) / fs;
            if (v > 1.0) v = 1.0;
            if (v < -1.0) v = -1.0;
            int x = x0 + (int)((i * (uint64_t)(x1 - x0)) / (s->filled - 1));
            int y = ymid - (int)(v * (double)halfh);
            if (s->tick[slot])
                toy_fb_line(fb, x, ymid - halfh, x, ymid + halfh, amber);
            if (i > 0) toy_fb_line(fb, px, py, x, y, toy_rgb565(255, 120, 140));
            px = x;
            py = y;
        }
        toy_fb_disc(fb, px, py, 1, 0xffffu);
    }
}

static const toy_param_t hb_params[] = {
    { "thr", "Threshold 0.1mg", TOY_PARAM_I32, 0,
      HB_THR_MIN, HB_THR_MAX, HB_THR_DEF, 0, 0, 0, 0 },
};

/* Proof script: raise the threshold to 3.0 mg mid-run and back,
 * so the proof pins the SET_PARAM path while the systolic peak
 * in dyn (measured in the parcel probe) stays over both. */
static const toy_proof_input_t hb_proof[] = {
    { 401, { TOY_EV_SET_PARAM, HB_PARAM_THR, 0, 0, 30, 0, 0 } },
    { 601, { TOY_EV_SET_PARAM, HB_PARAM_THR, 0, 0, 20, 0, 0 } },
};

/* demo 1 step/frame: at nominal 30 fps the stream plays at
 * 1.5x (50 ms per frame) - the heart pulses visibly faster than
 * life but the graph still reads. proof 800 steps x 5 samples
 * = 4000 samples = 40 s: >= 48 generator beats (exact count for
 * the pinned seed is quoted from the probe in the report). */
const toy_cart_t toy_cart_heartbeat = {
    "heartbeat", "Heartbeat (SCG toy)",
    "Puck on chest: heartbeat micro-accelerations. Toy, not medical.",
    sizeof(hb_state), 1, 800, 100,
    hb_reset, hb_step, hb_render,
    hb_apply, hb_params, 1, hb_proof, 2, TOY_CAP_PARAMS,
    0, /* state_version */
    hb_feed,
    { TOY_SIG_DEVICE_IMU6, TOY_SIG_KIND_SYNTHETIC, 2u, 100u, 5u,
      UINT64_C(0xbea75) }
};
