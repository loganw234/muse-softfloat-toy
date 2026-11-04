/* toy_signal.c - see toy_signal.h. Integer only; the only
 * includes are the header (stddef/stdint) so this file also
 * compiles freestanding for wasm32 -nostdlib like the rest of
 * core/ (toy_util.h exists for the same reason). */
#include "toy_signal.h"
#include "toy_signal_sin.inc"

/* --- primitives -------------------------------------------------- */

void toy_rng_seed(toy_rng_t *r, uint64_t seed)
{
    /* Any seed is valid for SplitMix64, including 0. Mixing the
     * golden gamma in at seed time keeps nearby seeds from
     * producing visibly correlated first outputs. */
    r->s = seed + 0x9e3779b97f4a7c15ull;
}

uint32_t toy_rng_u32(toy_rng_t *r)
{
    uint64_t z = (r->s += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    z = z ^ (z >> 31);
    return (uint32_t)(z >> 32);
}

int32_t toy_rng_bipolar_q15(toy_rng_t *r)
{
    /* Top 16 bits, recentred: [0,65535] -> [-32767,32768] clamped
     * to the symmetric range the rest of the file assumes. */
    int32_t v = (int32_t)(toy_rng_u32(r) >> 16) - 32768;
    if (v < -32767) v = -32767;
    return v;
}

int64_t toy_sig_asr64(int64_t v, unsigned shift)
{
    if (v >= 0) return (int64_t)((uint64_t)v >> shift);
    /* floor division by 2^shift for negatives, via unsigned. */
    uint64_t mag = (uint64_t)(-(v + 1)) + 1u; /* |v|, no overflow */
    uint64_t q = (mag + ((1ull << shift) - 1ull)) >> shift;
    return -(int64_t)q;
}

int32_t toy_sig_sin_q15(uint32_t phase)
{
    uint32_t idx = phase >> 22;                 /* 0..1023 */
    uint32_t frac = (phase >> 6) & 0xffffu;     /* inter-step  */
    int32_t a = toy_sig_sin_table[idx];
    int32_t b = toy_sig_sin_table[idx + 1u];
    /* |(b-a)| <= ~201, product fits int32 comfortably. */
    return a + (int32_t)toy_sig_asr64((int64_t)(b - a) * (int64_t)frac, 16);
}

uint32_t toy_sig_phase_inc(uint32_t freq_millihz, uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0u) return 0u;
    uint64_t num = (uint64_t)freq_millihz << 32;
    uint64_t den = (uint64_t)sample_rate_hz * 1000ull;
    return (uint32_t)((num + den / 2ull) / den);
}

int16_t toy_sig_clamp_i16(int32_t v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return (int16_t)-32768;
    return (int16_t)v;
}

int32_t toy_bandnoise_next(toy_bandnoise_t *b, toy_rng_t *r)
{
    int32_t w = toy_rng_bipolar_q15(r);
    /* One-pole lowpasses y += (x-y)/2^k with floor shifts so the
     * recursion is identical everywhere. fast (k=2) passes the
     * tremor band, slow (k=8) tracks the wander; their
     * difference is band-limited at both ends. */
    b->lp_fast += (int32_t)toy_sig_asr64((int64_t)w - b->lp_fast, 2);
    b->lp_slow += (int32_t)toy_sig_asr64((int64_t)w - b->lp_slow, 8);
    return b->lp_fast - b->lp_slow;
}

uint64_t toy_sig_fnv1a64(const void *bytes, size_t n)
{
    const unsigned char *p = (const unsigned char *)bytes;
    uint64_t h = 1469598103934665603ull; /* as toy_carts.c */
    for (size_t i = 0; i < n; i++) {
        h ^= (uint64_t)p[i];
        h *= 1099511628211ull;
    }
    return h;
}

uint64_t toy_sig_hash_samples(const int16_t *interleaved,
                              uint64_t n_samples, uint32_t channels)
{
    uint64_t h = 1469598103934665603ull;
    uint64_t total = n_samples * (uint64_t)channels;
    for (uint64_t i = 0; i < total; i++) {
        uint16_t u = (uint16_t)interleaved[i];
        h ^= (uint64_t)(u & 0xffu);        /* low byte first  */
        h *= 1099511628211ull;
        h ^= (uint64_t)((u >> 8) & 0xffu); /* then high byte  */
        h *= 1099511628211ull;
    }
    return h;
}

static int32_t scale_q15(int32_t q15_value, int32_t amp)
{
    /* amp * value / 32768, rounded to nearest (half up) by
     * biasing before the floor shift. The divisor 32768 rather
     * than 32767 is part of the stream definition. */
    int64_t p = (int64_t)q15_value * (int64_t)amp;
    return (int32_t)toy_sig_asr64(p + 16384, 15);
}

static int32_t scale_q8(int32_t value, int32_t gain_q8)
{
    int64_t p = (int64_t)value * (int64_t)gain_q8;
    return (int32_t)toy_sig_asr64(p + 128, 8);
}

uint32_t toy_sig_device_channels(uint32_t device)
{
    if (device == TOY_SIG_DEVICE_IMU6) return 6u;
    if (device == TOY_SIG_DEVICE_MIC1) return 1u;
    return 0u;
}

const char *toy_sig_kind_name(uint32_t kind)
{
    if (kind == TOY_SIG_KIND_SYNTHETIC) return "synthetic";
    if (kind == TOY_SIG_KIND_REPLAY) return "replay";
    if (kind == TOY_SIG_KIND_LIVE) return "live";
    return "none";
}

uint64_t toy_sig_param_stream_id(const toy_sig_info_t *info)
{
    /* FNV-1a 64 over canonical little-endian fields, never over
     * struct bytes (see toy_sig_hash_samples for the byte-order
     * rule). stream_id itself is excluded by construction. */
    uint32_t w32[8];
    uint64_t h = 1469598103934665603ull;
    if (!info) return 0;
    w32[0] = info->kind;
    w32[1] = info->orig_kind;
    w32[2] = info->device;
    w32[3] = info->channels;
    w32[4] = info->sample_rate_hz;
    w32[5] = info->scale_num;
    w32[6] = info->scale2_num;
    w32[7] = info->scenario_id;
    for (int w = 0; w < 8; w++)
        for (int b = 0; b < 4; b++) {
            h ^= (uint64_t)((w32[w] >> (8 * b)) & 0xffu);
            h *= 1099511628211ull;
        }
    for (int w = 0; w < 2; w++) {
        uint64_t v = w == 0 ? info->seed : info->n_samples;
        for (int b = 0; b < 8; b++) {
            h ^= (uint64_t)((v >> (8 * b)) & 0xffu);
            h *= 1099511628211ull;
        }
    }
    return h;
}

/* --- labelling (study 7) ------------------------------------------- */

void toy_sig_label_reset(toy_sig_label_t *l)
{
    if (!l) return;
    l->stream_id = 0;
    l->samples_seen = 0;
    l->kind = TOY_SIG_KIND_NONE;
    l->orig_kind = TOY_SIG_KIND_NONE;
    l->device = 0;
    l->gap_count = 0;
}

void toy_sig_label_feed(toy_sig_label_t *l, const toy_sig_info_t *info,
                        uint32_t n_samples, uint32_t flags)
{
    if (!l || !info) return;
    l->kind = info->kind;
    l->orig_kind = info->orig_kind;
    l->device = info->device;
    l->stream_id = info->stream_id;
    l->samples_seen += (uint64_t)n_samples;
    if (flags & TOY_SIG_FLAG_UNDERRUN) l->gap_count++;
}

/* --- dial derivation: gyro-z -> milliturns ------------------------- */

void toy_dial_reset(toy_dial_t *d, uint32_t scale_lsb_per_dps,
                    uint32_t sample_rate_hz)
{
    if (!d) return;
    d->accum_num = 0;
    d->total_milliturns = 0;
    d->scale_lsb_per_dps = scale_lsb_per_dps;
    d->sample_rate_hz = sample_rate_hz;
}

void toy_dial_reset_for_info(toy_dial_t *d, const toy_sig_info_t *info)
{
    if (!info) {
        toy_dial_reset(d, 0, 0);
        return;
    }
    toy_dial_reset(d, info->scale2_num, info->sample_rate_hz);
}

int32_t toy_dial_feed(toy_dial_t *d, const int16_t *frames,
                      uint32_t n_samples, uint32_t channels)
{
    int64_t denom, delta64;
    if (!d || !frames || channels != 6u) return 0;
    if (d->scale_lsb_per_dps == 0u || d->sample_rate_hz == 0u) return 0;
    /* One milliturn of numerator: (LSB per deg/s) * 360 deg *
     * samples/s, against gz*1000 per sample (see header). */
    denom = (int64_t)d->scale_lsb_per_dps * 360 *
            (int64_t)d->sample_rate_hz;
    if (denom <= 0) return 0;
    for (uint32_t i = 0; i < n_samples; i++)
        d->accum_num += (int64_t)frames[(size_t)i * channels + 5u] * 1000;
    /* C11 division truncates towards zero, so the remainder
     * keeps the sign of rotation and |remainder| < denom in
     * both directions: reverse rotation first pays back this
     * call's partial milliturn before emitting negative ones. */
    delta64 = d->accum_num / denom;
    d->accum_num -= delta64 * denom;
    d->total_milliturns += delta64;
    if (delta64 > 2147483647ll) return 2147483647;
    if (delta64 < -2147483647ll - 1ll) return -2147483647 - 1;
    return (int32_t)delta64;
}

int toy_dial_feed_event(toy_dial_t *d, const int16_t *frames,
                        uint32_t n_samples, uint32_t channels,
                        toy_event_t *out)
{
    int32_t delta = toy_dial_feed(d, frames, n_samples, channels);
    toy_event_t ev = { 0 };
    if (!out || delta == 0) return 0;
    ev.type = TOY_EV_DIAL;
    ev.id = 0; /* the device itself */
    ev.v = delta;
    *out = ev;
    return 1;
}

static void info_zero(toy_sig_info_t *i)
{
    i->kind = TOY_SIG_KIND_NONE;
    i->orig_kind = TOY_SIG_KIND_NONE;
    i->device = 0;
    i->channels = 0;
    i->sample_rate_hz = 0;
    i->scale_num = 0;
    i->scale2_num = 0;
    i->scenario_id = 0;
    i->seed = 0;
    i->stream_id = 0;
    i->n_samples = 0;
}

/* --- synthetic IMU ------------------------------------------------ */

void toy_sig_synth_imu_init(toy_sig_synth_imu_t *s, uint64_t seed,
                            uint32_t sample_rate_hz)
{
    info_zero(&s->info);
    s->info.kind = TOY_SIG_KIND_SYNTHETIC;
    s->info.orig_kind = TOY_SIG_KIND_SYNTHETIC;
    s->info.device = TOY_SIG_DEVICE_IMU6;
    s->info.channels = 6u;
    s->info.sample_rate_hz = sample_rate_hz;
    s->info.scale_num = 16384u;  /* LSB per g at +/-2 g      */
    s->info.scale2_num = 128u;   /* LSB per deg/s at +/-256  */
    s->info.scenario_id = 1u;    /* "desk"                   */
    s->info.seed = seed;
    s->info.n_samples = 0u;      /* unbounded until recorded */

    /* Tilt amplitude 0.25 g => 4096 LSB; az holds the rest of
     * the 1 g vector: round(sqrt(16384^2 - 4096^2)) = 15864
     * (offline constant, part of the stream definition). */
    s->tilt_amp_lsb = 4096;
    s->az_const_lsb = 15864;
    s->tremor_amp = 3;   /* Q8: bandnoise sd ~7100 -> ~80 LSB sd (~5 mg) */
    s->gyro_amp = 1;     /* Q8: -> ~28 LSB sd (~0.2 deg/s)               */
    s->tap_at = (uint64_t)sample_rate_hz * 30ull; /* tap at t=30 s */
    s->tap_amp_lsb = 8000;                        /* ~0.49 g peak  */
    s->tilt_inc = toy_sig_phase_inc(50u, sample_rate_hz);   /* 0.05 Hz */
    s->tap_inc = toy_sig_phase_inc(20000u, sample_rate_hz); /* 20 Hz   */

    s->seq = 0;
    s->tilt_phase = 0;
    s->tap_phase = 0;
    s->tap_env = 0;
    toy_rng_seed(&s->rng, seed);
    for (int i = 0; i < 6; i++) { s->bn[i].lp_fast = 0; s->bn[i].lp_slow = 0; }
    /* Settle the noise filters so sample 0 is already in-band;
     * the number of warm-up draws is part of the definition. */
    for (int i = 0; i < 512; i++)
        for (int a = 0; a < 6; a++) (void)toy_bandnoise_next(&s->bn[a], &s->rng);
    /* Name the stream before it exists, identically everywhere.
     * Set last; the parameter hash excludes stream_id itself. */
    s->info.stream_id = toy_sig_param_stream_id(&s->info);
}

/* --- synthetic IMU scenario 2 "chest" ------------------------------
 * Modelled seismocardiography (SCG) magnitudes, NOT a recording:
 * chest-wall micro-accelerations from the heartbeat are order
 * g*1e-3 (typical features ~5 mg, placement-dependent; see the
 * parcel report for the literature arithmetic). Shape tables are
 * per-mille of each bump's peak: a sharp systolic complex ~12
 * samples wide with a small undershoot, then a diastolic bump at
 * +30 samples at ~40% - the doublet a naive threshold detector
 * double-fires on, which is exactly why it is in the vector. */
static const int16_t scg_sys[12] = {
    0, 90, 300, 620, 900, 1000, 720, 300, -60, -180, -80, 0
};
static const int16_t scg_dia[8] = {
    0, 200, 600, 1000, 700, 250, -80, 0
};
#define SCG_SYS_PEAK_LSB 82  /* ~5.0 mg at 16384 LSB/g */
#define SCG_DIA_PEAK_LSB 33  /* ~40% of systolic (~2.0 mg) */
#define SCG_DIA_AT       30u /* diastolic offset, samples */
#define CHEST_BREATH_LSB 197 /* ~12 mg breathing on az */

void toy_sig_synth_imu_chest_init(toy_sig_synth_imu_t *s, uint64_t seed,
                                  uint32_t sample_rate_hz)
{
    info_zero(&s->info);
    s->info.kind = TOY_SIG_KIND_SYNTHETIC;
    s->info.orig_kind = TOY_SIG_KIND_SYNTHETIC;
    s->info.device = TOY_SIG_DEVICE_IMU6;
    s->info.channels = 6u;
    s->info.sample_rate_hz = sample_rate_hz;
    s->info.scale_num = 16384u;  /* LSB per g at +/-2 g      */
    s->info.scale2_num = 128u;   /* LSB per deg/s at +/-256  */
    s->info.scenario_id = 2u;    /* "chest"                  */
    s->info.seed = seed;
    s->info.n_samples = 0u;      /* unbounded until recorded */

    /* Desk fields unused; zeroed so no stale state leaks in. */
    s->tilt_amp_lsb = 0;
    s->az_const_lsb = 16384;     /* flat on sternum: +1 g in +az */
    s->tremor_amp = 0;
    s->gyro_amp = 0;
    s->tap_at = 0;
    s->tap_amp_lsb = 0;
    s->tilt_inc = 0;
    s->tap_inc = 0;

    s->breath_amp_lsb = CHEST_BREATH_LSB;
    s->breath_inc = toy_sig_phase_inc(250u, sample_rate_hz); /* 0.25 Hz */
    /* 75 BPM exactly: period = rate * 60 / 75 = rate*4/5 samples
     * (80 at 100 Hz). Integer; rates not divisible by 5 truncate
     * and the truncation is part of that stream's definition. */
    s->beat_period = sample_rate_hz ? (sample_rate_hz * 4u) / 5u : 0u;
    if (s->beat_period == 0u) s->beat_period = 1u;

    s->seq = 0;
    s->tilt_phase = 0;
    s->tap_phase = 0;
    s->tap_env = 0;
    toy_rng_seed(&s->rng, seed);
    for (int i = 0; i < 6; i++) { s->bn[i].lp_fast = 0; s->bn[i].lp_slow = 0; }
    /* Seed varies phase/noise only: breathing starts at a random
     * phase and the first systolic lands one full period in plus
     * a random phase, so a consumer's baseline can settle on
     * breathing before the first beat. Draw order is pinned. */
    s->breath_phase = toy_rng_u32(&s->rng);
    s->beat_first = (uint64_t)s->beat_period +
                    (uint64_t)(toy_rng_u32(&s->rng) % s->beat_period);
    s->info.stream_id = toy_sig_param_stream_id(&s->info);
}

/* Uniform integer noise in [-2, +2] LSB (~0.12 mg steps): the
 * quantisation-class floor of this vector, deliberately far below
 * the modelled sensor noise a real QMI8658C would add (report). */
static int32_t chest_noise(toy_rng_t *r)
{
    return (int32_t)(toy_rng_u32(r) % 5u) - 2;
}

static void chest_next(toy_sig_synth_imu_t *s, int16_t out6[6])
{
    int32_t pulse = 0;
    if (s->beat_period > 0u && s->seq >= s->beat_first) {
        uint32_t pos = (uint32_t)((s->seq - s->beat_first) % s->beat_period);
        if (pos < 12u)
            pulse += (SCG_SYS_PEAK_LSB * (int32_t)scg_sys[pos]) / 1000;
        if (pos >= SCG_DIA_AT && pos < SCG_DIA_AT + 8u)
            pulse += (SCG_DIA_PEAK_LSB * (int32_t)scg_dia[pos - SCG_DIA_AT]) / 1000;
    }
    int32_t breath = scale_q15(toy_sig_sin_q15(s->breath_phase),
                               s->breath_amp_lsb);
    s->breath_phase += s->breath_inc;

    int32_t v[6];
    v[0] = pulse / 4 + breath / 16 + chest_noise(&s->rng);   /* ax spill */
    v[1] = -(pulse / 6) + breath / 24 + chest_noise(&s->rng);/* ay spill */
    v[2] = s->az_const_lsb + breath + pulse + chest_noise(&s->rng);
    v[3] = chest_noise(&s->rng); /* gyro: no modelled rotation */
    v[4] = chest_noise(&s->rng);
    v[5] = chest_noise(&s->rng);
    for (int a = 0; a < 6; a++) out6[a] = toy_sig_clamp_i16(v[a]);
    s->seq++;
}

void toy_sig_synth_imu_next(toy_sig_synth_imu_t *s, int16_t out6[6])
{
    if (s->info.scenario_id == 2u) { chest_next(s, out6); return; }
    if (s->seq == s->tap_at) { s->tap_env = s->tap_amp_lsb; s->tap_phase = 0; }

    int32_t gx = scale_q15(toy_sig_sin_q15(s->tilt_phase), s->tilt_amp_lsb);
    int32_t gy = scale_q15(toy_sig_cos_q15(s->tilt_phase), s->tilt_amp_lsb);
    int32_t gz = s->az_const_lsb;
    if (s->tap_env > 8) {
        gz += scale_q15(toy_sig_sin_q15(s->tap_phase), s->tap_env);
        gx += scale_q15(toy_sig_cos_q15(s->tap_phase), s->tap_env / 4);
        s->tap_phase += s->tap_inc;
        /* env *= 0.90 in Q15 (29491/32768). */
        s->tap_env = (int32_t)toy_sig_asr64((int64_t)s->tap_env * 29491, 15);
    }

    int32_t v[6];
    v[0] = gx; v[1] = gy; v[2] = gz; v[3] = 0; v[4] = 0; v[5] = 0;
    for (int a = 0; a < 6; a++) {
        int32_t n = toy_bandnoise_next(&s->bn[a], &s->rng);
        v[a] += scale_q8(n, a < 3 ? s->tremor_amp : s->gyro_amp);
    }
    for (int a = 0; a < 6; a++) out6[a] = toy_sig_clamp_i16(v[a]);

    s->tilt_phase += s->tilt_inc;
    s->seq++;
}

/* --- synthetic microphone ------------------------------------------ */

void toy_sig_synth_mic_init(toy_sig_synth_mic_t *s, uint64_t seed,
                            uint32_t sample_rate_hz,
                            uint32_t freq0_millihz, uint32_t freq1_millihz,
                            uint64_t chirp_samples)
{
    info_zero(&s->info);
    s->info.kind = TOY_SIG_KIND_SYNTHETIC;
    s->info.orig_kind = TOY_SIG_KIND_SYNTHETIC;
    s->info.device = TOY_SIG_DEVICE_MIC1;
    s->info.channels = 1u;
    s->info.sample_rate_hz = sample_rate_hz;
    s->info.scale_num = 1u; /* PCM full scale */
    s->info.scenario_id = 1u;
    s->info.seed = seed;

    s->tone_amp = 12000;
    s->noise_amp = 4; /* Q8 -> a few hundred LSB of hiss */
    uint32_t inc0 = toy_sig_phase_inc(freq0_millihz, sample_rate_hz);
    uint32_t inc1 = toy_sig_phase_inc(freq1_millihz, sample_rate_hz);
    s->phase_inc = inc0;
    s->inc_delta = 0;
    if (chirp_samples > 0u && inc1 != inc0) {
        int64_t d = (int64_t)inc1 - (int64_t)inc0;
        s->inc_delta = (int32_t)(d / (int64_t)chirp_samples);
        if (s->inc_delta == 0) s->inc_delta = d > 0 ? 1 : -1;
    }
    s->seq = 0;
    s->phase = 0;
    toy_rng_seed(&s->rng, seed ^ 0x6d6963ull); /* decorrelate from IMU */
    s->bn.lp_fast = 0;
    s->bn.lp_slow = 0;
    for (int i = 0; i < 512; i++) (void)toy_bandnoise_next(&s->bn, &s->rng);
    s->info.stream_id = toy_sig_param_stream_id(&s->info);
}

int16_t toy_sig_synth_mic_next(toy_sig_synth_mic_t *s)
{
    int32_t v = scale_q15(toy_sig_sin_q15(s->phase), s->tone_amp);
    v += scale_q8(toy_bandnoise_next(&s->bn, &s->rng), s->noise_amp);
    s->phase += s->phase_inc;
    if (s->inc_delta != 0) {
        int64_t inc = (int64_t)s->phase_inc + s->inc_delta;
        if (inc > 0) s->phase_inc = (uint32_t)inc;
    }
    s->seq++;
    return toy_sig_clamp_i16(v);
}

/* --- replay byte format -------------------------------------------- */

static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
}
static void put_u32le(uint8_t *p, uint32_t v)
{
    put_u16le(p, (uint16_t)(v & 0xffffu));
    put_u16le(p + 2, (uint16_t)((v >> 16) & 0xffffu));
}
static void put_u64le(uint8_t *p, uint64_t v)
{
    put_u32le(p, (uint32_t)(v & 0xffffffffull));
    put_u32le(p + 4, (uint32_t)(v >> 32));
}
static uint16_t get_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t get_u32le(const uint8_t *p)
{
    return (uint32_t)get_u16le(p) | ((uint32_t)get_u16le(p + 2) << 16);
}
static uint64_t get_u64le(const uint8_t *p)
{
    return (uint64_t)get_u32le(p) | ((uint64_t)get_u32le(p + 4) << 32);
}

size_t toy_sig_write_header(uint8_t out[TOY_SIG_HEADER_SIZE],
                            const toy_sig_info_t *info,
                            uint64_t payload_hash)
{
    for (unsigned i = 0; i < TOY_SIG_HEADER_SIZE; i++) out[i] = 0;
    out[0] = 'T'; out[1] = 'S'; out[2] = 'I'; out[3] = 'G';
    put_u16le(out + 4, (uint16_t)TOY_SIG_FORMAT_VERSION);
    put_u16le(out + 6, (uint16_t)TOY_SIG_HEADER_SIZE);
    put_u32le(out + 8, info->orig_kind ? info->orig_kind : info->kind);
    put_u32le(out + 12, info->device);
    put_u32le(out + 16, info->channels);
    put_u32le(out + 20, info->sample_rate_hz);
    put_u32le(out + 24, info->scale_num);
    put_u32le(out + 28, info->scale2_num);
    put_u64le(out + 32, info->seed);
    put_u64le(out + 40, info->n_samples);
    put_u64le(out + 48, payload_hash);
    put_u32le(out + 56, info->scenario_id);
    return TOY_SIG_HEADER_SIZE;
}

int toy_sig_parse_header(const uint8_t *bytes, size_t len,
                         toy_sig_info_t *info_out,
                         uint64_t *payload_hash_out)
{
    if (!bytes || len < TOY_SIG_HEADER_SIZE) return -1;
    if (bytes[0] != 'T' || bytes[1] != 'S' || bytes[2] != 'I' ||
        bytes[3] != 'G')
        return -1;
    if (get_u16le(bytes + 4) != TOY_SIG_FORMAT_VERSION) return -1;
    if (get_u16le(bytes + 6) != TOY_SIG_HEADER_SIZE) return -1;
    if (get_u32le(bytes + 60) != 0u) return -1; /* reserved */

    toy_sig_info_t info;
    info_zero(&info);
    uint32_t orig = get_u32le(bytes + 8);
    if (orig != TOY_SIG_KIND_SYNTHETIC && orig != TOY_SIG_KIND_LIVE)
        return -1;
    info.kind = TOY_SIG_KIND_REPLAY;
    info.orig_kind = orig;
    info.device = get_u32le(bytes + 12);
    info.channels = get_u32le(bytes + 16);
    info.sample_rate_hz = get_u32le(bytes + 20);
    info.scale_num = get_u32le(bytes + 24);
    info.scale2_num = get_u32le(bytes + 28);
    info.seed = get_u64le(bytes + 32);
    info.n_samples = get_u64le(bytes + 40);
    info.scenario_id = get_u32le(bytes + 56);
    if (info.channels == 0u || info.channels > TOY_SIG_MAX_CHANNELS)
        return -1;
    /* Overflow-safe payload bound: every field above is checked
     * before the product can exceed len by construction. */
    uint64_t sample_bytes = (uint64_t)info.channels * 2ull;
    uint64_t avail = (uint64_t)(len - TOY_SIG_HEADER_SIZE);
    if (info.n_samples > avail / sample_bytes) return -1;

    info.stream_id = get_u64le(bytes + 48);
    if (info_out) *info_out = info;
    if (payload_hash_out) *payload_hash_out = info.stream_id;
    return 0;
}

void toy_sig_write_payload(uint8_t *out, const int16_t *interleaved,
                           uint64_t n_samples, uint32_t channels)
{
    uint64_t total = n_samples * (uint64_t)channels;
    for (uint64_t i = 0; i < total; i++)
        put_u16le(out + 2u * (size_t)i, (uint16_t)interleaved[i]);
}

int toy_sig_replay_init(toy_sig_replay_t *r, const uint8_t *bytes,
                        size_t len)
{
    uint64_t payload_hash = 0;
    if (toy_sig_parse_header(bytes, len, &r->info, &payload_hash) != 0)
        return -1;
    r->payload_hash = payload_hash;
    r->cursor = 0;
    return 0;
}

int toy_sig_replay_next(toy_sig_replay_t *r, const uint8_t *bytes,
                        size_t len, int16_t *out_sample)
{
    if (!bytes || r->cursor >= r->info.n_samples) return 0;
    uint64_t idx = r->cursor * (uint64_t)r->info.channels;
    size_t off = TOY_SIG_HEADER_SIZE + (size_t)idx * 2u;
    if (off + (size_t)r->info.channels * 2u > len) return 0;
    for (uint32_t c = 0; c < r->info.channels; c++)
        out_sample[c] = (int16_t)get_u16le(bytes + off + 2u * (size_t)c);
    r->cursor++;
    return 1;
}

/* --- unified source -------------------------------------------------- */

void toy_sig_source_synth_imu(toy_sig_source_t *src, uint64_t seed,
                              uint32_t sample_rate_hz)
{
    toy_sig_synth_imu_init(&src->synth_imu, seed, sample_rate_hz);
    src->info = src->synth_imu.info;
}

int toy_sig_source_synth_imu_scenario(toy_sig_source_t *src, uint64_t seed,
                                      uint32_t sample_rate_hz,
                                      uint32_t scenario_id)
{
    if (scenario_id == 1u) {
        toy_sig_source_synth_imu(src, seed, sample_rate_hz);
        return 0;
    }
    if (scenario_id == 2u) {
        toy_sig_synth_imu_chest_init(&src->synth_imu, seed, sample_rate_hz);
        src->info = src->synth_imu.info;
        return 0;
    }
    return -1;
}

void toy_sig_source_synth_mic(toy_sig_source_t *src, uint64_t seed,
                              uint32_t sample_rate_hz,
                              uint32_t freq0_millihz,
                              uint32_t freq1_millihz,
                              uint64_t chirp_samples)
{
    toy_sig_synth_mic_init(&src->synth_mic, seed, sample_rate_hz,
                           freq0_millihz, freq1_millihz, chirp_samples);
    src->info = src->synth_mic.info;
}

int toy_sig_source_replay(toy_sig_source_t *src, const uint8_t *bytes,
                          size_t len)
{
    if (toy_sig_replay_init(&src->replay, bytes, len) != 0) return -1;
    src->info = src->replay.info;
    return 0;
}

uint32_t toy_sig_poll(toy_sig_source_t *src,
                      const uint8_t *replay_bytes, size_t replay_len,
                      int16_t *out, uint32_t max_samples,
                      uint32_t *flags_out)
{
    uint32_t flags = 0;
    uint32_t produced = 0;
    if (flags_out) *flags_out = 0;
    if (!src || !out) return 0;

    for (uint32_t i = 0; i < max_samples; i++) {
        int16_t *dst = out + (size_t)i * src->info.channels;
        int ok = 1;
        if (src->info.kind == TOY_SIG_KIND_SYNTHETIC &&
            src->info.device == TOY_SIG_DEVICE_IMU6) {
            toy_sig_synth_imu_next(&src->synth_imu, dst);
        } else if (src->info.kind == TOY_SIG_KIND_SYNTHETIC &&
                   src->info.device == TOY_SIG_DEVICE_MIC1) {
            dst[0] = toy_sig_synth_mic_next(&src->synth_mic);
        } else if (src->info.kind == TOY_SIG_KIND_REPLAY) {
            ok = toy_sig_replay_next(&src->replay, replay_bytes,
                                     replay_len, dst);
            if (!ok) flags |= TOY_SIG_FLAG_END;
        } else {
            ok = 0; /* NONE / LIVE have no generator here */
            flags |= TOY_SIG_FLAG_END;
        }
        if (!ok) break;
        produced++;
    }
    if (produced < max_samples) flags |= TOY_SIG_FLAG_UNDERRUN;
    if (flags_out) *flags_out = flags;
    return produced;
}

