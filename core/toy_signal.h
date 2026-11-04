/* toy_signal.h - deterministic sample sources for sensor carts.
 *
 * Design: docs/study-signals.md (SIG-1 landed in Wave T2: the
 * cartridge feed seam in core/toy_cart.h consumes this). One
 * interface, three source kinds: SYNTHETIC (generated from a
 * seed, integer only), REPLAY (a recorded byte stream replayed
 * bit-exact) and LIVE (a real driver, later; only the kind tag
 * exists here, and proof runners instantiate SYNTHETIC only).
 *
 * Rules this file lives by, so a synthetic stream is identical on
 * host, RP2040/RP2350 and wasm32:
 *  - no native float/double, no <math.h>, no rand(), no wall clock
 *    anywhere in the sample path (docs/CARTRIDGES.md state rules);
 *  - only int32/int64/uint32/uint64 arithmetic whose overflow and
 *    shift behaviour is fully defined (see toy_sig_asr64);
 *  - samples are sensor-native int16 counts (IMU raw axes, mic
 *    PCM), quantised at generation, never float-then-converted.
 *
 * A source is NOT cartridge state. It is owned by the host layer
 * (emu/board/WASM glue) which polls it at step boundaries and
 * hands plain int16 frames to the cartridge; only the copied
 * values, the source kind and the stream id enter hashed state.
 * The replay byte buffer likewise stays host-side: the cursor
 * below holds offsets only, never a pointer.
 */
#ifndef TOY_SIGNAL_H
#define TOY_SIGNAL_H

#include <stddef.h>
#include <stdint.h>
#include "toy_input.h" /* toy_event_t for the dial derivation below */

/* Source kinds. The kind is part of the labelling contract: it is
 * copied into cartridge state and rendered, so an emulated run can
 * never silently masquerade as a measured one. 0 = no signal, a
 * state a cartridge must show as such, never as zero samples. */
#define TOY_SIG_KIND_NONE      0u
#define TOY_SIG_KIND_SYNTHETIC 1u
#define TOY_SIG_KIND_REPLAY    2u
#define TOY_SIG_KIND_LIVE      3u

/* Devices (sample layouts). */
#define TOY_SIG_DEVICE_IMU6 1u /* ax,ay,az,gx,gy,gz int16 counts */
#define TOY_SIG_DEVICE_MIC1 2u /* one int16 PCM channel          */

#define TOY_SIG_FORMAT_VERSION 1u
#define TOY_SIG_HEADER_SIZE 64u
#define TOY_SIG_MAX_CHANNELS 6u

/* Poll flags (out-parameter of toy_sig_poll). */
#define TOY_SIG_FLAG_END      1u /* replay exhausted (or would be)  */
#define TOY_SIG_FLAG_UNDERRUN 2u /* fewer samples than requested    */

/* Everything a cartridge and a proof line need to know about a
 * stream, in plain integers. stream_id is the FNV-1a 64 hash of
 * the serialised payload once known, else of the parameter block;
 * two runs may be compared only if their stream_ids match. */
typedef struct {
    uint32_t kind;          /* TOY_SIG_KIND_*                     */
    uint32_t orig_kind;     /* for REPLAY: SYNTHETIC or LIVE      */
    uint32_t device;        /* TOY_SIG_DEVICE_*                   */
    uint32_t channels;      /* interleaved int16 per sample       */
    uint32_t sample_rate_hz;
    uint32_t scale_num;     /* IMU: LSB per g. MIC: full-scale=1. */
    uint32_t scale2_num;    /* IMU: LSB per deg/s. MIC: unused=0. */
    uint32_t scenario_id;   /* synthetic scenario selector        */
    uint64_t seed;          /* synthetic seed; 0 for recordings   */
    uint64_t stream_id;
    uint64_t n_samples;     /* 0 = unbounded (live / long synth)  */
} toy_sig_info_t;

/* Interleaved channels for a TOY_SIG_DEVICE_*, or 0 if unknown. */
uint32_t toy_sig_device_channels(uint32_t device);
/* "none"/"synthetic"/"replay"/"live" for proof headers/labels. */
const char *toy_sig_kind_name(uint32_t kind);
/* stream_id for an unbounded synthetic stream: FNV-1a 64 over
 * the canonical little-endian parameter block (kind, orig_kind,
 * device, channels, rate, scales, scenario, seed, n_samples).
 * The synthetic inits set info.stream_id to this, so every
 * runner names the same (spec, seed) stream identically before
 * a single sample exists. Replay streams instead carry the
 * payload hash from their header - a different domain, and the
 * kind word says which. Two runs are comparable only if BOTH
 * kind and stream_id match. */
uint64_t toy_sig_param_stream_id(const toy_sig_info_t *info);

/* --- cartridge-facing declaration (SIG-1) ------------------------ */

/* Bounds for one step's feed frame, so every runner can own a
 * fixed static frame buffer: at most 1024 samples per step,
 * at most 6144 interleaved int16 values (1024 x 6 channels). */
#define TOY_SIG_MAX_SAMPLES_PER_STEP 1024u
#define TOY_SIG_MAX_FRAME_VALUES \
    (TOY_SIG_MAX_SAMPLES_PER_STEP * TOY_SIG_MAX_CHANNELS)

/* Static signal need in the cartridge descriptor (core/toy_cart.h,
 * appended after state_version). All-zero means no signal need.
 * When feed() is set, every field binds statically (study 4 rule
 * 3): runners refuse to start unless the instantiated source
 * matches device/channel/rate, no resampling in v1.
 * proof_kind names the source kind the standard proof run uses;
 * SIG-1 implements exactly one proof kind, TOY_SIG_KIND_SYNTHETIC,
 * on host/board/WASM alike, so a feed cart's proof hashes are
 * comparable everywhere. LIVE has no proof run by definition;
 * REPLAY proof bytes are a later parcel. scenario_id is 1 in v1
 * (IMU "desk", MIC 440 Hz tone); new scenarios are new proof
 * vectors, never edits to a pinned one. */
typedef struct {
    uint32_t device;           /* TOY_SIG_DEVICE_*               */
    uint32_t proof_kind;       /* TOY_SIG_KIND_SYNTHETIC in v1   */
    uint32_t scenario_id;      /* synthetic scenario, 1 in v1    */
    uint32_t sample_rate_hz;   /* rate the cart's dt assumes     */
    uint32_t samples_per_step; /* fixed frames fed before step() */
    uint64_t seed;             /* synthetic proof seed           */
} toy_sig_spec_t;

/* The labelling rule (study 7) as a POD block a sensor cart
 * embeds in its state, so kind/orig/stream identity sits inside
 * the hashed bytes: a synthetic run can never hash-collide with
 * a replay-of-live run on identical samples, and "no source" is
 * KIND_NONE, rendered as NO SIG, never a flat zero trace.
 * Fields ordered 8-byte first per the checkpoint layout rule. */
typedef struct {
    uint64_t stream_id;   /* from info at feed time             */
    uint64_t samples_seen;/* frames fed so far (next seq)       */
    uint32_t kind;        /* TOY_SIG_KIND_*; NONE until fed     */
    uint32_t orig_kind;   /* for REPLAY: SYNTHETIC or LIVE      */
    uint32_t device;      /* TOY_SIG_DEVICE_*                   */
    uint32_t gap_count;   /* underruns fed as gaps (live only)  */
} toy_sig_label_t;

/* Call from reset() after the whole-struct memset: kind NONE,
 * the visible no-signal state until the first feed. */
void toy_sig_label_reset(toy_sig_label_t *l);
/* Call from feed() with the same info/n/flags the runner handed
 * over: copies identity into state, advances samples_seen, and
 * counts a gap if UNDERRUN is set (proof runners fail loud on
 * underrun and never reach feed(); a future live runner feeds
 * the gap so the cart can show it). */
void toy_sig_label_feed(toy_sig_label_t *l, const toy_sig_info_t *info,
                        uint32_t n_samples, uint32_t flags);

/* --- dial derivation: IMU gyro-z -> TOY_EV_DIAL -------------------
 * The producer side of the turn-the-whole-device gesture
 * (docs/WAVE-T2.md). Integer yaw integration over interleaved
 * IMU6 frames: each sample adds gz*1000 to a numerator whose
 * unit milliturn is scale_lsb_per_dps * 360 * rate (LSB/(deg/s)
 * x deg/turn x samples/s, over the x1000 for milliturns). Whole
 * milliturns are extracted trunc-towards-zero (symmetric in
 * both directions) and the sub-milliturn remainder is kept, so
 * many samples below one milliturn each still accumulate
 * exactly, and rotation past a full turn never wraps: two
 * turns read 2000 milliturns, not 0. All arithmetic is int64
 * with bounded remainder; no angle is ever materialised.
 * POD state: a cartridge may embed it (deriving dial effects
 * from its feed in hashed state) or a runner may own it and
 * queue the events to apply(). Feed it per-step chunks
 * (<= TOY_SIG_MAX_SAMPLES_PER_STEP) so the returned delta fits
 * the event's int32 v with enormous margin; larger calls are
 * clamped on return while total_milliturns stays exact. */
typedef struct {
    int64_t accum_num;        /* sub-milliturn remainder numerator */
    int64_t total_milliturns; /* signed total emitted so far     */
    uint32_t scale_lsb_per_dps; /* info.scale2_num, e.g. 128       */
    uint32_t sample_rate_hz;    /* info.sample_rate_hz             */
} toy_dial_t;

void toy_dial_reset(toy_dial_t *d, uint32_t scale_lsb_per_dps,
                    uint32_t sample_rate_hz);
void toy_dial_reset_for_info(toy_dial_t *d, const toy_sig_info_t *info);
/* Integrate n_samples interleaved frames (channels must be the
 * IMU6 layout, 6; anything else integrates nothing). Returns
 * the whole-milliturn delta produced by THIS call. */
int32_t toy_dial_feed(toy_dial_t *d, const int16_t *frames,
                      uint32_t n_samples, uint32_t channels);
/* Same, then fill *out as a zero-based TOY_EV_DIAL event
 * (id 0 = the device itself, v = delta) iff delta != 0.
 * Returns 1 when an event was produced, else 0. */
int toy_dial_feed_event(toy_dial_t *d, const int16_t *frames,
                        uint32_t n_samples, uint32_t channels,
                        toy_event_t *out);

/* --- deterministic primitives ----------------------------------- */

/* SplitMix64 (Steele/Lea/Flood splittable generator, used here as
 * a plain sequential PRNG). One uint64 of state, period 2^64,
 * all operations defined unsigned arithmetic: identical output
 * on every conforming C11 platform. Not cryptographic; it only
 * has to make the same noise everywhere. */
typedef struct { uint64_t s; } toy_rng_t;
void toy_rng_seed(toy_rng_t *r, uint64_t seed);
uint32_t toy_rng_u32(toy_rng_t *r);
/* Uniform bipolar in [-32767, 32767]. */
int32_t toy_rng_bipolar_q15(toy_rng_t *r);

/* Arithmetic (floor) shift right for int64 without relying on the
 * implementation-defined signed >> of C11. shift in [1, 62]. */
int64_t toy_sig_asr64(int64_t v, unsigned shift);

/* Sine of a phase accumulator (2^32 = one cycle), Q15 out in
 * [-32767, 32767], table + linear interpolation, integer only. */
int32_t toy_sig_sin_q15(uint32_t phase);
static inline int32_t toy_sig_cos_q15(uint32_t phase)
{
    return toy_sig_sin_q15(phase + 0x40000000u);
}
/* Phase increment for freq_millihz at sample_rate_hz, rounded to
 * nearest: round(f * 2^32 / Fs) in uint64. Deterministic. */
uint32_t toy_sig_phase_inc(uint32_t freq_millihz, uint32_t sample_rate_hz);

int16_t toy_sig_clamp_i16(int32_t v);

/* Band-limited noise: white noise through two one-pole lowpasses,
 * output = fast - slow (a crude bandpass). Integer IIR with
 * floor-shift rounding; state is two int32s. Output is Q15-ish
 * amplitude well inside int32; callers scale to LSB. */
typedef struct { int32_t lp_fast, lp_slow; } toy_bandnoise_t;
int32_t toy_bandnoise_next(toy_bandnoise_t *b, toy_rng_t *r);

/* --- hash (same FNV-1a 64 as toy_hash_state, core/toy_carts.c) -- */
uint64_t toy_sig_fnv1a64(const void *bytes, size_t n);
/* Hash interleaved samples in their DEFINED byte order (little
 * endian, low byte first, sample-major) so the stream hash does
 * not depend on host endianness or struct layout. */
uint64_t toy_sig_hash_samples(const int16_t *interleaved,
                              uint64_t n_samples, uint32_t channels);

/* --- synthetic IMU ---------------------------------------------- */

/* Scenario 1 "desk": tilted device slowly describing a circle
 * (gravity 1 g, tilt amplitude tilt_amp_lsb, az = sqrt(1-A^2)),
 * band-limited tremor on every axis, one decaying tap transient
 * at tap_at. Gyro channels carry tremor only in this prototype
 * (not yet coherent with the accel attitude; see the study). */
typedef struct {
    toy_sig_info_t info;
    int32_t tilt_amp_lsb;   /* gravity XY circle radius, LSB      */
    int32_t az_const_lsb;   /* gravity Z component, LSB           */
    int32_t tremor_amp;     /* bandnoise -> accel LSB gain (Q8)   */
    int32_t gyro_amp;       /* bandnoise -> gyro LSB gain (Q8)    */
    uint64_t tap_at;        /* sample index of the tap            */
    int32_t tap_amp_lsb;    /* tap peak on az, LSB                */
    uint32_t tilt_inc;      /* phase increments                   */
    uint32_t tap_inc;
    /* running state */
    uint64_t seq;
    uint32_t tilt_phase, tap_phase;
    int32_t tap_env;
    toy_rng_t rng;
    toy_bandnoise_t bn[6];
} toy_sig_synth_imu_t;

/* Defaults: 100 Hz, +/-2 g at 16384 LSB/g, +/-256 deg/s at
 * 128 LSB/(deg/s) (assumed QMI8658C-class settings, to confirm
 * against the datasheet when the kits land). */
void toy_sig_synth_imu_init(toy_sig_synth_imu_t *s, uint64_t seed,
                            uint32_t sample_rate_hz);
void toy_sig_synth_imu_next(toy_sig_synth_imu_t *s, int16_t out6[6]);

/* --- synthetic microphone --------------------------------------- */

/* Tone (optional linear chirp via inc_delta per sample) plus
 * band-limited noise, int16 PCM. */
typedef struct {
    toy_sig_info_t info;
    int32_t tone_amp;       /* PCM peak                           */
    int32_t noise_amp;      /* bandnoise -> PCM gain (Q8)         */
    uint32_t phase_inc;
    int32_t inc_delta;      /* chirp: phase_inc change per sample */
    uint64_t seq;
    uint32_t phase;
    toy_rng_t rng;
    toy_bandnoise_t bn;
} toy_sig_synth_mic_t;

void toy_sig_synth_mic_init(toy_sig_synth_mic_t *s, uint64_t seed,
                            uint32_t sample_rate_hz,
                            uint32_t freq0_millihz, uint32_t freq1_millihz,
                            uint64_t chirp_samples);
int16_t toy_sig_synth_mic_next(toy_sig_synth_mic_t *s);

/* --- replay ------------------------------------------------------ */

/* TSIG v1 byte format (all integers little-endian, packed by
 * explicit byte stores, never by struct fwrite):
 *   [0..3]   magic "TSIG"
 *   [4..5]   u16 format version (=1)
 *   [6..7]   u16 header size (=64)
 *   [8..11]  u32 orig_kind (SYNTHETIC or LIVE)
 *   [12..15] u32 device
 *   [16..19] u32 channels
 *   [20..23] u32 sample_rate_hz
 *   [24..27] u32 scale_num   [28..31] u32 scale2_num
 *   [32..39] u64 seed (synthetic) or 0
 *   [40..47] u64 n_samples
 *   [48..55] u64 payload FNV-1a 64 in defined byte order
 *   [56..59] u32 scenario_id [60..63] reserved, must be 0
 * Payload follows: n_samples * channels int16 LE, sample-major. */
size_t toy_sig_write_header(uint8_t out[TOY_SIG_HEADER_SIZE],
                            const toy_sig_info_t *info,
                            uint64_t payload_hash);
/* 0 ok, -1 malformed. Payload bounds are checked against len. */
int toy_sig_parse_header(const uint8_t *bytes, size_t len,
                         toy_sig_info_t *info_out,
                         uint64_t *payload_hash_out);
void toy_sig_write_payload(uint8_t *out, const int16_t *interleaved,
                           uint64_t n_samples, uint32_t channels);

typedef struct {
    toy_sig_info_t info;
    uint64_t payload_hash;
    uint64_t cursor;        /* next sample index; offsets only    */
} toy_sig_replay_t;

/* Validates header. Does NOT verify the payload hash (that costs
 * a full pass; hosts choose when - proof runs must). */
int toy_sig_replay_init(toy_sig_replay_t *r, const uint8_t *bytes,
                        size_t len);
/* Reads one interleaved sample from bytes[len]. 1 ok, 0 at end. */
int toy_sig_replay_next(toy_sig_replay_t *r, const uint8_t *bytes,
                        size_t len, int16_t *out_sample);

/* --- unified source (prototype form of the study's descriptor) --- */

typedef struct {
    toy_sig_info_t info;    /* snapshot hosts show in proof lines */
    toy_sig_synth_imu_t synth_imu;
    toy_sig_synth_mic_t synth_mic;
    toy_sig_replay_t replay;
} toy_sig_source_t;

void toy_sig_source_synth_imu(toy_sig_source_t *src, uint64_t seed,
                              uint32_t sample_rate_hz);
void toy_sig_source_synth_mic(toy_sig_source_t *src, uint64_t seed,
                              uint32_t sample_rate_hz,
                              uint32_t freq0_millihz,
                              uint32_t freq1_millihz,
                              uint64_t chirp_samples);
int toy_sig_source_replay(toy_sig_source_t *src, const uint8_t *bytes,
                          size_t len);

/* Poll up to max_samples interleaved samples into out (which must
 * hold max_samples * info.channels int16). replay_bytes/len are
 * used only for REPLAY sources and may be NULL/0 otherwise.
 * Returns samples produced; flags report END / UNDERRUN. A proof
 * host treats UNDERRUN as a failed run, never as zero samples. */
uint32_t toy_sig_poll(toy_sig_source_t *src,
                      const uint8_t *replay_bytes, size_t replay_len,
                      int16_t *out, uint32_t max_samples,
                      uint32_t *flags_out);

#endif
