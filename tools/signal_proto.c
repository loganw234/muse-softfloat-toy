/* signal_proto.c - standalone prototype for docs/study-signals.md.
 *
 * NOT wired into tools/run.sh and not picked up by the emu/board
 * globs over core/ and emu/: build it on its own:
 *
 *   gcc -O2 -std=c11 -Wall -Wextra -Icore \
 *       tools/signal_proto.c core/toy_signal.c -o build/signal_proto
 *   ./build/signal_proto [replay.tsig]
 *
 * Demonstrates, with exit code as the verdict:
 *   1. seeded synthetic IMU stream, 60 s at 100 Hz (gravity +
 *      tremor + one tap), generated twice => identical FNV-1a;
 *   2. sanity statistics computed in INTEGER arithmetic only
 *      (int64 sums, integer sqrt; no native FP anywhere here);
 *   3. serialise to a TSIG byte buffer and to a file, replay
 *      both => identical hash; malformed headers rejected;
 *   4. synthetic microphone tone (+ hiss) and chirp: repeatable
 *      hash and the expected zero-crossing counts.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_signal.h"

#define IMU_RATE 100u
#define IMU_SECONDS 60u
#define IMU_N (IMU_RATE * IMU_SECONDS) /* 6000 */
#define IMU_CH 6u
#define SEED UINT64_C(0x5eed)

#define MIC_RATE 8000u
#define MIC_N 8000u /* 1 s */

static int failures;

static void expect(int cond, const char *what)
{
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) failures++;
}

static uint64_t isqrt_u64(uint64_t x)
{
    uint64_t r = 0, bit = UINT64_C(1) << 62;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

static int16_t *generate_imu(uint64_t seed)
{
    int16_t *buf = (int16_t *)malloc((size_t)IMU_N * IMU_CH * sizeof(int16_t));
    if (!buf) { fprintf(stderr, "oom\n"); exit(2); }
    toy_sig_source_t src;
    toy_sig_source_synth_imu(&src, seed, IMU_RATE);
    /* One sample per poll: the same shape a host proof loop
     * would use, one 100 Hz sample per cart step. */
    for (uint32_t i = 0; i < IMU_N; i++) {
        uint32_t flags = 0;
        uint32_t got = toy_sig_poll(&src, NULL, 0,
                                    buf + (size_t)i * IMU_CH, 1, &flags);
        if (got != 1u || flags != 0u) {
            fprintf(stderr, "synth poll failed at %u\n", i);
            exit(2);
        }
    }
    return buf;
}

static void imu_stats(const int16_t *buf)
{
    static const char *names[6] = { "ax", "ay", "az", "gx", "gy", "gz" };
    printf("IMU statistics (integer: int64 sums, mean = sum/n trunc):\n");
    for (uint32_t a = 0; a < 6u; a++) {
        int32_t mn = INT32_MAX, mx = INT32_MIN;
        int64_t sum = 0, diff_sum = 0;
        for (uint32_t i = 0; i < IMU_N; i++) {
            int32_t v = buf[(size_t)i * IMU_CH + a];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            sum += v;
            if (i) {
                int32_t d = v - buf[(size_t)(i - 1) * IMU_CH + a];
                diff_sum += d < 0 ? -(int64_t)d : d;
            }
        }
        printf("  %s min %6d max %6d mean %7lld  mean|d| %lld\n",
               names[a], mn, mx, (long long)(sum / (int64_t)IMU_N),
               (long long)(diff_sum / (int64_t)(IMU_N - 1)));
    }

    /* Gravity magnitude outside the tap window [2900, 3200). */
    int64_t mag2_sum = 0;
    uint32_t mag_n = 0;
    int32_t tap_best = 0;
    uint32_t tap_at = 0;
    for (uint32_t i = 0; i < IMU_N; i++) {
        int32_t ax = buf[(size_t)i * IMU_CH + 0];
        int32_t ay = buf[(size_t)i * IMU_CH + 1];
        int32_t az = buf[(size_t)i * IMU_CH + 2];
        if (i < 2900u || i >= 3200u) {
            mag2_sum += (int64_t)ax * ax + (int64_t)ay * ay +
                        (int64_t)az * az;
            mag_n++;
        }
        int32_t dev = az - 15864;
        if (dev < 0) dev = -dev;
        if (dev > tap_best) { tap_best = dev; tap_at = i; }
    }
    uint64_t mag = isqrt_u64((uint64_t)(mag2_sum / (int64_t)mag_n));
    uint64_t mag_mg = mag * 1000u / 16384u;
    printf("  |gravity| mean (tap window excluded): %" PRIu64
           " LSB = %" PRIu64 " mg at 16384 LSB/g\n", mag, mag_mg);
    expect(mag_mg >= 980u && mag_mg <= 1020u,
           "gravity magnitude within 2% of 1 g");
    printf("  largest az deviation from rest: %d LSB at sample %u"
           " (tap scheduled at 3000)\n", tap_best, tap_at);
    expect(tap_best > 4000 && tap_at >= 3000u && tap_at < 3064u,
           "tap spike visible at the scheduled sample");
    /* Tremor is present but small: ax range is the 0.25 g tilt
     * circle (8192 LSB peak-to-peak) plus a few hundred LSB. */
    int32_t axmn = INT32_MAX, axmx = INT32_MIN;
    for (uint32_t i = 0; i < IMU_N; i++) {
        int32_t v = buf[(size_t)i * IMU_CH + 0];
        if (v < axmn) axmn = v;
        if (v > axmx) axmx = v;
    }
    int32_t axpp = axmx - axmn;
    printf("  ax peak-to-peak: %d LSB (tilt circle alone: 8192)\n", axpp);
    expect(axpp > 8300 && axpp < 10300,
           "tremor adds visible noise on top of the tilt circle");
}

static uint64_t replay_all_hash(const uint8_t *bytes, size_t len)
{
    toy_sig_source_t src;
    if (toy_sig_source_replay(&src, bytes, len) != 0) {
        fprintf(stderr, "replay init failed\n");
        exit(2);
    }
    int16_t block[256 * IMU_CH];
    uint64_t seen = 0;
    /* Hash incrementally in the same defined byte order. */
    uint64_t h = 1469598103934665603ull;
    for (;;) {
        uint32_t flags = 0;
        uint32_t got = toy_sig_poll(&src, bytes, len, block, 256, &flags);
        for (uint32_t i = 0; i < got * IMU_CH; i++) {
            uint16_t u = (uint16_t)block[i];
            h ^= (uint64_t)(u & 0xffu); h *= 1099511628211ull;
            h ^= (uint64_t)((u >> 8) & 0xffu); h *= 1099511628211ull;
        }
        seen += got;
        if (flags & TOY_SIG_FLAG_END) break;
        if (got == 0u) break;
    }
    if (seen != IMU_N) {
        fprintf(stderr, "replay produced %" PRIu64 " samples\n", seen);
        exit(2);
    }
    return h;
}

static void mic_run(const char *label, uint32_t f0, uint32_t f1,
                    uint64_t *hash_out, uint32_t *zc_out)
{
    int16_t buf[MIC_N];
    toy_sig_source_t src;
    toy_sig_source_synth_mic(&src, SEED, MIC_RATE, f0, f1, MIC_N);
    uint32_t flags = 0;
    uint32_t got = toy_sig_poll(&src, NULL, 0, buf, MIC_N, &flags);
    if (got != MIC_N) { fprintf(stderr, "mic poll failed\n"); exit(2); }
    *hash_out = toy_sig_hash_samples(buf, MIC_N, 1u);
    uint32_t zc = 0;
    int32_t mn = INT32_MAX, mx = INT32_MIN;
    int64_t sum = 0;
    for (uint32_t i = 0; i < MIC_N; i++) {
        int32_t v = buf[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        if (i && ((buf[i - 1] < 0) != (buf[i] < 0))) zc++;
    }
    *zc_out = zc;
    printf("  %s: hash %016llx min %d max %d mean %lld zero-crossings %u\n",
           label, (unsigned long long)*hash_out, mn, mx,
           (long long)(sum / (int64_t)MIC_N), zc);
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "build/signal_proto_replay.tsig";

    printf("== synthetic IMU: %u s at %u Hz, seed 0x%llx, scenario 1 (desk)\n",
           IMU_SECONDS, IMU_RATE, (unsigned long long)SEED);
    int16_t *a = generate_imu(SEED);
    int16_t *b = generate_imu(SEED);
    uint64_t ha = toy_sig_hash_samples(a, IMU_N, IMU_CH);
    uint64_t hb = toy_sig_hash_samples(b, IMU_N, IMU_CH);
    printf("  run A hash %016llx\n  run B hash %016llx\n",
           (unsigned long long)ha, (unsigned long long)hb);
    expect(ha == hb, "same seed twice => identical stream hash");
    expect(memcmp(a, b, (size_t)IMU_N * IMU_CH * sizeof(int16_t)) == 0,
           "same seed twice => byte-identical samples");
    free(b);
    int16_t *c = generate_imu(SEED + 1u);
    uint64_t hc = toy_sig_hash_samples(c, IMU_N, IMU_CH);
    free(c);
    printf("  seed 0x%llx hash %016llx\n",
           (unsigned long long)(SEED + 1u), (unsigned long long)hc);
    expect(hc != ha, "different seed => different stream hash");

    imu_stats(a);

    printf("== replay equivalence (TSIG v1)\n");
    toy_sig_info_t info;
    memset(&info, 0, sizeof info);
    info.kind = TOY_SIG_KIND_SYNTHETIC;
    info.orig_kind = TOY_SIG_KIND_SYNTHETIC;
    info.device = TOY_SIG_DEVICE_IMU6;
    info.channels = IMU_CH;
    info.sample_rate_hz = IMU_RATE;
    info.scale_num = 16384u;
    info.scale2_num = 128u;
    info.scenario_id = 1u;
    info.seed = SEED;
    info.n_samples = IMU_N;
    size_t payload_len = (size_t)IMU_N * IMU_CH * 2u;
    size_t total_len = TOY_SIG_HEADER_SIZE + payload_len;
    uint8_t *bytes = (uint8_t *)malloc(total_len);
    if (!bytes) { fprintf(stderr, "oom\n"); return 2; }
    toy_sig_write_header(bytes, &info, ha);
    toy_sig_write_payload(bytes + TOY_SIG_HEADER_SIZE, a, IMU_N, IMU_CH);
    expect(toy_sig_fnv1a64(bytes + TOY_SIG_HEADER_SIZE, payload_len) == ha,
           "payload bytes hash to the stream hash (defined byte order)");

    toy_sig_info_t parsed;
    uint64_t parsed_hash = 0;
    expect(toy_sig_parse_header(bytes, total_len, &parsed, &parsed_hash) == 0 &&
           parsed_hash == ha && parsed.kind == TOY_SIG_KIND_REPLAY &&
           parsed.orig_kind == TOY_SIG_KIND_SYNTHETIC &&
           parsed.n_samples == IMU_N && parsed.seed == SEED,
           "header round-trips; kind becomes REPLAY of SYNTHETIC");
    expect(toy_sig_parse_header(bytes, total_len - 1u, &parsed,
                                &parsed_hash) != 0,
           "truncated stream rejected");
    uint8_t saved = bytes[0];
    bytes[0] = 'X';
    expect(toy_sig_parse_header(bytes, total_len, &parsed,
                                &parsed_hash) != 0,
           "bad magic rejected");
    bytes[0] = saved;

    uint64_t hr = replay_all_hash(bytes, total_len);
    printf("  replay-of-buffer hash %016llx\n", (unsigned long long)hr);
    expect(hr == ha, "serialise -> buffer -> replay => identical hash");

    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "cannot write %s\n", path); return 2; }
    if (fwrite(bytes, 1, total_len, f) != total_len) {
        fprintf(stderr, "short write\n"); return 2;
    }
    fclose(f);
    f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot read %s\n", path); return 2; }
    uint8_t *file_bytes = (uint8_t *)malloc(total_len);
    if (!file_bytes) { fprintf(stderr, "oom\n"); return 2; }
    size_t rd = fread(file_bytes, 1, total_len, f);
    fclose(f);
    uint64_t hf = (rd == total_len) ? replay_all_hash(file_bytes, rd) : 0;
    printf("  replay-of-file hash %016llx (%s, %zu bytes)\n",
           (unsigned long long)hf, path, rd);
    expect(hf == ha, "serialise -> file -> replay => identical hash");

    printf("== synthetic microphone: 1 s at %u Hz\n", MIC_RATE);
    uint64_t h_tone1, h_tone2, h_chirp;
    uint32_t zc_tone1, zc_tone2, zc_chirp;
    mic_run("tone 440 Hz + hiss, run A", 440000u, 440000u, &h_tone1, &zc_tone1);
    mic_run("tone 440 Hz + hiss, run B", 440000u, 440000u, &h_tone2, &zc_tone2);
    expect(h_tone1 == h_tone2, "mic tone regenerates identically");
    expect(zc_tone1 >= 840u && zc_tone1 <= 920u,
           "440 Hz tone has ~880 zero crossings in 1 s");
    mic_run("chirp 440->880 Hz + hiss", 440000u, 880000u, &h_chirp, &zc_chirp);
    expect(h_chirp != h_tone1 && zc_chirp > zc_tone1 + 200u,
           "chirp sweeps upward (more crossings than the fixed tone)");

    /* Scenario 2 "chest" (additive, Wave T3): modelled SCG
     * magnitudes on az at rest +1 g. Regeneration must be exact,
     * the stream must differ from scenario 1, az must rest near
     * 16384 with only mg-class excursions (no desk tap here). */
    printf("== synthetic IMU scenario 2 (chest): 40 s at %u Hz\n", IMU_RATE);
    {
        enum { CHEST_N = 4000 };
        static int16_t ca[CHEST_N * 6], cb[CHEST_N * 6];
        uint64_t hch[2];
        for (int run = 0; run < 2; run++) {
            toy_sig_source_t src;
            int16_t *dst = run ? cb : ca;
            if (toy_sig_source_synth_imu_scenario(&src, UINT64_C(0xbea75),
                                                  IMU_RATE, 2u) != 0) {
                fprintf(stderr, "chest source failed\n");
                return 2;
            }
            if (run == 0)
                printf("  scenario 2 stream id %016llx (scenario %u)\n",
                       (unsigned long long)src.info.stream_id,
                       src.info.scenario_id);
            for (uint32_t i = 0; i < CHEST_N; i++) {
                uint32_t flags2 = 0;
                if (toy_sig_poll(&src, NULL, 0, dst + (size_t)i * 6u, 1,
                                 &flags2) != 1u || flags2 != 0u) {
                    fprintf(stderr, "chest poll failed at %u\n", i);
                    return 2;
                }
            }
            hch[run] = toy_sig_hash_samples(dst, CHEST_N, 6u);
        }
        printf("  chest run A hash %016llx run B hash %016llx\n",
               (unsigned long long)hch[0], (unsigned long long)hch[1]);
        expect(hch[0] == hch[1] && hch[0] != ha,
               "chest regenerates identically and differs from desk");
        int64_t az_sum = 0;
        int32_t az_dev_max = 0, ax_abs_max = 0;
        uint32_t over60 = 0;
        for (uint32_t i = 0; i < CHEST_N; i++) {
            int32_t ax = ca[(size_t)i * 6u];
            int32_t az = ca[(size_t)i * 6u + 2u];
            int32_t dev = az - 16384;
            az_sum += az;
            if (dev < 0) dev = -dev;
            if (dev > az_dev_max) az_dev_max = dev;
            if (az - 16384 > 60) over60++;
            if (ax < 0) ax = -ax;
            if (ax > ax_abs_max) ax_abs_max = ax;
        }
        int64_t az_mean = az_sum / CHEST_N;
        printf("  chest az mean %lld max|dev| %d LSB, ax max %d LSB, "
               "samples >60 LSB over rest: %u\n",
               (long long)az_mean, az_dev_max, ax_abs_max, over60);
        expect(az_mean > 16200 && az_mean < 16600 &&
               az_dev_max > 200 && az_dev_max < 450 && over60 > 100u,
               "az rests at +1 g with breathing+pulse mg excursions");
    }

    printf("signal_proto: %s (%d failure%s)\n",
           failures ? "FAIL" : "PASS", failures, failures == 1 ? "" : "s");
    free(file_bytes);
    free(bytes);
    free(a);
    return failures ? 1 : 0;
}
