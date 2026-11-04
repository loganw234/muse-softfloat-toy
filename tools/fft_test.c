/* fft_test.c - standalone host test for core/toy_fft.c.
 *
 * Deliberately NOT part of tools/run.sh (which globs the C
 * sources under core and emu only): build and run it separately,
 * mirroring the run.sh host line - gcc -O2 -std=c11 -Wall
 * -Wextra with include dirs libs/muse-softfloat/include and
 * core, sources core/toy_fft.c, this file, and every C file
 * under libs/muse-softfloat/src, output build/fft_test.
 *
 * What it checks:
 *  (a) FFT vs an independent direct O(n^2) DFT written below
 *      straight from the defining sum. Both use the same
 *      softfloat ops and correctly rounded sinPi/cosPi twiddles,
 *      but sum in different orders, so bitwise equality is NOT
 *      expected: each rounding lands in a different place. The
 *      honest metric is ulp distance between the two results
 *      (plus the exact-bit-equality count), printed per size.
 *  (b) Analytically known signals: impulse (flat spectrum,
 *      bitwise exact), DC (single bin, bitwise exact up to the
 *      sign of zero), a cosine sitting exactly on bin k (two
 *      peaks of N/2, checked in ulp), and a forward+inverse
 *      round trip.
 *  (c) A fixed test vector hashed with FNV-1a 64 over the
 *      output bit patterns in an explicit little-endian byte
 *      order, so host, board and WASM builds can compare the
 *      same number later regardless of memory layout.
 *  (d) Host timing at N = 64..1024 (host-only numbers; the
 *      RP2040 posture is argued in docs/study-fft.md, not here).
 *
 * Native double appears in this file ONLY to read finished f64
 * bit patterns for a magnitude sanity check and to print
 * timings. It never feeds the transform.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "toy_fft.h"

#define MAXN TOY_FFT_MAX_N

static f64 re_buf[MAXN], im_buf[MAXN];
static f64 re_ref[MAXN], im_ref[MAXN];
static f64 re_orig[MAXN], im_orig[MAXN];

static int failures = 0;

static void check(int ok, const char *what)
{
    printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) failures++;
}

static f64 f64_from_i32(int32_t v)
{
    msf_flags_t f = 0;
    return msf64_from_i32(v, MSF_ROUND_TIES_TO_EVEN, &f);
}
static f64 f64_from_u32(uint32_t v)
{
    msf_flags_t f = 0;
    return msf64_from_u32(v, MSF_ROUND_TIES_TO_EVEN, &f);
}

/* 2^-8 as a bit pattern: exponent field 1023-8. Structural
 * constant, exact power of two. */
#define F64_2M8 ((f64)(1015ull << 52))

static int is_zero_bits(f64 a) { return (a << 1) == 0; }

/* Ulp distance between two finite f64 patterns, via their
 * sign-magnitude ordering (integer ops only). */
static uint64_t ulp_distance(f64 a, f64 b)
{
    uint64_t ma = a & 0x7fffffffffffffffull;
    uint64_t mb = b & 0x7fffffffffffffffull;
    int na = (a >> 63) != 0 && ma != 0;
    int nb = (b >> 63) != 0 && mb != 0;
    if (na == nb) return ma > mb ? ma - mb : mb - ma;
    return ma + mb;
}

static double read_native(f64 a) /* host-only display read */
{
    double d;
    memcpy(&d, &a, sizeof d);
    return d;
}

/* Direct DFT from the definition X[k] = sum_n x[n] e^{-2pi i kn/N},
 * summed left to right over n - a different rounding order from
 * the FFT's butterfly tree on purpose. Twiddle angle reduced in
 * integers first ((k*n) mod N), then the same exact dyadic
 * 2*m/N cycles argument as the FFT uses. */
static void dft_direct(const f64 *xr, const f64 *xi,
                       f64 *yr, f64 *yi, uint32_t n)
{
    f64 fn = f64_from_u32(n);
    for (uint32_t k = 0; k < n; k++) {
        f64 sr = F64_ZERO, si = F64_ZERO;
        for (uint32_t t = 0; t < n; t++) {
            uint32_t m = (uint32_t)(((uint64_t)k * t) % n);
            f64 wr, wi;
            if (m == 0u) {
                wr = F64_ONE; wi = F64_ZERO;
            } else {
                f64 theta = f64_div(f64_from_u32(2u * m), fn);
                wr = f64_cos_pi(theta);
                wi = f64_neg(f64_sin_pi(theta));
            }
            sr = f64_add(sr, f64_sub(f64_mul(wr, xr[t]), f64_mul(wi, xi[t])));
            si = f64_add(si, f64_add(f64_mul(wr, xi[t]), f64_mul(wi, xr[t])));
        }
        yr[k] = sr;
        yi[k] = si;
    }
}

/* Fixed pseudo-random complex vector: LCG integers in
 * [-1000, 1000] scaled by 2^-8 - every sample exact. */
static void fill_vector(f64 *re, f64 *im, uint32_t n, uint32_t seed)
{
    uint32_t s = seed;
    for (uint32_t i = 0; i < n; i++) {
        s = s * 1664525u + 1013904223u;
        int32_t vr = (int32_t)((s >> 8) % 2001u) - 1000;
        s = s * 1664525u + 1013904223u;
        int32_t vi = (int32_t)((s >> 8) % 2001u) - 1000;
        re[i] = f64_mul(f64_from_i32(vr), F64_2M8);
        im[i] = f64_mul(f64_from_i32(vi), F64_2M8);
    }
}

/* FNV-1a 64 over f64 patterns, least-significant byte first:
 * defined byte order, so the hash is a property of the numbers,
 * not of the machine's memory layout. */
static uint64_t hash_patterns(const f64 *re, const f64 *im, uint32_t n)
{
    uint64_t h = 1469598103934665603ull;
    for (int part = 0; part < 2; part++) {
        const f64 *a = part == 0 ? re : im;
        for (uint32_t i = 0; i < n; i++) {
            uint64_t v = a[i];
            for (int b = 0; b < 8; b++) {
                h ^= (v >> (8 * b)) & 0xffull;
                h *= 1099511628211ull;
            }
        }
    }
    return h;
}

static void test_invalid(void)
{
    printf("invalid inputs:\n");
    re_buf[0] = F64_ONE; im_buf[0] = F64_ZERO;
    check(toy_fft_forward(re_buf, im_buf, 0u) == -1, "n=0 rejected");
    check(toy_fft_forward(re_buf, im_buf, 3u) == -1, "n=3 rejected");
    check(toy_fft_forward(re_buf, im_buf, 100u) == -1, "n=100 rejected");
    check(toy_fft_forward(re_buf, im_buf, 4096u) == -1, "n=4096 rejected (> MAX)");
    check(toy_fft_forward(0, im_buf, 64u) == -1, "NULL re rejected");
    check(toy_fft_inverse(re_buf, 0, 64u) == -1, "NULL im rejected");
    check(re_buf[0] == F64_ONE && is_zero_bits(im_buf[0]),
          "buffers untouched after rejection");
    check(toy_fft_forward(re_buf, im_buf, 1u) == 0 && re_buf[0] == F64_ONE,
          "n=1 is the identity");
}

static void test_known(uint32_t n)
{
    char msg[128];

    /* Impulse at n=0 -> every bin exactly 1 + 0i, bitwise. */
    for (uint32_t i = 0; i < n; i++) { re_buf[i] = F64_ZERO; im_buf[i] = F64_ZERO; }
    re_buf[0] = F64_ONE;
    toy_fft_forward(re_buf, im_buf, n);
    int ok = 1;
    for (uint32_t i = 0; i < n; i++)
        if (re_buf[i] != F64_ONE || !is_zero_bits(im_buf[i])) ok = 0;
    snprintf(msg, sizeof msg, "impulse -> flat 1+0i bitwise (N=%u)", n);
    check(ok, msg);

    /* DC -> X[0] = N exactly, all other bins zero (sign of zero
     * may vary through twiddle*0 products, so test magnitude 0). */
    for (uint32_t i = 0; i < n; i++) { re_buf[i] = F64_ONE; im_buf[i] = F64_ZERO; }
    toy_fft_forward(re_buf, im_buf, n);
    ok = re_buf[0] == f64_from_u32(n) && is_zero_bits(im_buf[0]);
    for (uint32_t i = 1; i < n; i++)
        if (!is_zero_bits(re_buf[i]) || !is_zero_bits(im_buf[i])) ok = 0;
    snprintf(msg, sizeof msg, "DC -> X[0]=N exact, rest zero (N=%u)", n);
    check(ok, msg);
}

static void test_cosine_bin(uint32_t n, uint32_t k)
{
    char msg[192];
    f64 fn = f64_from_u32(n);
    for (uint32_t t = 0; t < n; t++) {
        uint32_t m = (uint32_t)(((uint64_t)k * t) % n);
        f64 theta = f64_div(f64_from_u32(2u * m), fn);
        re_buf[t] = f64_cos_pi(theta);
        im_buf[t] = F64_ZERO;
    }
    toy_fft_forward(re_buf, im_buf, n);
    f64 half_n = f64_mul(fn, F64_HALF);
    uint64_t peak_ulp = ulp_distance(re_buf[k], half_n);
    uint64_t mir_ulp = ulp_distance(re_buf[n - k], half_n);
    double worst_other = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        if (i == k || i == n - k) continue;
        double vr = read_native(re_buf[i]); if (vr < 0) vr = -vr;
        double vi = read_native(im_buf[i]); if (vi < 0) vi = -vi;
        if (vr > worst_other) worst_other = vr;
        if (vi > worst_other) worst_other = vi;
    }
    double peak_im = read_native(im_buf[k]); if (peak_im < 0) peak_im = -peak_im;
    printf("  cosine bin %u (N=%u): peak ulp err %llu/%llu, "
           "peak |im| %.3g, worst other bin %.3g (N/2 = %u)\n",
           k, n, (unsigned long long)peak_ulp, (unsigned long long)mir_ulp,
           peak_im, worst_other, n / 2u);
    snprintf(msg, sizeof msg, "cosine peaks within 8192 ulp of N/2 (N=%u)", n);
    check(peak_ulp <= 8192u && mir_ulp <= 8192u, msg);
    snprintf(msg, sizeof msg, "cosine leakage < N * 2^-40 (N=%u)", n);
    check(worst_other < (double)n * 0x1p-40 && peak_im < (double)n * 0x1p-40, msg);
}

static void test_vs_dft(uint32_t n)
{
    fill_vector(re_buf, im_buf, n, 0x1234567u);
    memcpy(re_orig, re_buf, n * sizeof(f64));
    memcpy(im_orig, im_buf, n * sizeof(f64));
    dft_direct(re_orig, im_orig, re_ref, im_ref, n);
    toy_fft_forward(re_buf, im_buf, n);
    uint64_t max_ulp = 0;
    uint32_t exact = 0;
    double scale = 0.0, max_abs_err = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        uint64_t dr = ulp_distance(re_buf[i], re_ref[i]);
        uint64_t di = ulp_distance(im_buf[i], im_ref[i]);
        if (dr > max_ulp) max_ulp = dr;
        if (di > max_ulp) max_ulp = di;
        if (dr == 0u) exact++;
        if (di == 0u) exact++;
        double vr = read_native(re_ref[i]); if (vr < 0) vr = -vr;
        double vi = read_native(im_ref[i]); if (vi < 0) vi = -vi;
        if (vr > scale) scale = vr;
        if (vi > scale) scale = vi;
        double er = read_native(re_buf[i]) - read_native(re_ref[i]);
        double ei = read_native(im_buf[i]) - read_native(im_ref[i]);
        if (er < 0) er = -er;
        if (ei < 0) ei = -ei;
        if (er > max_abs_err) max_abs_err = er;
        if (ei > max_abs_err) max_abs_err = ei;
    }
    /* Raw per-bin ulp explodes on near-cancellation bins (tiny
     * bin, tiny ulp, ordinary absolute error). The meaningful
     * gate is error in ulps of the largest bin ("full-scale
     * ulp"); the raw number is printed alongside, honestly. */
    double fs_ulp = max_abs_err / (scale * 0x1p-52);
    printf("  N=%-5u max ulp vs DFT: %llu   full-scale ulp err: %.1f   "
           "exact components: %u/%u\n",
           n, (unsigned long long)max_ulp, fs_ulp, exact, 2u * n);
    char msg[128];
    snprintf(msg, sizeof msg,
             "FFT within 256 full-scale ulp of direct DFT (N=%u)", n);
    check(fs_ulp <= 256.0, msg);
}

static void test_roundtrip_and_hash(void)
{
    const uint32_t n = 256u;
    fill_vector(re_buf, im_buf, n, 0x5eed1234u);
    memcpy(re_orig, re_buf, n * sizeof(f64));
    memcpy(im_orig, im_buf, n * sizeof(f64));
    toy_fft_forward(re_buf, im_buf, n);
    uint64_t h = hash_patterns(re_buf, im_buf, n);
    printf("  fixed vector (N=256, seed 0x5eed1234) output hash: %016llx\n",
           (unsigned long long)h);
    toy_fft_inverse(re_buf, im_buf, n);
    uint64_t max_ulp = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint64_t dr = ulp_distance(re_buf[i], re_orig[i]);
        uint64_t di = ulp_distance(im_buf[i], im_orig[i]);
        if (dr > max_ulp) max_ulp = dr;
        if (di > max_ulp) max_ulp = di;
    }
    printf("  forward+inverse round trip max ulp: %llu\n",
           (unsigned long long)max_ulp);
    check(max_ulp <= 4096u, "round trip within 4096 ulp");
}

static void test_timing(void)
{
    static const struct { uint32_t n; int reps; } cases[] = {
        { 64u, 2000 }, { 128u, 1000 }, { 256u, 400 },
        { 512u, 150 }, { 1024u, 60 }
    };
    uint64_t sink = 0;
    printf("host timing (this machine only, -O2):\n");
    for (size_t c = 0; c < sizeof cases / sizeof cases[0]; c++) {
        uint32_t n = cases[c].n;
        int reps = cases[c].reps;
        fill_vector(re_buf, im_buf, n, 0xabc123u);
        memcpy(re_orig, re_buf, n * sizeof(f64));
        memcpy(im_orig, im_buf, n * sizeof(f64));
        clock_t t0 = clock();
        for (int r = 0; r < reps; r++) {
            memcpy(re_buf, re_orig, n * sizeof(f64));
            memcpy(im_buf, im_orig, n * sizeof(f64));
            toy_fft_forward(re_buf, im_buf, n);
            sink = sink * 1099511628211ull ^ (re_buf[0] + im_buf[n - 1u]);
        }
        clock_t t1 = clock();
        double us = (double)(t1 - t0) * 1.0e6 / (double)CLOCKS_PER_SEC
                    / (double)reps;
        printf("  N=%-5u %10.1f us/transform  (%d reps)\n", n, us, reps);
    }
    printf("  (checksum sink %016llx - keeps the loops honest)\n",
           (unsigned long long)sink);
}

int main(void)
{
    printf("fft_test: toy_fft forward/inverse, MAX_N=%u\n", TOY_FFT_MAX_N);
    test_invalid();
    printf("known signals:\n");
    test_known(64u);
    test_known(256u);
    test_cosine_bin(64u, 5u);
    test_cosine_bin(256u, 17u);
    printf("FFT vs independent direct DFT (bitwise equality NOT expected;\n"
           "different summation order rounds differently - ulp is the metric):\n");
    test_vs_dft(16u);
    test_vs_dft(64u);
    test_vs_dft(128u);
    test_vs_dft(256u);
    printf("round trip + cross-target hash:\n");
    test_roundtrip_and_hash();
    test_timing();
    printf("fft_test: %s (%d failure%s)\n",
           failures == 0 ? "PASS" : "FAIL", failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
