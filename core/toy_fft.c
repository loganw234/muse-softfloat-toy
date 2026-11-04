/* toy_fft.c - iterative radix-2 decimation-in-time FFT.
 *
 * Structure: in-place bit-reversal permutation (integer ops
 * only), then log2(n) butterfly stages of length len = 2, 4,
 * ..., n. This is the standard textbook factorisation of the
 * DFT sum; nothing here is adapted from any existing FFT
 * implementation.
 *
 * Twiddles: at stage len, butterfly offset j needs
 * W = exp(-sign * 2*pi*i*j/len). The angle in cycles for the
 * library's argument-in-pi trig is theta = 2*j/len, formed as
 * f64_div(from_u32(2*j), from_u32(len)). Both operands are
 * exact small integers and the quotient is a dyadic rational
 * with a short binary expansion, so the division is exact -
 * theta carries no rounding at all. f64_cos_pi / f64_sin_pi
 * then deliver the correctly rounded twiddle components for
 * that exact angle, identical bit patterns on every target.
 *
 * The alternative - iterating W *= W_step across a stage -
 * would trade ~2 trig calls per (stage, j) pair for one complex
 * multiply per butterfly, but each recurrence step rounds, so
 * twiddle error grows along the stage and the factors drift
 * off the unit circle. Here accuracy is the product: every
 * twiddle is independently correctly rounded, and the cost is
 * n-1 twiddle pairs per transform (measured in
 * tools/fft_test.c, sized in docs/study-fft.md). j == 0 is
 * special-cased to W = 1 exactly, which is both faster and
 * keeps DC/impulse responses exact.
 *
 * Rounding inside a butterfly is one rounding per softfloat
 * add/sub/mul, ties-to-even, as everywhere in the sim core.
 * The transform is therefore a fixed function of its input bit
 * patterns: same input, same output bits, on any target. It is
 * NOT expected to agree bitwise with a direct O(n^2) DFT sum,
 * which rounds in a different order; tools/fft_test.c measures
 * the actual ulp agreement.
 */
#include "toy_fft.h"

static f64 f64_from_u32(uint32_t v)
{
    msf_flags_t f = 0;
    return msf64_from_u32(v, MSF_ROUND_TIES_TO_EVEN, &f);
}

int toy_fft_valid_size(uint32_t n)
{
    return n >= 1u && n <= TOY_FFT_MAX_N && (n & (n - 1u)) == 0u;
}

static void bit_reverse(f64 *re, f64 *im, uint32_t n)
{
    uint32_t j = 0;
    for (uint32_t i = 1; i < n; i++) {
        uint32_t bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) {
            f64 tr = re[i]; re[i] = re[j]; re[j] = tr;
            f64 ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
}

/* inverse_dir = 0: forward twiddles exp(-2*pi*i*j/len).
 * inverse_dir = 1: conjugate twiddles, unscaled (the caller in
 * toy_fft_inverse applies 1/n afterwards). */
static void fft_run(f64 *re, f64 *im, uint32_t n, int inverse_dir)
{
    bit_reverse(re, im, n);
    for (uint32_t len = 2; len <= n; len <<= 1) {
        uint32_t half = len >> 1;
        f64 flen = f64_from_u32(len);
        /* j is the outer loop so each twiddle is computed once
         * per stage and reused across blocks: n-1 twiddle pairs
         * per transform, no table storage. Butterflies within a
         * stage touch disjoint index pairs, so this ordering
         * computes exactly the same values as block-major order. */
        for (uint32_t j = 0; j < half; j++) {
            f64 wr, wi;
            if (j == 0u) {
                wr = F64_ONE;
                wi = F64_ZERO;
            } else {
                /* Exact dyadic angle: cosPi/sinPi of 2j/len
                 * give cos/sin(2*pi*j/len). See file header. */
                f64 theta = f64_div(f64_from_u32(2u * j), flen);
                wr = f64_cos_pi(theta);
                f64 s = f64_sin_pi(theta);
                wi = inverse_dir ? s : f64_neg(s);
            }
            for (uint32_t base = 0; base < n; base += len) {
                uint32_t a = base + j;
                uint32_t b = a + half;
                f64 xr = re[b], xi = im[b];
                f64 tr = f64_sub(f64_mul(wr, xr), f64_mul(wi, xi));
                f64 ti = f64_add(f64_mul(wr, xi), f64_mul(wi, xr));
                re[b] = f64_sub(re[a], tr);
                im[b] = f64_sub(im[a], ti);
                re[a] = f64_add(re[a], tr);
                im[a] = f64_add(im[a], ti);
            }
        }
    }
}

int toy_fft_forward(f64 *re, f64 *im, uint32_t n)
{
    if (re == 0 || im == 0 || !toy_fft_valid_size(n)) return -1;
    if (n == 1u) return 0;
    fft_run(re, im, n, 0);
    return 0;
}

int toy_fft_inverse(f64 *re, f64 *im, uint32_t n)
{
    if (re == 0 || im == 0 || !toy_fft_valid_size(n)) return -1;
    if (n == 1u) return 0;
    fft_run(re, im, n, 1);
    f64 inv_n = f64_div(F64_ONE, f64_from_u32(n)); /* exact: n is 2^k */
    for (uint32_t i = 0; i < n; i++) {
        re[i] = f64_mul(re[i], inv_n);
        im[i] = f64_mul(im[i], inv_n);
    }
    return 0;
}
