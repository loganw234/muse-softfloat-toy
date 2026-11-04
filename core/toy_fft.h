/* toy_fft.h - iterative radix-2 complex FFT in binary64.
 *
 * Own-code decimation-in-time FFT written from the textbook DFT
 * definition (docs/study-fft.md). Every floating-point operation
 * goes through toy_math.h over muse-softfloat, and every twiddle
 * factor is computed per stage from f64_cos_pi / f64_sin_pi at an
 * exactly representable angle in cycles, so the twiddles - and
 * therefore the whole transform - are bit-identical on host,
 * RP2040, RP2350 and WebAssembly. No native float/double, no heap,
 * no globals written, no third-party FFT code.
 *
 * Buffers are caller-provided split real/imaginary arrays of f64
 * bit patterns, transformed in place. A cartridge holds them
 * inside its POD state: the two arrays cost 16*N bytes, which
 * against TOY_MAX_STATE_SIZE (65536) means
 *
 *     N =   64 ->  1024 bytes      N =  512 ->  8192 bytes
 *     N =  128 ->  2048 bytes      N = 1024 -> 16384 bytes
 *     N =  256 ->  4096 bytes      N = 2048 -> 32768 bytes
 *
 * N = 4096 would consume the entire state budget on its own and
 * is therefore not supported: TOY_FFT_MAX_N is 2048, leaving at
 * least half the state for the signal history, magnitudes and
 * whatever else the cartridge keeps. See docs/study-fft.md for
 * the full state-fit table (including an optional magnitude
 * array at +8*N bytes).
 *
 * Forward transform (unscaled, engineering sign convention):
 *
 *     X[k] = sum_{n=0}^{N-1} x[n] * exp(-2*pi*i*k*n/N)
 *
 * The inverse uses the conjugate sign and includes the 1/N
 * scaling, so inverse(forward(x)) returns x up to rounding.
 * A caller that wants a 1/N-scaled forward spectrum scales the
 * output itself; the transform does not guess.
 */
#ifndef TOY_FFT_H
#define TOY_FFT_H

#include <stdint.h>
#include "toy_math.h"

/* Largest supported transform length. 16*2048 = 32768 bytes of
 * buffers, half of TOY_MAX_STATE_SIZE. */
#define TOY_FFT_MAX_N 2048u

/* 1 when n is a transform length toy_fft_forward accepts:
 * a power of two with 1 <= n <= TOY_FFT_MAX_N. */
int toy_fft_valid_size(uint32_t n);

/* In-place forward FFT of the n complex samples (re[i], im[i]).
 * Returns 0 on success. Returns -1, leaving both buffers
 * untouched, when re or im is NULL or n is not a valid size.
 * Bounded work: (n/2)*log2(n) butterflies, n-1 twiddle pairs. */
int toy_fft_forward(f64 *re, f64 *im, uint32_t n);

/* In-place inverse FFT (conjugate twiddles, result scaled by
 * 1/n). Same contract and error behaviour as toy_fft_forward. */
int toy_fft_inverse(f64 *re, f64 *im, uint32_t n);

#endif
