/* toy_math.h - thin binary64 wrappers over muse-softfloat.
 *
 * Every floating-point operation in the sim core goes through the
 * library, so host, RP2040 and RP2350 produce identical bit
 * patterns. Flags are discarded per call (sim code does not branch
 * on them); rounding is ties-to-even everywhere. Display-only
 * conversion to native double lives in the renderers, never in
 * state updates.
 */
#ifndef TOY_MATH_H
#define TOY_MATH_H

#include "muse_softfloat.h"

typedef msf64_t f64;

#define F64_ZERO ((f64)0x0000000000000000ull)
#define F64_ONE  ((f64)0x3ff0000000000000ull)
#define F64_TWO  ((f64)0x4000000000000000ull)
#define F64_HALF ((f64)0x3fe0000000000000ull)
#define F64_SIGN ((f64)0x8000000000000000ull)

static inline f64 f64_dec(const char *s)
{
    msf_flags_t fl = 0;
    return msf64_from_decimal(s, 0, MSF_ROUND_TIES_TO_EVEN, &fl);
}
static inline f64 f64_add(f64 a, f64 b) { msf_flags_t f = 0; return msf64_add(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_sub(f64 a, f64 b) { msf_flags_t f = 0; return msf64_sub(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_mul(f64 a, f64 b) { msf_flags_t f = 0; return msf64_mul(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_div(f64 a, f64 b) { msf_flags_t f = 0; return msf64_div(a, b, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_sqrt(f64 a)       { msf_flags_t f = 0; return msf64_sqrt(a, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_fma(f64 a, f64 b, f64 c) { msf_flags_t f = 0; return msf64_fma(a, b, c, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_neg(f64 a) { return a ^ F64_SIGN; }
static inline f64 f64_abs(f64 a) { return msf64_abs(a); }
static inline f64 f64_next_up(f64 a) { msf_flags_t f = 0; return msf64_next_up(a, &f); }
static inline f64 f64_sin_pi(f64 a) { msf_flags_t f = 0; return msf64_sin_pi(a, MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_cos_pi(f64 a) { msf_flags_t f = 0; return msf64_cos_pi(a, MSF_ROUND_TIES_TO_EVEN, &f); }

/* sin/cos in radians via the library's argument-in-pi forms:
 * sin(x) = sinPi(x/pi). The multiply by 1/pi is one correctly
 * rounded step; deterministic everywhere. Cartridges whose state
 * winds without bound should hold angles in cycles instead. */
static inline f64 f64_sin_rad(f64 x)
{
    static const f64 inv_pi = (f64)0x3fd45f306dc9c883ull; /* 1/pi to nearest */
    return f64_sin_pi(f64_mul(x, inv_pi));
}
static inline f64 f64_cos_rad(f64 x)
{
    static const f64 inv_pi = (f64)0x3fd45f306dc9c883ull;
    return f64_cos_pi(f64_mul(x, inv_pi));
}

#endif
