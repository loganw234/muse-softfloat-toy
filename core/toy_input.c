/* toy_input.c - softfloat value plumbing for the input seam.
 *
 * Only the helpers that must call the floating library live
 * here (decimal/int -> binary64 bits, SET_PARAM construction).
 * The integer helpers and apply helpers are static inline in
 * toy_input.h / toy_cart.h so the board proof firmware, whose
 * sources are enumerated explicitly, links them without a new
 * CMake line. Emu and WASM builds glob the core sources and
 * pick this file up automatically.
 */
#include "toy_cart.h"
#include "toy_math.h"

uint64_t toy_f64_bits_from_dec(const char *dec)
{
    return (uint64_t)f64_dec(dec);
}

uint64_t toy_f64_bits_from_i32(int32_t a)
{
    msf_flags_t f = 0;
    return (uint64_t)msf64_from_i64((int64_t)a, MSF_ROUND_TIES_TO_EVEN, &f);
}

uint64_t toy_f64_bits_from_milli(int32_t milli)
{
    f64 num = (f64)toy_f64_bits_from_i32(milli);
    return (uint64_t)f64_div(num, f64_dec("1000"));
}

int toy_param_event(const toy_cart_t *cart, uint32_t idx,
                    int32_t ival, const char *fdec_or_null,
                    toy_event_t *out)
{
    if (!cart || !out || !cart->params || idx >= cart->param_count)
        return -1;
    const toy_param_t *p = &cart->params[idx];
    toy_event_t ev = { 0 };
    ev.type = TOY_EV_SET_PARAM;
    ev.id = idx;
    switch (p->kind) {
    case TOY_PARAM_I32:
    case TOY_PARAM_ENUM:
        ev.v = ival;
        break;
    case TOY_PARAM_F64: {
        const char *dec = fdec_or_null ? fdec_or_null : p->f_def;
        if (!dec) return -1;
        ev.bits = toy_f64_bits_from_dec(dec);
        break;
    }
    default:
        return -1;
    }
    *out = ev;
    return 0;
}
