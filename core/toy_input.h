/* toy_input.h - input & settings seam (study-input section 5).
 *
 * One fixed-size integer-only event type carries every input
 * source (touch, IMU-derived summaries, keys/buttons/encoder,
 * browser pointer, emu scripts) to a cartridge's optional
 * pre-step apply() callback. Events are quantised to STEP
 * BOUNDARIES: an event assigned to step s is applied after step
 * s-1 completes and before step s integrates; step 0 is the
 * post-reset setup slot. There are no sub-step timestamps.
 *
 * Cartridges store only event EFFECTS in state, so the existing
 * state hash covers inputs. A cartridge with apply == NULL is
 * deaf: runners skip queueing for it and its runs are identical
 * to the pre-seam behaviour.
 *
 * Drain fate (study verifier gap D8, decided here): at most
 * TOY_MAX_EVENTS_PER_STEP events are applied before one step.
 * Events beyond the cap CARRY to the next step boundary in
 * their original FIFO order; they are never dropped at drain
 * time and never reordered. Drops happen only at enqueue time
 * on a full live queue (see toy_queue_push), which proof and
 * replayed runs must be authored to avoid. Every runner drains
 * through these shared helpers, so the fate is identical on
 * host, board and WASM.
 */
#ifndef TOY_INPUT_H
#define TOY_INPUT_H

#include <stdint.h>

/* The descriptor type is defined in toy_cart.h, which includes
 * this header for the event/param/script payload types. */
typedef struct toy_cart_s toy_cart_t;

/* Limits. A runner never applies more than MAX_EVENTS_PER_STEP
 * events before one step(); the live queue holds at most QUEUE. */
#define TOY_MAX_EVENTS_PER_STEP 16u
#define TOY_INPUT_QUEUE_LEN     64u
#define TOY_MAX_PARAMS          12u   /* per cart, keep UI on glass sane */
#define TOY_MAX_PROOF_INPUTS  4096u   /* entries; board flash is finite */
#define TOY_COORD_MAX         1500   /* milli-radius clamp */

enum {
    TOY_EV_NONE = 0,
    TOY_EV_POINTER_DOWN = 1,  /* id=contact, x/y=milli-radius */
    TOY_EV_POINTER_MOVE = 2,
    TOY_EV_POINTER_UP   = 3,  /* also used for cancel/lift-outside */
    TOY_EV_KEY_DOWN     = 4,  /* id=TOY_KEY_*, v=1 if auto-repeat */
    TOY_EV_KEY_UP       = 5,
    TOY_EV_BUTTON_DOWN  = 6,  /* id=button index 0.. */
    TOY_EV_BUTTON_UP    = 7,
    TOY_EV_ENCODER      = 8,  /* id=encoder index, v=summed detents */
    TOY_EV_TILT         = 9,  /* x/y=milli-g quantised */
    TOY_EV_SHAKE        = 10, /* v=intensity bucket 1..3 */
    TOY_EV_FACE         = 11, /* v: 0=unknown 1=up 2=down */
    TOY_EV_SET_PARAM    = 12, /* id=param index; v or bits=value */
    /* Turn-the-whole-device gesture: v = signed milliturn delta
     * (1000 = one full turn, sign = direction), id=dial index
     * (0 = the device itself). IMU-derived: a runner integrates
     * gyro rotation about the display axis into milliturns and
     * emits the quantised delta. The producer (IMU layer) lands
     * with signals/board work; the vocabulary lands now. */
    TOY_EV_DIAL         = 13
};
#define TOY_EV_MAX_KNOWN TOY_EV_DIAL

/* Small runner-independent key namespace (not HID, not JS codes). */
enum {
    TOY_KEY_LEFT = 1, TOY_KEY_RIGHT, TOY_KEY_UP, TOY_KEY_DOWN,
    TOY_KEY_A, TOY_KEY_B, TOY_KEY_X, TOY_KEY_Y,
    TOY_KEY_SPACE, TOY_KEY_ENTER, TOY_KEY_ESC,
    TOY_KEY_PLUS, TOY_KEY_MINUS
};

/* What a cart listens to; runners queue/show UI only for these. */
#define TOY_CAP_POINTER  (1u<<0)
#define TOY_CAP_KEYS     (1u<<1)
#define TOY_CAP_BUTTONS  (1u<<2)
#define TOY_CAP_ENCODER  (1u<<3)
#define TOY_CAP_TILT     (1u<<4)
#define TOY_CAP_SHAKE    (1u<<5)
#define TOY_CAP_FACE     (1u<<6)
#define TOY_CAP_PARAMS   (1u<<7)  /* implies params table present */
#define TOY_CAP_DIAL     (1u<<8)

/* Fixed 32 bytes: six 32-bit words + one 64-bit word, integers
 * only, no pointers, no native float. Producers zero the whole
 * struct, then fill the fields named for the type; every other
 * field stays 0. Consumers ignore fields not named for the
 * type. There is deliberately no timestamp: the step an event
 * is applied to IS its time. */
typedef struct {
    uint32_t type;   /* TOY_EV_* */
    uint32_t id;     /* contact / key / button / encoder / param idx */
    int32_t  x;      /* pointer: milli-radius, +right */
    int32_t  y;      /* pointer: milli-radius, +down (display space) */
    int32_t  v;      /* delta / bucket / repeat / int param value */
    uint32_t flags;  /* reserved, must be 0 in v1 */
    uint64_t bits;   /* SET_PARAM f64 payload: IEEE bits, else 0 */
} toy_event_t;

_Static_assert(sizeof(toy_event_t) == 32, "toy_event_t must be 32 bytes");

/* Cap bit a runner must see before queueing this event type.
 * SET_PARAM is gated by TOY_CAP_PARAMS; NONE/unknown need none
 * (they are never queued). */
static inline uint32_t toy_event_cap_mask(uint32_t type)
{
    switch (type) {
    case TOY_EV_POINTER_DOWN:
    case TOY_EV_POINTER_MOVE:
    case TOY_EV_POINTER_UP:   return TOY_CAP_POINTER;
    case TOY_EV_KEY_DOWN:
    case TOY_EV_KEY_UP:       return TOY_CAP_KEYS;
    case TOY_EV_BUTTON_DOWN:
    case TOY_EV_BUTTON_UP:    return TOY_CAP_BUTTONS;
    case TOY_EV_ENCODER:      return TOY_CAP_ENCODER;
    case TOY_EV_TILT:         return TOY_CAP_TILT;
    case TOY_EV_SHAKE:        return TOY_CAP_SHAKE;
    case TOY_EV_FACE:         return TOY_CAP_FACE;
    case TOY_EV_SET_PARAM:    return TOY_CAP_PARAMS;
    case TOY_EV_DIAL:         return TOY_CAP_DIAL;
    default:                  return 0;
    }
}

/* --- parameters and proof script (descriptor payloads) --- */

enum { TOY_PARAM_I32 = 1, TOY_PARAM_ENUM = 2, TOY_PARAM_F64 = 3 };
#define TOY_PARAM_RESET_ONLY (1u<<0) /* runner re-resets on change */
typedef struct {
    const char *name;        /* "bodies" - script + persistence key */
    const char *label;       /* "Bodies" - UI label */
    uint32_t kind;           /* TOY_PARAM_* */
    uint32_t flags;          /* TOY_PARAM_* flags, else 0 */
    int32_t i_min, i_max, i_def;        /* I32 and ENUM */
    const char *f_min, *f_max, *f_def;  /* F64: decimal strings */
    const char *const *enum_names;      /* ENUM only; i_max+1 names */
} toy_param_t;

/* Canonical proof inputs: sorted by step, stable order within a
 * step. step==0 means "after reset, before step 1". */
typedef struct {
    uint32_t step;
    toy_event_t ev;
} toy_proof_input_t;

/* --- coordinates: display pixels -> milli-radius ---
 * Origin at display centre, +x right, +y down; unit = 1/1000
 * of the inscribed radius size_px/2 (same centre convention as
 * toy_cx). Integer math only; round-to-nearest, ties away from
 * zero; clamped to +/-TOY_COORD_MAX. Producers pass raw pixels
 * plus size: no JS/driver float scaling touches sim values. */
static inline int32_t toy_px_to_milli(int32_t px, int32_t size_px)
{
    if (size_px <= 1) return 0;
    int64_t half = (int64_t)(size_px / 2);
    int64_t num = ((int64_t)px - half) * 1000;
    int64_t q = num / half;
    int64_t r = num % half;
    int64_t ar = r < 0 ? -r : r;
    if (2 * ar >= half) q += (num < 0 ? -1 : 1); /* ties away from 0 */
    if (q > TOY_COORD_MAX) q = TOY_COORD_MAX;
    if (q < -TOY_COORD_MAX) q = -TOY_COORD_MAX;
    return (int32_t)q;
}

static inline void toy_pointer_event(toy_event_t *ev, uint32_t type,
                                     uint32_t id, int32_t px, int32_t py,
                                     int32_t size_px)
{
    toy_event_t e = { 0 };
    e.type = type;
    e.id = id;
    e.x = toy_px_to_milli(px, size_px);
    e.y = toy_px_to_milli(py, size_px);
    *ev = e;
}

/* --- live event queue (runner-owned, never in cart state) ---
 * Bounded at TOY_INPUT_QUEUE_LEN. Enqueue coalesces POINTER_MOVE
 * per contact in place (latest position wins, first slot kept).
 * On a full queue the NEW event is dropped, except POINTER_UP,
 * which evicts the oldest queued POINTER_MOVE to make room (a
 * lost release wedges cart state; a lost move does not). Drops
 * are counted runner-side only, never in state. Drain applies
 * at most TOY_MAX_EVENTS_PER_STEP per step boundary; the rest
 * CARRY to the next boundary in order (see header note). */
typedef struct {
    toy_event_t ev[TOY_INPUT_QUEUE_LEN];
    uint32_t len;
    uint32_t dropped; /* enqueue-time drops, diagnostics only */
} toy_queue_t;

static inline void toy_queue_clear(toy_queue_t *q)
{
    q->len = 0;
    q->dropped = 0;
}

/* Returns 1 if queued (or coalesced), 0 if dropped. */
static inline int toy_queue_push(toy_queue_t *q, const toy_event_t *ev)
{
    if (ev->type == TOY_EV_POINTER_MOVE) {
        for (uint32_t i = 0; i < q->len; i++) {
            if (q->ev[i].type == TOY_EV_POINTER_MOVE &&
                q->ev[i].id == ev->id) {
                q->ev[i] = *ev; /* latest wins, slot kept */
                return 1;
            }
        }
    }
    if (q->len < TOY_INPUT_QUEUE_LEN) {
        q->ev[q->len++] = *ev;
        return 1;
    }
    if (ev->type == TOY_EV_POINTER_UP) {
        for (uint32_t i = 0; i < q->len; i++) {
            if (q->ev[i].type == TOY_EV_POINTER_MOVE) {
                for (uint32_t j = i + 1; j < q->len; j++)
                    q->ev[j - 1] = q->ev[j];
                q->len--;
                q->dropped++; /* the evicted move */
                q->ev[q->len++] = *ev;
                return 1;
            }
        }
    }
    q->dropped++;
    return 0;
}

static inline int toy_queue_push_pointer(toy_queue_t *q, uint32_t type,
                                         uint32_t id, int32_t px,
                                         int32_t py, int32_t size_px)
{
    toy_event_t ev;
    toy_pointer_event(&ev, type, id, px, py, size_px);
    return toy_queue_push(q, &ev);
}

/* FNV-1a 64 over canonical little-endian fields (not raw struct
 * bytes): identifies a proof script in logs and pins. Per entry:
 * step, type, id, x, y, v, flags (4 bytes each, signed fields by
 * bit pattern), then bits (8 bytes). */
static inline uint64_t toy_hash_inputs(const toy_proof_input_t *in, uint32_t n)
{
    uint64_t h = 1469598103934665603ull; /* FNV-1a 64 offset basis */
    for (uint32_t i = 0; i < n; i++) {
        uint32_t w32[7];
        w32[0] = in[i].step;
        w32[1] = in[i].ev.type;
        w32[2] = in[i].ev.id;
        w32[3] = (uint32_t)in[i].ev.x;
        w32[4] = (uint32_t)in[i].ev.y;
        w32[5] = (uint32_t)in[i].ev.v;
        w32[6] = in[i].ev.flags;
        for (int w = 0; w < 7; w++)
            for (int b = 0; b < 4; b++) {
                h ^= (uint64_t)((w32[w] >> (8 * b)) & 0xffu);
                h *= 1099511628211ull;
            }
        for (int b = 0; b < 8; b++) {
            h ^= (uint64_t)((in[i].ev.bits >> (8 * b)) & 0xffu);
            h *= 1099511628211ull;
        }
    }
    return h;
}

/* --- softfloat value plumbing (core/toy_input.c) ---
 * IEEE binary64 bit patterns, produced only through the library
 * (decimal parse / integer conversion), never from a host double.
 * Assignable to a cart's f64 state fields (f64 is msf64_t, a
 * uint64_t). milli uses one softfloat divide by 1000. */
uint64_t toy_f64_bits_from_dec(const char *dec);
uint64_t toy_f64_bits_from_i32(int32_t a);
uint64_t toy_f64_bits_from_milli(int32_t milli);

/* Make a SET_PARAM event for param idx of cart. I32/ENUM take
 * ival; F64 takes a decimal string (fdec_or_null == NULL means
 * the declared default f_def). Range clamping is the cart's job
 * in apply(). Returns 0 on success, -1 on bad idx/kind/NULL. */
int toy_param_event(const toy_cart_t *cart, uint32_t idx,
                    int32_t ival, const char *fdec_or_null,
                    toy_event_t *out);

/* Descriptor-taking apply helpers (toy_apply_proof_at,
 * toy_apply_live, toy_apply_queue, toy_cart_check_input) are
 * declared at the end of toy_cart.h, after the full descriptor
 * definition they read. Runners keep owning their step loops;
 * the helpers only apply events, they never call step(). */

#endif

