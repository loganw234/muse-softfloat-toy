/* wasm_glue.c - browser-facing instance of the sim core.
 *
 * One selected cartridge, one state allocation, one framebuffer.
 * The same state can be rendered at any panel size without
 * stepping it: panel size is presentation only (the web mock
 * device switches 240/360/466/480/custom glass over one live
 * state; the hash line proves continuity). Rendering never
 * feeds back into state on any runner.
 */
#include <stddef.h>
#include <stdint.h>
#include "toy_util.h"
#include "toy_cart.h"

/* Largest square panel the mock device can present. The render
 * path is size-parametric end to end (toy_fb_t carries w/h and
 * carts draw in centre/radius fractions), so this buffer bound
 * is the only size cap in the WASM runner. Cost: the static
 * framebuffer below grows with the square - 1024*1024 RGB565
 * pixels = 2 MiB of linear memory (360*360 was 253125 bytes). */
#define TOY_WASM_MAX_FB 1024

/* Exports beyond site/build.py's --export list (the input seam,
 * params introspection and session log) are named here so this
 * file stays the single place the mock's JS surface is defined;
 * wasm-ld honours export_name alongside explicit --export. */
#define TOY_EXPORT(name) __attribute__((export_name(name)))

static _Alignas(8) unsigned char toy_state[TOY_MAX_STATE_SIZE];
static uint16_t toy_pixels[TOY_WASM_MAX_FB * TOY_WASM_MAX_FB];
static toy_fb_t toy_fb;
static const toy_cart_t *toy_cart;
static uint64_t toy_steps_done;

/* Input seam (runner-owned, never in state): a 64-entry live
 * queue fed by JS through the toy_wasm_queue_* pushes, plus a
 * cursor into the selected cart's compiled-in proof_inputs.
 * toy_wasm_step applies, before each sub-step, proof entries
 * due at that step and then queued live events, sharing the
 * 16-per-step cap (proof first; excess live events CARRY to
 * the next step boundary in order). The Node checker never
 * queues anything, so its unmodified step(1) proof loop sees
 * the pure proof replay and pinned hashes stay valid; deaf
 * carts (apply == NULL, no script) are a no-op throughout. */
static toy_queue_t toy_live_queue;
static uint32_t toy_proof_cursor;

/* Session log (TOYSESSION1, runner-owned, never in state):
 * every live event DRAINED by toy_wasm_step is appended as one
 * text line with the absolute step it applied before, so a
 * downloaded session replays bit-exactly in the emulator. Proof
 * entries are not logged (they are compiled into the cart and
 * replay themselves). Format is normative in docs/CARTRIDGES.md
 * ("Web mock device"). The buffer is capped; event lines stop
 * TOY_SESSION_TAIL bytes short of the cap so the TRUNCATED and
 * END lines written at export time always fit, and events past
 * the cap are counted, never silently lost from the record of
 * loss. At ~40 bytes a line the cap holds roughly 13000 events. */
#define TOY_SESSION_CAP  524288u /* 512 KiB of linear memory */
#define TOY_SESSION_TAIL 80u     /* reserved for TRUNCATED + END */
static char toy_session_buf[TOY_SESSION_CAP];
static uint32_t toy_session_len;     /* header + event lines only */
static uint32_t toy_session_dropped; /* event lines past the cap */

static void session_putc(char c)
{
    if (toy_session_len < TOY_SESSION_CAP - TOY_SESSION_TAIL)
        toy_session_buf[toy_session_len++] = c;
}

static void session_str(const char *s)
{
    while (*s) session_putc(*s++);
}

static void session_u64(uint64_t v)
{
    char tmp[20];
    int n = 0;
    do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v);
    while (n > 0) session_putc(tmp[--n]);
}

static void session_i64(int64_t v)
{
    if (v < 0) { session_putc('-'); session_u64((uint64_t)(-(v + 1)) + 1u); }
    else session_u64((uint64_t)v);
}

/* Worst case line: 20+1+10+1+10+1+11+1+11+1+11+1+20+1 = 100. */
static void session_log_event(uint64_t step, const toy_event_t *ev)
{
    if (toy_session_len + 100u > TOY_SESSION_CAP - TOY_SESSION_TAIL) {
        toy_session_dropped++;
        return;
    }
    session_u64(step);
    session_putc(' ');
    session_u64(ev->type);
    session_putc(' ');
    session_u64(ev->id);
    session_putc(' ');
    session_i64(ev->x);
    session_putc(' ');
    session_i64(ev->y);
    session_putc(' ');
    session_i64(ev->v);
    session_putc(' ');
    session_u64(ev->bits);
    session_putc('\n');
}

static void session_reset(const char *cart_name)
{
    toy_session_len = 0;
    toy_session_dropped = 0;
    session_str("TOYSESSION1 cart=");
    session_str(cart_name ? cart_name : "");
    session_putc('\n');
    toy_session_buf[toy_session_len] = '\0';
}

/* Signal seam (SIG-1, runner-owned, never in state): select
 * instantiates the descriptor's synthetic proof spec exactly
 * as emu/board (toy_sig_proof_source_init is the one creation
 * point a future live source would swap), and toy_wasm_step
 * feeds exactly samples_per_step frames FIRST at each
 * sub-step, then applies events, then steps - feed -> apply
 * -> step. The Node checker is unchanged and its step(1)
 * proof loop therefore covers signal carts automatically;
 * signal-deaf carts never touch any of this. A synthetic
 * source is unbounded, so a feed failure here is a logic
 * error: the batch stops without stepping (the hash stalls
 * and the checker fails) rather than inventing samples. */
static toy_sig_source_t toy_sig_src;
static int16_t toy_sig_frame[TOY_SIG_MAX_FRAME_VALUES];
static int toy_sig_active;

int toy_wasm_count(void) { return toy_cart_count; }

static const toy_cart_t *cart_at(int index)
{
    if (index < 0 || index >= toy_cart_count) return 0;
    return toy_carts[index];
}

const char *toy_wasm_name(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->name : "";
}

const char *toy_wasm_title(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->title : "";
}

const char *toy_wasm_blurb(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->blurb : "";
}

uint32_t toy_wasm_demo_steps_per_frame(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->demo_steps_per_frame : 0;
}

uint32_t toy_wasm_proof_steps(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->proof_steps : 0;
}

uint32_t toy_wasm_proof_report_every(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->proof_report_every : 0;
}

int toy_wasm_select(int index)
{
    const toy_cart_t *c = cart_at(index);
    if (!c || c->state_size > sizeof toy_state) return 0;
    if (toy_cart_check_input(c)) return 0;
    if (toy_cart_check_signal(c)) return 0;
    toy_sig_active = toy_sig_proof_source_init(c, &toy_sig_src);
    if (toy_sig_active < 0) { toy_sig_active = 0; return 0; }
    toy_cart = c;
    toy_memset(toy_state, 0, sizeof toy_state);
    toy_cart->reset(toy_state);
    toy_steps_done = 0;
    toy_queue_clear(&toy_live_queue);
    toy_proof_cursor = 0;
    session_reset(c->name);
    toy_apply_proof_at(toy_cart, toy_state, 0, &toy_proof_cursor);
    return 1;
}

void toy_wasm_step(uint32_t count)
{
    if (!toy_cart) return;
    if (count > 100000u) count = 100000u;
    for (uint32_t i = 0; i < count; i++) {
        if (toy_feed_step(toy_cart, toy_state,
                          toy_sig_active ? &toy_sig_src : 0,
                          toy_sig_frame, TOY_SIG_MAX_FRAME_VALUES))
            return; /* never invent samples; hash stalls, checker fails */
        uint32_t before = toy_proof_cursor;
        toy_apply_proof_at(toy_cart, toy_state,
                           (uint32_t)(toy_steps_done + 1),
                           &toy_proof_cursor);
        /* Snapshot the queue head before the drain consumes it;
         * the return count says how many were actually applied
         * at this boundary (shared per-step cap, proof first),
         * and exactly those enter the session log under the
         * step they precede. Carried events log later, at the
         * boundary that really applies them. */
        uint32_t budget = TOY_MAX_EVENTS_PER_STEP - (toy_proof_cursor - before);
        uint32_t snap_n = toy_live_queue.len < budget ? toy_live_queue.len : budget;
        toy_event_t snap[TOY_MAX_EVENTS_PER_STEP];
        for (uint32_t k = 0; k < snap_n; k++) snap[k] = toy_live_queue.ev[k];
        uint32_t applied = toy_apply_queue(toy_cart, toy_state,
                                           &toy_live_queue, budget);
        for (uint32_t k = 0; k < applied; k++)
            session_log_event(toy_steps_done + 1, &snap[k]);
        toy_cart->step(toy_state);
        toy_steps_done++;
    }
}

int toy_wasm_render(int size)
{
    if (!toy_cart || size < 16 || size > TOY_WASM_MAX_FB) return 0;
    toy_fb.w = size;
    toy_fb.h = size;
    toy_fb.px = toy_pixels;
    toy_cart->render(toy_state, &toy_fb);
    return 1;
}

const uint16_t *toy_wasm_framebuffer(void) { return toy_pixels; }
int toy_wasm_framebuffer_width(void) { return toy_fb.w; }
int toy_wasm_framebuffer_height(void) { return toy_fb.h; }

uint32_t toy_wasm_hash_lo(void)
{
    if (!toy_cart) return 0;
    return (uint32_t)toy_hash_state(toy_state, toy_cart->state_size);
}

uint32_t toy_wasm_hash_hi(void)
{
    if (!toy_cart) return 0;
    return (uint32_t)(toy_hash_state(toy_state, toy_cart->state_size) >> 32);
}

uint32_t toy_wasm_steps_lo(void) { return (uint32_t)toy_steps_done; }
uint32_t toy_wasm_steps_hi(void) { return (uint32_t)(toy_steps_done >> 32); }
uint32_t toy_wasm_state_size(void) { return toy_cart ? (uint32_t)toy_cart->state_size : 0; }

/* --- input seam exports ---
 * Reachable from JS via TOY_EXPORT above (app.js still
 * feature-checks them, so an older cached wasm degrades to a
 * deaf page instead of throwing). Queue pushes return 0 when
 * cap-gated for the selected cart or dropped on a full queue,
 * 1 when queued/coalesced. Pointer pushes take raw integer
 * pixels + canvas size; milli-radius conversion happens here
 * in C (integer only), never in JS floats. */

TOY_EXPORT("toy_wasm_input_caps")
uint32_t toy_wasm_input_caps(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->input_caps : 0;
}

TOY_EXPORT("toy_wasm_param_count")
uint32_t toy_wasm_param_count(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->param_count : 0;
}

static const toy_param_t *param_at(int index, uint32_t p)
{
    const toy_cart_t *c = cart_at(index);
    if (!c || !c->params || p >= c->param_count) return 0;
    return &c->params[p];
}

TOY_EXPORT("toy_wasm_param_name")
const char *toy_wasm_param_name(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->name : "";
}

TOY_EXPORT("toy_wasm_param_label")
const char *toy_wasm_param_label(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->label : "";
}

TOY_EXPORT("toy_wasm_param_kind")
uint32_t toy_wasm_param_kind(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->kind : 0;
}

TOY_EXPORT("toy_wasm_param_flags")
uint32_t toy_wasm_param_flags(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->flags : 0;
}

/* which: 0=min 1=max 2=def */
TOY_EXPORT("toy_wasm_param_i")
int32_t toy_wasm_param_i(int index, uint32_t p, int which)
{
    const toy_param_t *t = param_at(index, p);
    if (!t) return 0;
    return which == 0 ? t->i_min : which == 1 ? t->i_max : t->i_def;
}

TOY_EXPORT("toy_wasm_param_f")
const char *toy_wasm_param_f(int index, uint32_t p, int which)
{
    const toy_param_t *t = param_at(index, p);
    if (!t) return "";
    const char *s = which == 0 ? t->f_min : which == 1 ? t->f_max : t->f_def;
    return s ? s : "";
}

TOY_EXPORT("toy_wasm_param_enum_count")
uint32_t toy_wasm_param_enum_count(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    if (!t || t->kind != TOY_PARAM_ENUM || t->i_max < 0) return 0;
    return (uint32_t)(t->i_max + 1);
}

TOY_EXPORT("toy_wasm_param_enum_name")
const char *toy_wasm_param_enum_name(int index, uint32_t p, uint32_t e)
{
    const toy_param_t *t = param_at(index, p);
    if (!t || !t->enum_names || e >= toy_wasm_param_enum_count(index, p))
        return "";
    return t->enum_names[e] ? t->enum_names[e] : "";
}

uint32_t toy_wasm_proof_input_count(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->proof_input_count : 0;
}

uint32_t toy_wasm_input_hash_lo(int index)
{
    const toy_cart_t *c = cart_at(index);
    if (!c || !c->proof_inputs) return 0;
    return (uint32_t)toy_hash_inputs(c->proof_inputs, c->proof_input_count);
}

uint32_t toy_wasm_input_hash_hi(int index)
{
    const toy_cart_t *c = cart_at(index);
    if (!c || !c->proof_inputs) return 0;
    return (uint32_t)(toy_hash_inputs(c->proof_inputs,
                                      c->proof_input_count) >> 32);
}

TOY_EXPORT("toy_wasm_pending")
uint32_t toy_wasm_pending(void) { return toy_live_queue.len; }
TOY_EXPORT("toy_wasm_dropped")
uint32_t toy_wasm_dropped(void) { return toy_live_queue.dropped; }

/* Generic push: fields as in toy_event_t (bits split lo/hi). */
TOY_EXPORT("toy_wasm_queue_event")
int toy_wasm_queue_event(uint32_t type, uint32_t id, int32_t x, int32_t y,
                         int32_t v, uint32_t bits_lo, uint32_t bits_hi)
{
    if (!toy_cart || !toy_cart->apply) return 0;
    if (type == TOY_EV_NONE || type > TOY_EV_MAX_KNOWN) return 0;
    uint32_t need = toy_event_cap_mask(type);
    if (need && (toy_cart->input_caps & need) == 0) return 0;
    if (type == TOY_EV_SET_PARAM && id >= toy_cart->param_count) return 0;
    toy_event_t ev = { 0 };
    ev.type = type;
    ev.id = id;
    ev.x = x;
    ev.y = y;
    ev.v = v;
    ev.bits = (uint64_t)bits_lo | ((uint64_t)bits_hi << 32);
    return toy_queue_push(&toy_live_queue, &ev);
}

TOY_EXPORT("toy_wasm_queue_pointer")
int toy_wasm_queue_pointer(uint32_t type, uint32_t id, int32_t px, int32_t py,
                           int32_t size_px)
{
    if (!toy_cart || !toy_cart->apply) return 0;
    if ((toy_cart->input_caps & TOY_CAP_POINTER) == 0) return 0;
    if (type != TOY_EV_POINTER_DOWN && type != TOY_EV_POINTER_MOVE &&
        type != TOY_EV_POINTER_UP) return 0;
    return toy_queue_push_pointer(&toy_live_queue, type, id, px, py, size_px);
}

int toy_wasm_queue_key(uint32_t type, uint32_t key, int32_t repeat)
{
    return toy_wasm_queue_event(type, key, 0, 0, repeat, 0, 0);
}

int toy_wasm_queue_encoder(uint32_t id, int32_t delta)
{
    return toy_wasm_queue_event(TOY_EV_ENCODER, id, 0, 0, delta, 0, 0);
}

TOY_EXPORT("toy_wasm_queue_tilt")
int toy_wasm_queue_tilt(int32_t mgx, int32_t mgy)
{
    return toy_wasm_queue_event(TOY_EV_TILT, 0, mgx, mgy, 0, 0, 0);
}

TOY_EXPORT("toy_wasm_queue_dial")
int toy_wasm_queue_dial(int32_t milliturn_delta)
{
    return toy_wasm_queue_event(TOY_EV_DIAL, 0, 0, 0, milliturn_delta, 0, 0);
}

int toy_wasm_queue_set_param_i(uint32_t p, int32_t value)
{
    return toy_wasm_queue_event(TOY_EV_SET_PARAM, p, 0, 0, value, 0, 0);
}

/* Mock-device settings push: one entry point for every param
 * kind. I32/ENUM carry the value in v (bits ignored, pass 0);
 * F64 carries the IEEE binary64 bit pattern split lo/hi (v
 * ignored, pass 0). The JS side derives F64 bits from the
 * exact decimal its control displays; the bit pattern is what
 * the session log records, so a replayed session reproduces
 * the live value bit-for-bit regardless of who rounded it.
 * Proofs never come through here - they build F64 values from
 * decimal strings through toy_param_event as always. Range
 * clamping stays the cart's job in apply(). */
TOY_EXPORT("toy_wasm_queue_param")
int toy_wasm_queue_param(uint32_t p, int32_t v,
                         uint32_t bits_lo, uint32_t bits_hi)
{
    return toy_wasm_queue_event(TOY_EV_SET_PARAM, p, 0, 0, v, bits_lo, bits_hi);
}

/* Session log export. Appends the export-time tail (TRUNCATED
 * if the cap was hit, then END with the step count as of this
 * call) after the recorded lines and returns the whole text.
 * The tail does not advance toy_session_len: later event lines
 * overwrite it, so repeated exports stay well-formed and END
 * always names the steps completed when it was written. Read
 * the string before the next step/select call. */
TOY_EXPORT("toy_wasm_session_log")
const char *toy_wasm_session_log(void)
{
    uint32_t at = toy_session_len;
    if (toy_session_dropped) {
        const char *t = "TRUNCATED dropped=";
        while (*t) toy_session_buf[at++] = *t++;
        uint32_t v = toy_session_dropped;
        char tmp[10];
        int n = 0;
        do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v);
        while (n > 0) toy_session_buf[at++] = tmp[--n];
        toy_session_buf[at++] = '\n';
    }
    const char *e = "END ";
    while (*e) toy_session_buf[at++] = *e++;
    uint64_t steps = toy_steps_done;
    char tmp[20];
    int n = 0;
    do { tmp[n++] = (char)('0' + (steps % 10u)); steps /= 10u; } while (steps);
    while (n > 0) toy_session_buf[at++] = tmp[--n];
    toy_session_buf[at++] = '\n';
    toy_session_buf[at] = '\0';
    return toy_session_buf;
}

/* --- time layer: JS-provided clock imports (glue only) ---
 * The browser runner owns wall time (docs/study-timebank.md
 * section 2.4): JS provides these env imports, the glue
 * assembles toy_time_t from them, and the toy_wasm_clock_*
 * exports let the page read the value back through C. Manual
 * set happens JS-side (the page changes what the imports
 * return; it starts unset), so toy_clock_set has no authority
 * here and returns false. Nothing clock-shaped is read by
 * toy_wasm_step: pacing stays JS stepDebt -> integer counts,
 * exactly as before, so pinned hashes cannot move. Banking UI
 * is a later parcel; this is plumbing only. */
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"

__attribute__((import_module("env"), import_name("toy_js_clock_valid")))
extern int32_t toy_js_clock_valid(void);
__attribute__((import_module("env"), import_name("toy_js_clock_tz_min")))
extern int32_t toy_js_clock_tz_min(void);
__attribute__((import_module("env"), import_name("toy_js_clock_utc_lo")))
extern uint32_t toy_js_clock_utc_lo(void);
__attribute__((import_module("env"), import_name("toy_js_clock_utc_hi")))
extern uint32_t toy_js_clock_utc_hi(void);
__attribute__((import_module("env"), import_name("toy_js_mono_lo")))
extern uint32_t toy_js_mono_lo(void);
__attribute__((import_module("env"), import_name("toy_js_mono_hi")))
extern uint32_t toy_js_mono_hi(void);

toy_time_t toy_clock_now(void)
{
    toy_time_t t;
    t.utc_seconds = (int64_t)((uint64_t)toy_js_clock_utc_lo() |
                              ((uint64_t)toy_js_clock_utc_hi() << 32));
    t.tz_offset_min = toy_js_clock_tz_min();
    t.valid = toy_js_clock_valid() != 0;
    return t;
}

uint64_t toy_clock_mono_ms(void)
{
    return (uint64_t)toy_js_mono_lo() | ((uint64_t)toy_js_mono_hi() << 32);
}

bool toy_clock_set(int64_t utc_seconds, int32_t tz_offset_min)
{
    (void)utc_seconds;
    (void)tz_offset_min;
    return false; /* JS owns the value; set it page-side */
}

void toy_clock_clear(void)
{
    /* JS owns the value; unset it page-side. */
}

int toy_wasm_clock_valid(void) { return toy_clock_now().valid ? 1 : 0; }
uint32_t toy_wasm_clock_utc_lo(void)
{
    return (uint32_t)(uint64_t)toy_clock_now().utc_seconds;
}
uint32_t toy_wasm_clock_utc_hi(void)
{
    return (uint32_t)(((uint64_t)toy_clock_now().utc_seconds) >> 32);
}
int32_t toy_wasm_clock_tz_min(void) { return toy_clock_now().tz_offset_min; }
uint32_t toy_wasm_clock_mono_lo(void) { return (uint32_t)toy_clock_mono_ms(); }
uint32_t toy_wasm_clock_mono_hi(void)
{
    return (uint32_t)(toy_clock_mono_ms() >> 32);
}
