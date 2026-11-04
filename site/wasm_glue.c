/* wasm_glue.c - browser-facing instance of the sim core.
 *
 * One selected cartridge, one state allocation, one framebuffer.
 * The same state can be rendered at multiple sizes without stepping
 * it, which lets the page show both target glasses at once.
 */
#include <stddef.h>
#include <stdint.h>
#include "toy_util.h"
#include "toy_cart.h"

#define TOY_WASM_MAX_FB 360

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
        toy_apply_queue(toy_cart, toy_state, &toy_live_queue,
                        TOY_MAX_EVENTS_PER_STEP - (toy_proof_cursor - before));
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
 * NOTE for the site build: these must be added to EXPORTS in
 * site/build.py to be reachable from JS (app.js feature-checks
 * them, so the page still runs until that lands). Queue pushes
 * return 0 when cap-gated for the selected cart or dropped on
 * a full queue, 1 when queued/coalesced. Pointer pushes take
 * raw integer pixels + canvas size; milli-radius conversion
 * happens here in C (integer only), never in JS floats. */

uint32_t toy_wasm_input_caps(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->input_caps : 0;
}

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

const char *toy_wasm_param_name(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->name : "";
}

const char *toy_wasm_param_label(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->label : "";
}

uint32_t toy_wasm_param_kind(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->kind : 0;
}

uint32_t toy_wasm_param_flags(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    return t ? t->flags : 0;
}

/* which: 0=min 1=max 2=def */
int32_t toy_wasm_param_i(int index, uint32_t p, int which)
{
    const toy_param_t *t = param_at(index, p);
    if (!t) return 0;
    return which == 0 ? t->i_min : which == 1 ? t->i_max : t->i_def;
}

const char *toy_wasm_param_f(int index, uint32_t p, int which)
{
    const toy_param_t *t = param_at(index, p);
    if (!t) return "";
    const char *s = which == 0 ? t->f_min : which == 1 ? t->f_max : t->f_def;
    return s ? s : "";
}

uint32_t toy_wasm_param_enum_count(int index, uint32_t p)
{
    const toy_param_t *t = param_at(index, p);
    if (!t || t->kind != TOY_PARAM_ENUM || t->i_max < 0) return 0;
    return (uint32_t)(t->i_max + 1);
}

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

uint32_t toy_wasm_pending(void) { return toy_live_queue.len; }
uint32_t toy_wasm_dropped(void) { return toy_live_queue.dropped; }

/* Generic push: fields as in toy_event_t (bits split lo/hi). */
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

int toy_wasm_queue_tilt(int32_t mgx, int32_t mgy)
{
    return toy_wasm_queue_event(TOY_EV_TILT, 0, mgx, mgy, 0, 0, 0);
}

int toy_wasm_queue_dial(int32_t milliturn_delta)
{
    return toy_wasm_queue_event(TOY_EV_DIAL, 0, 0, 0, milliturn_delta, 0, 0);
}

int toy_wasm_queue_set_param_i(uint32_t p, int32_t value)
{
    return toy_wasm_queue_event(TOY_EV_SET_PARAM, p, 0, 0, value, 0, 0);
}
