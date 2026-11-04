/* toy_cart.h - the cartridge interface.
 *
 * A cartridge owns a fixed-size state made only of f64 bit
 * patterns and integers (no pointers, no heap), zero-initialised
 * before reset() so state bytes are hashable and checkpointable
 * verbatim. step() advances exactly one fixed internal dt.
 * render() draws from state; it may convert to native double for
 * display placement only and must never write state.
 */
#ifndef TOY_CART_H
#define TOY_CART_H

#include <stddef.h>
#include <stdint.h>
#include "toy_fb.h"
#include "toy_input.h"
#include "toy_signal.h"

/* Every host (emulator, board, WASM) allocates this much state.
 * A cartridge whose state exceeds it does not load anywhere. */
#define TOY_MAX_STATE_SIZE 65536u

/* toy_cart_t is typedef'd in toy_input.h (struct toy_cart_s). */
struct toy_cart_s {
    const char *name;   /* short id, e.g. "fig8" */
    const char *title;  /* display title */
    const char *blurb;  /* one line: what to notice */
    size_t state_size;
    /* Live-demo pacing and the standard cross-platform proof run.
     * Keeping these in the descriptor makes a new cartridge enter
     * every runner without editing emulator, board or site code. */
    uint32_t demo_steps_per_frame;
    uint32_t proof_steps;
    uint32_t proof_report_every;
    void (*reset)(void *st);
    void (*step)(void *st);
    void (*render)(const void *st, toy_fb_t *fb);
    /* --- input & settings seam, APPENDED (study-input 5.3) ---
     * All zero/NULL for deaf carts. Existing positional
     * initialisers (ten fields) zero-fill this tail, so deaf
     * carts keep their source and proof hashes unchanged.
     * apply() obeys the step() rules (softfloat only, no alloc,
     * no I/O, bounded) and runs only at step boundaries. */
    void (*apply)(void *st, const toy_event_t *ev); /* or NULL */
    const toy_param_t *params;          /* or NULL */
    uint32_t param_count;               /* <= TOY_MAX_PARAMS */
    const toy_proof_input_t *proof_inputs; /* or NULL */
    uint32_t proof_input_count;         /* <= TOY_MAX_PROOF_INPUTS */
    uint32_t input_caps;                /* TOY_CAP_* mask */
    /* Checkpoint payload-layout version (core/toy_ckpt.h,
     * docs/study-checkpoint.md section 4). Bump on ANY state-struct
     * or step-semantics change; loaders reject mismatches rather
     * than guess. Appended after the input fields (WAVE-T2 tail
     * order) so existing positional initialisers zero-fill it: 0
     * is every current cartridge's layout version. Referenced by
     * name only, never by offset. */
    uint32_t state_version;
    /* --- signal feed seam, APPENDED after state_version (SIG-1,
     * docs/study-signals.md section 4) ---
     * All zero/NULL for signal-deaf carts; existing positional
     * initialisers zero-fill this tail, so deaf carts keep their
     * source and proof hashes unchanged. feed() obeys the
     * step()/apply() rules (integer/softfloat only, no alloc, no
     * I/O, bounded, writes only its own state) and runs only at
     * step boundaries: each step, the runner polls exactly
     * sig_spec.samples_per_step frames and calls feed() FIRST,
     * then applies that boundary's input events, then step() -
     * feed delivers the samples the step consumes, apply() acts
     * on that fresh sensor state (including dial events a runner
     * derived from these very frames). There is no step-0 feed;
     * the first feed precedes step 1. feed() copies values (and,
     * via toy_sig_label_feed, stream kind + stream_id) into POD
     * state; step() still reads no input. flags carries
     * TOY_SIG_FLAG_* from the poll: proof runners fail loud on
     * underrun and never call feed() with flags set. */
    void (*feed)(void *st, const int16_t *samples, uint32_t n_samples,
                 uint32_t channels, const toy_sig_info_t *info,
                 uint32_t flags); /* or NULL */
    toy_sig_spec_t sig_spec;      /* all zero = no signal need */
};

extern const toy_cart_t toy_cart_fig8;
extern const toy_cart_t toy_cart_pendulum;
extern const toy_cart_t toy_cart_carousel;
extern const toy_cart_t toy_cart_stagnation;
extern const toy_cart_t toy_cart_microscope;
extern const toy_cart_t toy_cart_harmonograph;
extern const toy_cart_t toy_cart_lorenz;
extern const toy_cart_t toy_cart_intervallens;
extern const toy_cart_t toy_cart_pendwave;
extern const toy_cart_t toy_cart_orrery;
extern const toy_cart_t toy_cart_cloud;
extern const toy_cart_t toy_cart_mandelbrot;
extern const toy_cart_t toy_cart_orbitspec;
extern const toy_cart_t toy_cart_julia;
extern const toy_cart_t toy_cart_gravity;
extern const toy_cart_t toy_cart_gpend;
extern const toy_cart_t toy_cart_seismo;
extern const toy_cart_t toy_cart_attitude;
extern const toy_cart_t toy_cart_synspec;

extern const toy_cart_t *const toy_carts[];
extern const int toy_cart_count;

const toy_cart_t *toy_cart_by_name(const char *name);

/* FNV-1a 64 over raw state bytes. The toy's proof hook:
 * identical on host, RP2040 and RP2350 for identical step counts. */
uint64_t toy_hash_state(const void *st, size_t n);

/* --- shared apply helpers (study-input 5.4) ---
 * One implementation for every runner, so "same inputs" means
 * the same thing everywhere. Runners keep owning their step
 * loops; these helpers only apply events, never call step().
 * Header-inline so the board proof firmware (explicit source
 * list) links them with no new build line.
 *
 * Per-step cap and drain fate: at most TOY_MAX_EVENTS_PER_STEP
 * events are applied per call (one step boundary). Anything
 * beyond the cap CARRIES to the next step boundary in order:
 * proof entries wait behind the cursor, live events wait in
 * the queue/array tail. Nothing is dropped or reordered here.
 * Hashes/checkpoints are taken only after step(), never between
 * apply and step. */

/* Apply proof-script entries due at step s (s==0: post-reset
 * setup, before step 1). cursor is runner-owned scratch (index
 * into proof_inputs, start at 0). Applies entries with
 * entry.step <= s in script order, at most the per-step cap;
 * excess carries to the next call. No-op for scriptless/deaf
 * carts - the zero-churn fast path. Applied count for this call
 * is (*cursor_after - *cursor_before). */
static inline void toy_apply_proof_at(const toy_cart_t *cart, void *st,
                                      uint32_t step, uint32_t *cursor)
{
    if (!cart->apply || !cart->proof_inputs) return;
    uint32_t applied = 0;
    while (*cursor < cart->proof_input_count &&
           cart->proof_inputs[*cursor].step <= step &&
           applied < TOY_MAX_EVENTS_PER_STEP) {
        cart->apply(st, &cart->proof_inputs[*cursor].ev);
        (*cursor)++;
        applied++;
    }
}

/* Apply live events from an array, then NOT step. Caps applied
 * events at TOY_MAX_EVENTS_PER_STEP and returns how many array
 * entries were consumed; the caller carries evs[consumed..n)
 * to the next step boundary. A POINTER_MOVE immediately
 * followed by another MOVE for the same contact is coalesced
 * (skipped, still consumed); queue users get full in-place
 * coalescing at enqueue (toy_queue_push) and apply via
 * toy_apply_queue instead. Proof scripts never pass here. */
static inline uint32_t toy_apply_live(const toy_cart_t *cart, void *st,
                                      const toy_event_t *evs, uint32_t n)
{
    if (!cart->apply) return n; /* deaf: consumed, no effect */
    uint32_t consumed = 0, applied = 0;
    while (consumed < n && applied < TOY_MAX_EVENTS_PER_STEP) {
        const toy_event_t *ev = &evs[consumed];
        if (ev->type == TOY_EV_POINTER_MOVE && consumed + 1 < n &&
            evs[consumed + 1].type == TOY_EV_POINTER_MOVE &&
            evs[consumed + 1].id == ev->id) {
            consumed++; /* superseded by the next move */
            continue;
        }
        cart->apply(st, ev);
        consumed++;
        applied++;
    }
    return consumed;
}

/* Drain a live queue at one step boundary: apply at most
 * max_apply (itself capped at TOY_MAX_EVENTS_PER_STEP) events
 * from the head in FIFO order; the remainder CARRY in the queue
 * for the next boundary. Returns the number applied. The queue
 * was coalesced at enqueue, so no coalescing happens here. */
static inline uint32_t toy_apply_queue(const toy_cart_t *cart, void *st,
                                       toy_queue_t *q, uint32_t max_apply)
{
    if (max_apply > TOY_MAX_EVENTS_PER_STEP)
        max_apply = TOY_MAX_EVENTS_PER_STEP;
    if (q->len > TOY_INPUT_QUEUE_LEN) q->len = TOY_INPUT_QUEUE_LEN;
    uint32_t n = q->len < max_apply ? q->len : max_apply;
    if (cart->apply) {
        for (uint32_t i = 0; i < n; i++) cart->apply(st, &q->ev[i]);
    }
    uint32_t rem = q->len - n;
    for (uint32_t i = 0; i < rem; i++) q->ev[i] = q->ev[i + n];
    q->len = rem;
    return n;
}

/* Returns 0 if the descriptor's input fields are consistent
 * (study Appendix B, structural/integer checks), else a small
 * nonzero code: 1 param table shape, 2 proof script shape or
 * missing apply, 3 caps coherence, 4 param entries, 5 proof
 * entries. F64 bound strings are checked non-NULL only; their
 * numeric order is enforced by the cart's clamping in apply().
 * Runners call once at select/start. */
static inline int toy_cart_check_input(const toy_cart_t *cart)
{
    if (cart->param_count > TOY_MAX_PARAMS) return 1;
    if ((cart->param_count > 0) != (cart->params != 0)) return 1;
    if (cart->proof_input_count > TOY_MAX_PROOF_INPUTS) return 2;
    if ((cart->proof_input_count > 0) != (cart->proof_inputs != 0)) return 2;
    if ((cart->proof_inputs || cart->param_count > 0) && !cart->apply) return 2;
    if (!cart->apply && cart->input_caps != 0) return 3;
    if (((cart->input_caps & TOY_CAP_PARAMS) != 0) != (cart->param_count > 0))
        return 3;
    for (uint32_t i = 0; i < cart->param_count; i++) {
        const toy_param_t *p = &cart->params[i];
        if (!p->name || !p->name[0] || !p->label) return 4;
        for (uint32_t j = 0; j < i; j++) {
            const char *a = cart->params[j].name, *b = p->name;
            uint32_t k = 0;
            while (a[k] && a[k] == b[k]) k++;
            if (a[k] == b[k]) return 4; /* duplicate name */
        }
        if (p->kind == TOY_PARAM_I32) {
            if (!(p->i_min <= p->i_def && p->i_def <= p->i_max)) return 4;
        } else if (p->kind == TOY_PARAM_ENUM) {
            if (p->i_min != 0) return 4; /* v indexes enum_names */
            if (!(0 <= p->i_def && p->i_def <= p->i_max)) return 4;
            if (p->i_max < 0 || p->i_max > 64 || !p->enum_names) return 4;
            for (int32_t e = 0; e <= p->i_max; e++)
                if (!p->enum_names[e]) return 4;
        } else if (p->kind == TOY_PARAM_F64) {
            if (!p->f_min || !p->f_max || !p->f_def) return 4;
        } else {
            return 4;
        }
    }
    uint32_t prev_step = 0, at_step = 0;
    for (uint32_t i = 0; i < cart->proof_input_count; i++) {
        const toy_proof_input_t *in = &cart->proof_inputs[i];
        if (in->step < prev_step || in->step > cart->proof_steps) return 5;
        at_step = (i > 0 && in->step == prev_step) ? at_step + 1 : 1;
        if (at_step > TOY_MAX_EVENTS_PER_STEP) return 5;
        prev_step = in->step;
        if (in->ev.type > TOY_EV_MAX_KNOWN || in->ev.flags != 0) return 5;
        if (in->ev.type == TOY_EV_SET_PARAM && in->ev.id >= cart->param_count)
            return 5;
    }
    return 0;
}

/* --- shared signal feed helpers (study-signals section 4) ---
 * One implementation for every runner, the feed analogue of the
 * apply helpers above: runners keep owning their step loops and
 * call toy_feed_step() FIRST at each boundary, then the apply
 * helpers, then step(). Header-inline like them; the only new
 * link dependency is core/toy_signal.c (host/WASM glob it, the
 * board lists it deliberately). */

/* Returns 0 if the descriptor's signal fields are consistent,
 * else a small nonzero code: 1 spec without feed or feed
 * without spec, 2 bad proof kind/scenario (v1: SYNTHETIC,
 * scenario 1 only), 3 unknown device, 4 bad rate or
 * samples-per-step. Runners call once at select/start, beside
 * toy_cart_check_input. A signal-deaf cart passes trivially. */
static inline int toy_cart_check_signal(const toy_cart_t *cart)
{
    const toy_sig_spec_t *spec = &cart->sig_spec;
    int spec_set = spec->device != 0 || spec->proof_kind != 0 ||
                   spec->scenario_id != 0 || spec->sample_rate_hz != 0 ||
                   spec->samples_per_step != 0 || spec->seed != 0;
    if (!cart->feed) return spec_set ? 1 : 0;
    if (!spec_set) return 1;
    if (spec->proof_kind != TOY_SIG_KIND_SYNTHETIC) return 2;
    if (spec->scenario_id != 1u) return 2;
    if (toy_sig_device_channels(spec->device) == 0u) return 3;
    if (spec->sample_rate_hz == 0u) return 4;
    if (spec->samples_per_step == 0u ||
        spec->samples_per_step > TOY_SIG_MAX_SAMPLES_PER_STEP) return 4;
    return 0;
}

/* Instantiate the PROOF source for a cart from its descriptor
 * spec: the seeded synthetic stream named there, identical on
 * host, board and WASM. Returns 1 if a source was created, 0
 * for a signal-deaf cart (src untouched), -1 on a spec the
 * synthetic generators cannot satisfy exactly.
 * This is the single source-creation point a future live
 * runner swaps: it constructs its LIVE (or REPLAY) source
 * instead and the poll/feed loop below does not change. Only
 * proof paths call this version, and proofs are synthetic. */
static inline int toy_sig_proof_source_init(const toy_cart_t *cart,
                                            toy_sig_source_t *src)
{
    const toy_sig_spec_t *spec = &cart->sig_spec;
    if (!cart->feed) return 0;
    if (toy_cart_check_signal(cart)) return -1;
    if (spec->device == TOY_SIG_DEVICE_IMU6) {
        toy_sig_source_synth_imu(src, spec->seed, spec->sample_rate_hz);
    } else if (spec->device == TOY_SIG_DEVICE_MIC1) {
        /* Scenario 1 (v1): fixed 440 Hz tone + hiss. */
        toy_sig_source_synth_mic(src, spec->seed, spec->sample_rate_hz,
                                 440000u, 440000u, 0);
    } else {
        return -1;
    }
    if (src->info.kind != spec->proof_kind ||
        src->info.device != spec->device ||
        src->info.scenario_id != spec->scenario_id ||
        src->info.seed != spec->seed ||
        src->info.sample_rate_hz != spec->sample_rate_hz ||
        src->info.channels != toy_sig_device_channels(spec->device))
        return -1;
    return 1;
}

/* Feed one step's frames: poll exactly sig_spec.samples_per_step
 * samples into the runner-owned frame buffer (at most
 * TOY_SIG_MAX_FRAME_VALUES int16) and hand them to feed()
 * BEFORE apply()/step() at this boundary. Samples are
 * index-derived, never clock-derived: the i-th sample of step
 * s is (s-1)*samples_per_step + i by construction.
 * Returns 0 on success - including the signal-deaf no-op, the
 * zero-churn fast path - else nonzero and feed() was NOT
 * called: 1 source missing or mismatched with the spec, 2
 * underrun/end (study rule 4: a proof run fails loud, samples
 * are never zero-filled or repeated into a proof), 3 frame
 * buffer too small. Proof runners abort the run on nonzero; a
 * future live runner feeds gaps itself and does not use this
 * proof-strict helper for them. */
static inline int toy_feed_step(const toy_cart_t *cart, void *st,
                                toy_sig_source_t *src,
                                int16_t *frame, uint32_t frame_cap_values)
{
    const toy_sig_spec_t *spec = &cart->sig_spec;
    uint32_t channels, flags = 0, got;
    if (!cart->feed) return 0;
    if (!src || !frame) return 1;
    channels = toy_sig_device_channels(spec->device);
    if (channels == 0u || src->info.device != spec->device ||
        src->info.channels != channels ||
        src->info.sample_rate_hz != spec->sample_rate_hz) return 1;
    if ((uint64_t)spec->samples_per_step * channels >
        (uint64_t)frame_cap_values) return 3;
    got = toy_sig_poll(src, 0, 0, frame, spec->samples_per_step, &flags);
    if (got != spec->samples_per_step || flags != 0u) return 2;
    cart->feed(st, frame, got, channels, &src->info, flags);
    return 0;
}

#endif

