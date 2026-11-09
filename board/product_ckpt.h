/* product_ckpt.h - pure checkpoint policy seam for the
 * product runner (toy5-flashprod): when a running cart
 * saves, when a LIVE feed run must not, and which flash
 * slot (if any) boot resumes.
 *
 * Everything storage-shaped already exists elsewhere and
 * is reused, never re-decided here: the TOYCKPT1 image
 * (core/toy_ckpt.h), the A/B slot geometry + newest/next
 * selection rule (core/toy_ckpt_slot.h, host-pinned by
 * tools/flashslot_test.c), and the flash backend over
 * flash_safe_execute (board/ckpt_flash.{c,h}). This header
 * owns only the product glue decisions:
 *
 *  - Cadence: while a cart is RUNNING, a checkpoint is due
 *    every TOY_PROD_CKPT_SECONDS of wall time (the runner
 *    also saves once on clean exit-to-gallery; that call
 *    is unconditional in main_product.c, not timer work).
 *    The timer re-arms from each save's completion, so the
 *    spacing is between completed saves and a long flash
 *    stall never schedules an immediate catch-up write.
 *    TOY_PROD_CKPT_SECONDS 0 disables cadence saves (the
 *    exit save still runs). Wear arithmetic for the
 *    default lives in board/PRODUCT.md - raise the
 *    constant there-and-here together, they cite each
 *    other.
 *  - Skip-live: a LIVE-source feed run is never
 *    checkpointed. Its sample position is not a function
 *    of a step count, and core refuses to reconstruct it
 *    (toy_sig_seek_samples fails for non-SYNTHETIC kinds,
 *    core/toy_signal.h); a saved live state would resume
 *    against the wrong stream position. Synthetic feed
 *    runs save like any other cart: the runner feeds
 *    exactly samples_per_step before every step, so the
 *    saved step_count respawns the source by seek (the
 *    toy4-feedckpt semantics, emu cmd_load shape).
 *  - Boot resume: a pure decision over per-slot scan views
 *    (header parsed? full validate passed? cart name?
 *    header step_count?) crossed with the live cart
 *    registry. The runner builds views from
 *    toy_ckpt_parse_header + toy_ckpt_slot_probe over the
 *    XIP slot pointers, then acts on the verdict. Among
 *    valid slots naming registered carts, the highest
 *    step_count wins with ties to the lowest slot - the
 *    same rule toy_ckpt_slot_newest defines, re-expressed
 *    over views so the host test can pin the registry
 *    cross-product (slot choice itself stays pinned by
 *    flashslot_test.c; this layer adds the registry).
 *    A valid image for an UNregistered cart never resumes
 *    (firmware shrank/renamed carts: gallery, loudly).
 *    Mixed-cart slots follow the same raw highest-step
 *    rule - step_count is per-run and not comparable
 *    across carts (the toy_ckpt_slot.h caveat), so the
 *    product prevents the mixture instead of trusting the
 *    comparison: every fresh (non-resume) cart start
 *    erases both slots in main_product.c, leaving at most
 *    one run's images to ever be scanned. A mixed scan
 *    therefore means foreign or older firmware wrote last.
 *
 * Host-compilable and SDK-free like product_input.h /
 * live_feed.h; no link dependency beyond what the runner
 * already has (the decision reads descriptor names only).
 * tools/prod_ckpt_test.c drives every function here.
 */
#ifndef TOY_PRODUCT_CKPT_H
#define TOY_PRODUCT_CKPT_H

#include <stdint.h>

#include "live_feed.h" /* TOY_LIVE_FEED_* source modes */
#include "toy_cart.h"  /* toy_cart_t descriptors (names) */

/* --- cadence ---------------------------------------------------- */

#ifndef TOY_PROD_CKPT_SECONDS
#define TOY_PROD_CKPT_SECONDS 300u /* owner wear decision 2026-11-07: 5 min */
#endif
#define TOY_PROD_CKPT_INTERVAL_MS \
    ((uint64_t)TOY_PROD_CKPT_SECONDS * 1000ull)

typedef struct {
    uint64_t last_ms; /* arm / last completed save time */
    int armed;        /* a cart is running under cadence */
} toy_prod_ckpt_timer_t;

static inline void toy_prod_ckpt_timer_init(toy_prod_ckpt_timer_t *t)
{
    t->last_ms = 0;
    t->armed = 0;
}

static inline void toy_prod_ckpt_arm(toy_prod_ckpt_timer_t *t,
                                     uint64_t now_ms)
{
    t->last_ms = now_ms;
    t->armed = 1;
}

/* A save attempt (success or failure) spaces the next one
 * a full interval out - same state transition as arm(),
 * named for the call site. */
static inline void toy_prod_ckpt_note(toy_prod_ckpt_timer_t *t,
                                      uint64_t now_ms)
{
    toy_prod_ckpt_arm(t, now_ms);
}

static inline void toy_prod_ckpt_disarm(toy_prod_ckpt_timer_t *t)
{
    t->armed = 0;
}

/* Due iff armed, the interval is nonzero, and a full
 * interval elapsed since last_ms. A backwards clock reads
 * as not due (mono time should never go backwards; if a
 * caller's clock does, deferring a save is the safe side). */
static inline int toy_prod_ckpt_due(const toy_prod_ckpt_timer_t *t,
                                    uint64_t now_ms, uint64_t interval_ms)
{
    if (!t->armed || interval_ms == 0u || now_ms < t->last_ms)
        return 0;
    return now_ms - t->last_ms >= interval_ms;
}

/* --- save / skip-live decision ---------------------------------- */

#define TOY_PROD_CKPT_SAVE      0
#define TOY_PROD_CKPT_SKIP_LIVE 1

/* feed_mode is a live_feed.h TOY_LIVE_FEED_* value chosen
 * at cart start. Only LIVE skips: signal-deaf carts (NONE)
 * and synthetic feed carts (seek-restorable) both save. */
static inline int toy_prod_ckpt_save_decision(int feed_mode)
{
    return feed_mode == TOY_LIVE_FEED_LIVE ? TOY_PROD_CKPT_SKIP_LIVE
                                           : TOY_PROD_CKPT_SAVE;
}

/* --- boot-resume decision --------------------------------------- */

/* One slot as the boot scan sees it. header_ok means
 * toy_ckpt_parse_header passed (magic/version/header CRC/
 * name shape), so cart_name + step_count are the header's
 * claims; valid means toy_ckpt_slot_probe passed in full
 * (both CRCs + state hash + registry checks), so the
 * claims are proven. step_count is only ever COMPARED for
 * valid slots - a torn slot's header field is, at best, a
 * half-programmed claim and never outranks validity. */
typedef struct {
    int header_ok;
    int valid;
    char cart_name[16]; /* NUL-terminated when header_ok */
    uint64_t step_count;
} toy_prod_ckpt_slot_view_t;

enum {
    TOY_PROD_RESUME_NONE = 0,        /* no slot holds a header */
    TOY_PROD_RESUME_CART,            /* resume slot/cart_index */
    TOY_PROD_RESUME_UNKNOWN_CART,    /* named cart not registered */
    TOY_PROD_RESUME_NO_VALID         /* headers, none fully valid */
};

typedef struct {
    int action;     /* TOY_PROD_RESUME_* */
    int slot;       /* resume/unknown slot index, else -1 */
    int cart_index; /* registry index when RESUME_CART, else -1 */
} toy_prod_ckpt_resume_t;

static inline int toy_prod_ckpt_name_eq(const char *a, const char *b)
{
    if (!a || !b)
        return 0;
    while (*a && *b && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

/* Registry index of `name`, or -1. The registry is the
 * runner's own view (toy_carts in the product), so a cart
 * core knows but this runner does not list still counts
 * as unknown here - it cannot be launched either way. */
static inline int toy_prod_ckpt_find_cart(const char *name,
                                          const toy_cart_t *const *carts,
                                          int32_t cart_count)
{
    if (!name || !carts)
        return -1;
    for (int32_t i = 0; i < cart_count; i++)
        if (carts[i] && toy_prod_ckpt_name_eq(name, carts[i]->name))
            return (int)i;
    return -1;
}

/* The boot decision, stated once:
 *  1. Any slot both valid AND registered is resumable;
 *     resume the highest step_count (ties: lowest slot).
 *  2. Else, any header naming an unregistered cart is
 *     UNKNOWN_CART (first such slot reported) - the image
 *     may be perfectly intact, this firmware just cannot
 *     run it.
 *  3. Else, any surviving header at all is NO_VALID -
 *     something was written, nothing survived validation
 *     (torn newest with no older slot, corrupt payload).
 *  4. Else NONE: blank device, gallery quietly.
 * *out is always written when given. */
static inline void toy_prod_ckpt_resume_decide(
    const toy_prod_ckpt_slot_view_t *slots, uint32_t slot_count,
    const toy_cart_t *const *carts, int32_t cart_count,
    toy_prod_ckpt_resume_t *out)
{
    toy_prod_ckpt_resume_t sink;
    if (!out)
        out = &sink;
    out->action = TOY_PROD_RESUME_NONE;
    out->slot = -1;
    out->cart_index = -1;
    if (!slots)
        return;

    int best = -1, best_cart = -1;
    int first_unknown = -1, any_header = 0;
    for (uint32_t i = 0; i < slot_count; i++) {
        if (!slots[i].header_ok)
            continue;
        any_header = 1;
        int ci = toy_prod_ckpt_find_cart(slots[i].cart_name, carts,
                                         cart_count);
        if (ci < 0) {
            if (first_unknown < 0)
                first_unknown = (int)i;
            continue;
        }
        if (!slots[i].valid)
            continue;
        if (best < 0 ||
            slots[i].step_count > slots[best].step_count) {
            best = (int)i;
            best_cart = ci;
        }
    }
    if (best >= 0) {
        out->action = TOY_PROD_RESUME_CART;
        out->slot = best;
        out->cart_index = best_cart;
    } else if (first_unknown >= 0) {
        out->action = TOY_PROD_RESUME_UNKNOWN_CART;
        out->slot = first_unknown;
    } else if (any_header) {
        out->action = TOY_PROD_RESUME_NO_VALID;
    }
}

#endif
