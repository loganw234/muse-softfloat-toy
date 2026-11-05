/* toy_shell.h - the product runner shell (presentation layer).
 *
 * PLACEMENT RULE (mirrors toy_clock.h): the shell is runner-side
 * code. It owns which cart is showing, the gallery picker, the
 * per-cart settings screen, the clock-set screen and the idle
 * clock screensaver - the screens the web mock (site/app.js)
 * rehearses in JS today. It is never inside a cartridge: a
 * cartridge still receives input only through apply()/feed() at
 * step boundaries, and nothing in here touches cartridge state.
 *
 * Enforced by construction, like the clock: including this
 * header without first defining TOY_SHELL_ALLOW is a compile
 * error, and because the shell renders runner wall time it also
 * requires TOY_CLOCK_ALLOW (it includes toy_clock.h for
 * toy_time_t / toy_schedule_t). Only runner code defines those
 * (emu/, board/, site glue, host tests); cartridge sources
 * define neither, so a cartridge that tried to drive the shell
 * - or read wall time through it - does not build. Header
 * location alone (core/) would not suffice as an argument:
 * toy_cart.h/toy_fb.h also live in core/ and ARE includable
 * from cartridges, so placement here proves nothing without
 * the guard.
 *
 * The shell never reads time itself: the runner passes a
 * toy_time_t INTO toy_shell_render() every frame. The shell
 * caches that runner-supplied value only to seed the clock-set
 * editor; it calls no clock function (toy_clock_now/mono/set
 * are never referenced from toy_shell.c - committing a
 * clock-set is the runner's toy_clock_set() call, signalled by
 * TOY_SH_ACT_CLOCK_COMMIT).
 *
 * Runner loop contract:
 *   1. Feed EVERY input event to toy_shell_event() FIRST.
 *      If it returns TOY_SH_ACT_FORWARD_TO_CART, queue the same
 *      event for the running cart (caps/coalescing as usual);
 *      otherwise the shell consumed it and the cart never sees
 *      it (this is how the exit gesture cannot leak into a
 *      cart's proof-visible input stream).
 *   2. Act on the returned action mask (below).
 *   3. Render: if toy_shell_screen() == TOY_SHELL_RUNNING the
 *      cart owns the glass (call cart->render, and step the
 *      cart); on every other screen call toy_shell_render()
 *      and do not step (settings/clock editing pauses the sim;
 *      the runner keeps the state bytes untouched).
 *   4. Call toy_shell_tick() once per presented frame for the
 *      idle screensaver; any event resets the idle count.
 */
#ifndef TOY_SHELL_ALLOW
#error "toy_shell.h is runner-side only: cartridges must never drive \
the product shell or read wall time through it. Define \
TOY_SHELL_ALLOW (and TOY_CLOCK_ALLOW) before including it from \
runner code (emu/, board/, site glue, host tests) only."
#endif
#ifndef TOY_SHELL_H
#define TOY_SHELL_H

#include <stddef.h>
#include <stdint.h>
#include "toy_cart.h"   /* toy_cart_t descriptors, params */
#include "toy_clock.h"  /* toy_time_t, toy_schedule_t (needs TOY_CLOCK_ALLOW) */
#include "toy_fb.h"
#include "toy_input.h"  /* toy_event_t, toy_queue_t */

enum {
    TOY_SHELL_GALLERY  = 0, /* cart list picker */
    TOY_SHELL_RUNNING  = 1, /* cart owns the glass; shell draws nothing */
    TOY_SHELL_SETTINGS = 2, /* param list for the current cart */
    TOY_SHELL_CLOCKSET = 3, /* hh:mm + UTC offset editor */
    TOY_SHELL_CLOCKFACE = 4 /* idle screensaver clock */
};

/* Action mask returned by toy_shell_event()/toy_shell_tick(). */
#define TOY_SH_ACT_NONE            0u
/* (Re)start toy_shell_current(): runner resets the cart, then
 * drains shell->param_queue (already filled by the shell, in
 * descriptor order, with every user-touched param - the site
 * mock's reset semantics) BEFORE the first step (the step-0
 * slot), then runs. Untouched params are never queued. */
#define TOY_SH_ACT_START_CART      (1u << 0)
/* The running cart was exited; runner stops stepping it. */
#define TOY_SH_ACT_EXIT_TO_GALLERY (1u << 1)
/* shell->param_queue holds SET_PARAM event(s) to drain into
 * the cart at the next step boundary (or at step 0 when
 * START_CART is also set). Drain with toy_apply_queue(). */
#define TOY_SH_ACT_PARAM_QUEUED    (1u << 2)
/* Clock-set committed: runner reads shell->clock_hh/mm/
 * tz_offset_min, combines them with its own date, and calls
 * toy_clock_set(). The shell never sets a clock itself. */
#define TOY_SH_ACT_CLOCK_COMMIT    (1u << 3)
/* The visible screen changed (redraw; start/stop stepping). */
#define TOY_SH_ACT_SCREEN          (1u << 4)
/* RUNNING and unconsumed: forward the original event to the
 * cart's live queue. Never set together with START/EXIT. */
#define TOY_SH_ACT_FORWARD_TO_CART (1u << 5)

/* Exit/settings gestures, using only existing event types.
 * Primary exit: TOY_EV_KEY_DOWN id=TOY_KEY_ESC. Touch-only
 * drivers synthesise that event from their own long-press
 * timing (toy_event_t has no timestamp, so the shell cannot
 * time a press, and a stolen rim zone would eat cart pointer
 * input). Button equivalents for cased hardware: */
#define TOY_SHELL_BUTTON_EXIT     0u /* BUTTON_DOWN id: exit/back */
#define TOY_SHELL_BUTTON_SETTINGS 1u /* BUTTON_DOWN id: settings */
/* Settings gesture: TOY_EV_KEY_DOWN id=TOY_KEY_X. */

#define TOY_SHELL_VISIBLE_ROWS 6u   /* gallery/settings rows on glass */
#define TOY_SHELL_IDLE_FRAMES_DEFAULT 1800u /* 60 s at 30 fps */
#define TOY_SHELL_F64_STEPS 1000    /* F64 params edit per-mille of range */

typedef struct {
    int32_t  i_val;   /* I32/ENUM value; F64: per-mille 0..1000 */
    uint64_t f_bits;  /* F64: IEEE bits for i_val (exact f_def until edited) */
    uint8_t  touched; /* user changed it at least once */
    uint8_t  staged;  /* applies on next (re)start (RESET_ONLY, or not running) */
} toy_shell_param_t;

typedef struct {
    const toy_cart_t *const *carts; /* runner-supplied registry view */
    int32_t cart_count;
    int32_t current;     /* cart settings/running refer to, or -1 */
    int32_t selected;    /* gallery row: 0..count+1 (last two are
                          * the SET CLOCK / CLOCK FACE virtual rows) */
    int32_t scroll;      /* first visible gallery row */
    uint8_t screen;      /* TOY_SHELL_* */
    uint8_t return_screen; /* where SETTINGS/CLOCKSET back goes */
    uint8_t running_valid; /* a cart was started and not exited */
    /* settings (for the current cart only) */
    toy_shell_param_t params[TOY_MAX_PARAMS];
    int32_t param_sel;
    int32_t param_scroll;
    toy_queue_t param_queue; /* SET_PARAM out-queue the runner drains */
    /* clock-set editor + last runner-supplied time */
    toy_time_t last_now; /* written only from render/begin args */
    int32_t clock_hh, clock_mm, clock_tz_offset_min;
    int32_t clock_field; /* 0 hh, 1 mm, 2 tz, 3 SET, 4 CANCEL */
    /* Runner-owned schedule copy: the runner allocates the
     * shell, so it owns this field and may overwrite it after
     * init (persisted settings); the shell only defaults it
     * and never edits it (schedule windows are not edited on
     * the clock-set screen in v1). */
    toy_schedule_t schedule;
    /* idle screensaver */
    uint32_t idle_frames;
    uint32_t idle_limit_frames; /* 0 disables; GALLERY only */
} toy_shell_t;

/* Initialise over a cart registry (toy_carts/toy_cart_count, or
 * any runner-filtered view of it). No allocation; the shell is
 * plain runner-side state and must never be hashed into or
 * checkpointed with cartridge state. Starts in GALLERY. */
void toy_shell_init(toy_shell_t *sh, const toy_cart_t *const *carts,
                    int32_t count);

/* Feed one event. Returns the TOY_SH_ACT_* mask. */
uint32_t toy_shell_event(toy_shell_t *sh, const toy_event_t *ev);

/* One presented frame with no event: idle bookkeeping.
 * GALLERY idles into CLOCKFACE at idle_limit_frames (returns
 * TOY_SH_ACT_SCREEN then); RUNNING never auto-hides the sim. */
uint32_t toy_shell_tick(toy_shell_t *sh);

/* Draw the current screen. RUNNING draws nothing (the cart
 * owns the glass). `now` is the runner's wall time for THIS
 * frame (toy_clock_now() is the runner's call, never the
 * shell's); it is also cached to seed the clock-set editor. */
void toy_shell_render(toy_shell_t *sh, toy_fb_t *fb, toy_time_t now);

const toy_cart_t *toy_shell_current(const toy_shell_t *sh); /* or NULL */
static inline uint8_t toy_shell_screen(const toy_shell_t *sh) { return sh->screen; }

/* Programmatic navigation for runners (e.g. "show the clock on
 * face-down wake"). Entering RUNNING this way does NOT emit
 * START_CART - the runner that asks for it owns the start. */
void toy_shell_enter(toy_shell_t *sh, uint8_t screen);

/* Seed the clock-set editor from a runner time and show it. */
void toy_shell_clockset_begin(toy_shell_t *sh, toy_time_t now);

/* Refill param_queue with every touched param of the current
 * cart in descriptor order (clearing staged flags). The shell
 * calls this itself whenever it emits START_CART; exposed for
 * runners that restart a cart for their own reasons (the mock
 * site's Reset button) so user values survive the reset,
 * exactly like site/app.js re-queues them in param order.
 * Returns the number queued. */
uint32_t toy_shell_queue_start_params(toy_shell_t *sh);

#endif
