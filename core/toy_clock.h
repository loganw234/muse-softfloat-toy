/* toy_clock.h - the runner-owned wall clock (time layer v1).
 *
 * PLACEMENT RULE (docs/study-timebank.md sections 2.1-2.3, owner
 * decisions in docs/WAVE-T2.md): wall time belongs to the runner
 * (layer 2), never to the sim core (layer 1). Cartridges must
 * not read a clock, and nothing time-shaped - no timestamp, no
 * elapsed-ms accumulator, no warp factor, no schedule enum - may
 * ever enter cartridge state. State bytes are the toy's
 * identity: hashed verbatim (toy_hash_state), pinned across
 * host/board/WASM, and checkpointed as-is. If state were a
 * function of *when* steps ran, identical step counts would hash
 * differently in different timezones or after a clock-set, and
 * the host == board == WASM gate would be unprovable in
 * principle. The one time-like quantity allowed in state is a
 * step count, because it is a pure function of the step
 * sequence itself.
 *
 * This is enforced by construction, not by convention:
 *
 *  - Including this header without first defining
 *    TOY_CLOCK_ALLOW is a compile error. Only runner-side code
 *    (emu/, board/, site/ glue, host tests) defines it;
 *    cartridge sources never do, so a cartridge that tries to
 *    include this header does not build.
 *  - The header deliberately does NOT include toy_cart.h, and
 *    no function here takes or returns a cartridge state
 *    pointer. The clock API offers core code no path from a
 *    time reading into state even if it were included.
 *  - The implementations are per-runner, never in core/:
 *    emu/clock_emu.c (host), board/clock_board.c (unset stub
 *    until the RP2350 RTC kit is in hand), site/wasm_glue.c
 *    forwards to JS-provided imports. The time layer's entire
 *    output to the sim is non-negative integer step counts plus
 *    run/hold scheduling decisions made by the runner.
 *
 * Two time concepts, kept apart (study section 2.2): monotonic
 * milliseconds for pacing arithmetic ONLY (so clock-setting can
 * never inject or swallow sim steps), and civil wall time for
 * scheduling and report labels ONLY. Civil time is manually set
 * (time + zone offset by the user; no network time). Unset is a
 * first-class state: it disables schedule features (the
 * schedule helpers below degrade to always-active) but never
 * blocks simulation. The device is always plugged in, so no
 * sleep/power policy hangs off this API in v1.
 *
 * Warp is parameterised (warp_x100, 100 = 1.00x). v1 defaults:
 * live warp 1.00x; playback warp 16.00x (an 8 h bank replays in
 * ~30 min, study section 4.3); keyframe interval 60 s of demo
 * time (study section 3.3). These are layer-2 settings values;
 * only the step counts they produce ever reach the sim.
 */
#ifndef TOY_CLOCK_ALLOW
#error "toy_clock.h is runner-side only (docs/study-timebank.md s2.1): \
cartridges and other core code must never read wall time. Define \
TOY_CLOCK_ALLOW before including it from runner code (emu/, board/, \
site glue, host tests) only."
#endif
#ifndef TOY_CLOCK_H
#define TOY_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

/* Civil wall-time snapshot. utc_seconds is seconds since the
 * Unix epoch (UTC) as an int64; tz_offset_min is the manually
 * set local offset from UTC in minutes (e.g. +330 for UTC+5:30).
 * valid == false means unset/never set/lost: every other field
 * is then meaningless and must not be displayed as a time. */
typedef struct {
    int64_t utc_seconds;
    int32_t tz_offset_min;
    bool    valid;
} toy_time_t;

/* --- runner-implemented (one implementation per runner) --- */

/* Current civil time. On runners without a set clock this
 * returns valid == false forever (board stub until the kit). */
toy_time_t toy_clock_now(void);

/* Monotonic milliseconds since a runner-local epoch (boot).
 * Never runs backwards within a power cycle; the ONLY source
 * pacing arithmetic may use. Setting civil time never disturbs
 * it. */
uint64_t toy_clock_mono_ms(void);

/* Manually set civil time (touch entry, host env/CLI, JS UI).
 * Setting never touches the sim. Runners that cannot accept a
 * set (board stub, WASM glue where JS owns the value) return
 * false and stay as they were. */
bool toy_clock_set(int64_t utc_seconds, int32_t tz_offset_min);

/* Return to the unset state. No-op where the runner's time is
 * owned elsewhere (JS) or always unset (board stub). */
void toy_clock_clear(void);

/* --- pure helpers (integer only, runner-side use) --- */

#define TOY_NOMINAL_FPS              30u
#define TOY_LIVE_WARP_DEFAULT_X100   100u  /* 1.00x */
#define TOY_PLAYBACK_WARP_DEFAULT_X100 1600u /* 16x: 8 h in ~30 min */
#define TOY_KEYFRAME_INTERVAL_DEFAULT_S 60u

static inline int64_t toy_clock_local_seconds(toy_time_t t)
{
    return t.utc_seconds + (int64_t)t.tz_offset_min * 60;
}

/* Local minutes-of-day 0..1439 for schedule comparisons (naive
 * local civil time, study section 7.2: no tz/DST database on
 * device). Meaningless when !t.valid. */
static inline uint32_t toy_clock_minutes_of_day(toy_time_t t)
{
    int64_t s = toy_clock_local_seconds(t) % 86400;
    if (s < 0) s += 86400;
    return (uint32_t)(s / 60);
}

/* Layer-2 schedule/pacing settings (shape per study section
 * 5.1; storage/editing belongs to the settings seam - this is
 * the value the time layer consumes, with compiled defaults).
 * Active window is local minutes-of-day, [start, end); end <
 * start crosses midnight; start == end means always active.
 * keyframe_interval_s == 0 means start+final keyframes only. */
typedef struct {
    uint16_t active_start_min;
    uint16_t active_end_min;
    uint16_t live_warp_x100;
    uint16_t playback_warp_x100;
    uint16_t keyframe_interval_s;
    uint8_t  banking_enabled;
} toy_schedule_t;

static inline toy_schedule_t toy_schedule_default(void)
{
    toy_schedule_t s;
    s.active_start_min = 7u * 60u;   /* 07:00 */
    s.active_end_min = 23u * 60u;    /* 23:00 */
    s.live_warp_x100 = TOY_LIVE_WARP_DEFAULT_X100;
    s.playback_warp_x100 = TOY_PLAYBACK_WARP_DEFAULT_X100;
    s.keyframe_interval_s = TOY_KEYFRAME_INTERVAL_DEFAULT_S;
    s.banking_enabled = 1u;
    return s;
}

/* Is `now` inside the active window? Clock-unset degrades to
 * always-active (study section 7.1): an unset clock costs
 * convenience, never correctness, and never holds the sim. */
static inline bool toy_clock_is_active(toy_time_t now,
                                       const toy_schedule_t *sched)
{
    if (!now.valid) return true;
    if (sched->active_start_min == sched->active_end_min) return true;
    uint32_t m = toy_clock_minutes_of_day(now);
    if (sched->active_start_min < sched->active_end_min)
        return m >= sched->active_start_min && m < sched->active_end_min;
    return m >= sched->active_start_min || m < sched->active_end_min;
}

/* Steps owed for `elapsed_ms` of monotonic time at a
 * cartridge's demo pacing under a warp (study section 4.5,
 * generalised from site/app.js). Runner-side native arithmetic;
 * only the returned count reaches the sim. Callers clamp debt
 * per their own policy, as the site already does. */
static inline uint64_t toy_clock_steps_due(uint64_t elapsed_ms,
                                           uint32_t demo_steps_per_frame,
                                           uint16_t warp_x100)
{
    return elapsed_ms * (uint64_t)demo_steps_per_frame *
           (uint64_t)TOY_NOMINAL_FPS * (uint64_t)warp_x100 / (1000ull * 100ull);
}

/* Keyframe spacing in steps for `interval_s` of demo time
 * (study section 3.3: K = demo_steps_per_frame x 30 x
 * interval). 0 interval -> 0 (start+final only). */
static inline uint64_t toy_keyframe_every_steps(uint32_t demo_steps_per_frame,
                                                uint16_t interval_s)
{
    return (uint64_t)demo_steps_per_frame * (uint64_t)TOY_NOMINAL_FPS *
           (uint64_t)interval_s;
}

#endif
