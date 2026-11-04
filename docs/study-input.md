# Study: input & settings seam for interactive cartridges

Status: design study (Wave T1, branch `toy1-input`, base `8f63c78`).
No code in this parcel. It designs the seam that
[CARTRIDGES.md](CARTRIDGES.md) lists as not yet parcel-ready —
"No input event, settings, or parameter interface" — so that Wave T2
interactive cartridges (gravity sandbox, Julia-by-touch, parameterised
carts; deferred in [WAVE-T1.md](WAVE-T1.md)) become ordinary
cartridge parcels.

**Recommendation in one paragraph.** Add an optional pre-step
`apply()` callback to the cartridge descriptor, using appended
descriptor fields only. Runners convert every input source (touch,
IMU-derived events, encoder/buttons, browser pointer/keys, emu script
file) into one fixed-size integer-only `toy_event_t`, quantise events
to step boundaries, call `apply()` zero or more times between steps,
then call the unchanged `step()`. User settings are a per-cartridge
parameter table whose values enter state as `SET_PARAM` events (at
step 0 after `reset()`, or live at a step boundary). Proof runs for
interactive carts replay a compiled-in, step-indexed input script
carried in the descriptor. Cartridges store only event *effects* in
state, so the existing state hash covers inputs, and existing carts
stay source- and hash-identical.

Contents: §1 today (evidence) · §2 what T2 needs · §3 options ·
§4 recommendation · §5 C interface · §6 determinism contract ·
§7 settings · §8 coordinates · §9 runner mappings · §10 runner diffs ·
§11 worked examples · §12 migration · §13 T2 parcels ·
§14 non-goals/risks · §15 owner questions · appendices.

## 1. What exists today

Every claim in this section cites the code as of base `8f63c78`.

### 1.1 Cartridge contract

- The descriptor is `toy_cart_t` with `name/title/blurb`,
  `state_size`, pacing/proof fields, and exactly three callbacks,
  `reset(void*)`, `step(void*)`, `render(const void*, toy_fb_t*)`
  ([core/toy_cart.h:21](../core/toy_cart.h)).
  `step()` "advances exactly one fixed internal dt"
  ([core/toy_cart.h:6](../core/toy_cart.h)).
- Hosts allocate at most `TOY_MAX_STATE_SIZE` = 64 KiB of state
  ([core/toy_cart.h:19](../core/toy_cart.h)); state is POD
  bit-pattern integers only, no pointers/heap/native float, zeroed
  before `reset()`, and `reset()` memsets the whole struct so padding
  is defined before hashing ([CARTRIDGES.md:19](CARTRIDGES.md)).
- `step()` "does not allocate, print, read input, or branch on
  platform" and `render()` is const in effect
  ([CARTRIDGES.md:34](CARTRIDGES.md)). All state floating-point
  goes through `toy_math.h` wrappers over muse-softfloat; constants
  come from decimal strings via `f64_dec()` at reset
  ([CARTRIDGES.md:27](CARTRIDGES.md),
  [core/toy_math.h:23](../core/toy_math.h)). `toy_math.h` today has
  no integer→`f64` wrapper ([core/toy_math.h:28](../core/toy_math.h)
  lists arithmetic only), although the library provides
  `msf64_from_i64` (`libs/muse-softfloat/include/muse_softfloat.h:642`).
  Interactive carts need that wrapper (§5.4).
- The proof hook is FNV-1a 64 over raw state bytes
  ([core/toy_cart.h:45](../core/toy_cart.h),
  [core/toy_carts.c:18](../core/toy_carts.c)). Identical step
  counts must give identical hashes on host, RP2040 and RP2350.
- A new cartridge currently enters every runner by registration
  alone — one `core/cart_<name>.c`, one `extern` in `toy_cart.h`, one
  registry line in `toy_carts.c`, one source line in
  `board/CMakeLists.txt` — and "must not require edits to the
  emulator, board runner logic, WASM glue or website"
  ([CARTRIDGES.md:3](CARTRIDGES.md)). The registry itself is a plain
  array plus count ([core/toy_carts.c:5](../core/toy_carts.c)).
  The input seam must preserve this property: after the seam lands,
  an interactive cart is still registration-only.
- Both shipping carts initialise their descriptors positionally with
  exactly the eight current fields, e.g. fig8
  ([core/cart_fig8.c:135](../core/cart_fig8.c)) and pendulum
  ([core/cart_pendulum.c:144](../core/cart_pendulum.c)). Wave T1
  adds ten more carts in the same style
  ([WAVE-T1.md:12](WAVE-T1.md)). Any descriptor evolution must be
  append-only or it rewrites every cart (§12).
- Framebuffers are square RGB565 with runtime size (240 or 360);
  carts draw via centre/radius helpers and must never assume a size
  ([core/toy_fb.h:14](../core/toy_fb.h),
  [CARTRIDGES.md:52](CARTRIDGES.md)). Input coordinates need the
  same resolution independence (§8).
- Pacing varies a lot per cart: fig8 demos at 4 steps/frame and
  proves 2000 steps ([core/cart_fig8.c:138](../core/cart_fig8.c));
  pendulum demos at 30 steps/frame and proves 1000 steps
  ([core/cart_pendulum.c:147](../core/cart_pendulum.c)).
  Interactive delivery must therefore define which step of a
  multi-step frame receives events (§6.2).

### 1.2 Runners

There are three runners, and none has a live "play" loop on hardware
yet. All three share one shape: zero state, `reset()`, loop `step()`,
hash state.

**Host emulator** (`emu/main_emu.c`). Three modes plus `list`:
`proof <cart>` resets and steps `proof_steps`, printing
`TOY-RUN/step/TOY-DONE` hash lines
([emu/main_emu.c:54](../emu/main_emu.c)); `frames` steps
`steps-per-frame × frames`, rendering PPMs and per-frame hashes
([emu/main_emu.c:70](../emu/main_emu.c)); `run` is `proof` with
explicit counts ([emu/main_emu.c:94](../emu/main_emu.c)).
Argument arities are checked exactly, so new modes must be additive,
not arity-changing. The determinism budget runs `toy_emu run <cart>
10000 1000` twice and byte-compares ([tools/run.sh:39](../tools/run.sh));
the host budget renders 40 frames at both sizes
([tools/run.sh:25](../tools/run.sh)). The emu build globs
`core/*.c` and `emu/*.c` ([tools/run.sh:14](../tools/run.sh)), so a
new core file joins the emu build automatically.

**Board proof firmware** (`board/main_board.c`). Core-only and
headless, explicitly "no display yet"
([board/main_board.c:1](../board/main_board.c)). It runs *every*
registered cart's proof run in sequence and streams the same line
format over USB CDC ([board/main_board.c:21](../board/main_board.c),
[board/main_board.c:55](../board/main_board.c)). The board budget
flashes by serial under a lock and diffs the captured stream against
`toy_emu proof` output for every cart
([board/run_toy.sh:50](../board/run_toy.sh)). Board sources are
enumerated explicitly, carts included
([board/CMakeLists.txt:29](../board/CMakeLists.txt)) — deliberately,
so a new board-visible file is a reviewed line
([CARTRIDGES.md:85](CARTRIDGES.md)). There is no display, touch,
IMU, pacing, or cartridge-switching firmware today; anything this
study says about a "live board runner" describes future firmware,
labelled as such.

**WASM site** (`site/wasm_glue.c`, `site/app.js`). The glue holds one
selected cart, one state, one framebuffer, and a step counter
([site/wasm_glue.c:14](../site/wasm_glue.c)). `toy_wasm_select`
zeroes state and resets ([site/wasm_glue.c:64](../site/wasm_glue.c));
`toy_wasm_step(count)` steps in a batch, capped at 100 000
([site/wasm_glue.c:75](../site/wasm_glue.c)); render is separate
and size-parameterised ([site/wasm_glue.c:83](../site/wasm_glue.c)).
Exports are an explicit list in the build script
([site/build.py:19](../site/build.py)); the core sources are
globbed ([site/build.py:44](../site/build.py)). The Node checker
selects each cart and calls `toy_wasm_step(1)` per proof step,
comparing hashes to pinned values
([site/check_wasm.mjs:23](../site/check_wasm.mjs),
[site/expected-hashes.json:1](../site/expected-hashes.json)).
The page steps from `requestAnimationFrame` with an elapsed-time
step debt and a speed multiplier — batch sizes vary with wall time
([site/app.js:99](../site/app.js)). Its only inputs today are page
chrome (cart select, play/reset, speed, display mode)
([site/app.js:176](../site/app.js),
[site/index.html:18](../site/index.html)); the two canvases exist
but have no pointer handlers ([site/index.html:40](../site/index.html)).
Deep links pin `?cart=`, `?steps=`, `?pause=1`
([site/app.js:145](../site/app.js)).

### 1.3 Hardware context (kits in transit)

TOY-0 targets two Waveshare kits, neither arrived when its text was
written: RP2350-Touch-LCD-1.85C, 360×360, CST816 touch, 6-axis IMU,
RTC, mic/codec, microSD ([TOY-0.md:23](TOY-0.md)); and
RP2040-Touch-LCD-1.28-B, 240×240, CST816-class touch, QMI8658C IMU
([TOY-0.md:36](TOY-0.md)). Display/touch/IMU drivers are a
per-board layer; "nothing chip-specific enters `core/`"
([TOY-0.md:42](TOY-0.md)). TOY-0 sketches device gestures — swipe =
next cartridge, rim-swipe = warp dial, tap = interact; IMU tilt,
shake, face-down = sleep, pickup = wake — and a dual-core split where
the sim never waits on display/input ([TOY-0.md:73](TOY-0.md)).
Whether physical buttons or a rotary encoder will ever be fitted on
the SH1.0 GPIOs is an open owner question in TOY-0
([TOY-0.md:155](TOY-0.md)). No driver code, register maps, touch
point counts, or IMU scale settings exist in the repo; §9 therefore
maps at *driver-output* level and marks thresholds provisional.

## 2. What Wave T2 needs from the seam

Derived from the deferred list ([WAVE-T1.md:37](WAVE-T1.md)) and
the study brief. Requirements are numbered for §13 traceability.

- **R1 Pointer.** Tap and drag with coordinates: Julia-by-touch maps
  a finger position to the Julia constant *c* continuously; gravity
  sandbox needs press-drag-release (place/fling a body). Needs
  down/move/up, at least one contact id, resolution-independent
  coordinates, and move coalescing (a 30-steps/frame cart cannot
  consume raw 100 Hz touch reports one per step unboundedly).
- **R2 Discrete controls.** Buttons/keys/encoder for parameterised
  carts and for hosts without touch (keyboard in browser, scripted
  keys in emu). Needs down/up and signed detent deltas with a small,
  runner-independent code namespace.
- **R3 IMU-derived events.** Tilt-as-gravity (sandbox, attitude
  toys), shake, face-up/down. This study covers *derived, quantised*
  events only. Raw sample streams with their own clock (seismometer,
  FFT/spectrum carts) belong to the toy1-signals study; TOY-0 already
  separates "sensor input abstraction" as future work
  ([CARTRIDGES.md:83](CARTRIDGES.md),
  [WAVE-T1.md:30](WAVE-T1.md)).
- **R4 Settings.** Per-cart user-tunable parameters (body count,
  zoom, palette, gravity scale…) with types, ranges and defaults,
  settable at reset and, where sensible, live — without a bespoke
  code path per runner or per cart.
- **R5 Proof.** Every interactive cart still gets a standard
  host/board/WASM proof run that joins the existing budgets
  automatically ([CARTRIDGES.md:61](CARTRIDGES.md)). "Interactive"
  must not mean "untestable headless" — the board proof binary has
  no glass at all ([board/main_board.c:1](../board/main_board.c)).
- **R6 Replay.** A recorded live session (on any runner) must be
  replayable on the others to identical hashes. That is the input
  form of the core invariant: same steps + same inputs ⇒ identical
  state hash on host, board, WASM.
- **R7 Bounded work.** `step()` is bounded today
  ([CARTRIDGES.md:34](CARTRIDGES.md)); input handling must be
  bounded per step too, with a specified overflow/coalescing policy.
- **R8 No churn.** Existing and in-flight T1 carts keep their source
  and their pinned proof hashes ([site/expected-hashes.json:1](../site/expected-hashes.json));
  the `run.sh check` streams for fig8/pendulum (final hashes recorded
  in [RECORD.md:17](RECORD.md)) must not move.
- **R9 Parcel-readiness.** After the seam, a new interactive cart is
  still four registration touches ([CARTRIDGES.md:3](CARTRIDGES.md));
  no runner edits per cart. Runner UIs (param controls, pointer
  capture) must be generatable from descriptor metadata.

## 3. Options for getting events to a cartridge

Three shapes were evaluated. All three assume the same event struct
(§5.1) and step-boundary quantisation (§6.1); they differ in delivery.

### Option A — `step(st, events, n)` signature change

`void (*step)(void *st, const toy_event_t *ev, uint32_t n)`. Runners
build a per-step array; the cart consumes inputs then integrates, in
one call.

- (+) Atomic: inputs and the step they affect are inseparable; no
  observable state exists "between apply and step".
- (+) No per-event call overhead; cart sees the whole batch (can
  coalesce internally).
- (−) Breaks every existing/in-flight cart source (signature edit in
  fig8, pendulum, ten T1 carts) and every call site
  ([emu/main_emu.c:61](../emu/main_emu.c),
  [board/main_board.c:28](../board/main_board.c),
  [site/wasm_glue.c:79](../site/wasm_glue.c)). Mechanical, but it
  is exactly the churn R8 forbids in spirit, and it land-mines every
  open T1 branch with merge conflicts in the same lines.
- (−) Batch ambiguity moves into the API: `toy_wasm_step(count)`
  steps many times per call ([site/wasm_glue.c:75](../site/wasm_glue.c)),
  so either WASM grows a second step API or events may only attach
  to single-step calls — an awkward invariant to teach.
- (−) Deaf carts must still accept and ignore parameters forever;
  "does this cart listen?" is not visible in the descriptor.

### Option B — bounded input ring in or near state

Two sub-variants. **B-in:** every state embeds a small event ring
(shared header struct at offset 0); runners write events into the
ring; `step()` drains it. **B-near:** the runner owns a ring beside
the state buffer; `step()` reads it via a core function/global.

- (+) Runners can push from interrupt-ish contexts (future core-1
  input task, [TOY-0.md:76](TOY-0.md)) without call sequencing.
- (−) B-in rewrites every state layout and `state_size` ⇒ every proof
  hash churns; ring spare capacity/slack bytes must be kept
  deterministically defined inside hashed state forever; the 64 KiB
  budget ([core/toy_cart.h:19](../core/toy_cart.h)) is spent by
  carts that need no input. Fails R8 outright.
- (−) B-near puts pending inputs outside hashed state: a hash no
  longer determines the future, checkpoints miss pending events, and
  `step()` now "reads input" against the written rule
  ([CARTRIDGES.md:34](CARTRIDGES.md)). It also needs a current-ring
  global or extra parameter — reentrancy and snapshot bugs (dual-core
  double buffering) follow.
- (−) Ring capacity/timeout policy becomes sim semantics: "event
  arrived while ring full" diverges across runners unless the ring is
  itself replayed — at which point it is a script (§6.5) with extra
  machinery.

### Option C — optional pre-step `apply()` callback (recommended)

Additive descriptor field `void (*apply)(void *st, const toy_event_t *ev)`,
`NULL` for deaf carts. The runner, at each step boundary, calls
`apply()` once per queued event in order, then calls unchanged
`step()`.

- (+) `reset/step/render` untouched; appended fields keep positional
  initialisers valid (§12) — R8 holds with zero cart edits.
- (+) "Listens?" is visible (`apply != NULL`, plus a caps mask, §5.3)
  so runners can skip queueing/UI for deaf carts — R9.
- (+) Batching stays a runner concern with one specified rule
  (§6.2); `toy_wasm_step(count)` keeps its shape.
- (+) Settings unify with events (`SET_PARAM`, §7) — one delivery
  path for "thing the user changed", live or at reset.
- (+) A single core helper performs "apply script/queue for step s,
  then step" for all three runners, so proof semantics cannot drift
  between emu/board/WASM the way three hand-copied loops could.
- (−) A state exists transiently between `apply()` and `step()` at
  which a hash would include input effects but not the integration
  they precede. Mitigated by rule: hashes/checkpoints are only taken
  after `step()` (§6.4). Today's hosts already hash only after
  stepping ([emu/main_emu.c:61](../emu/main_emu.c)).
- (−) Per-event call overhead. Harmless at capped counts (§6.3):
  worst case 16 calls per step vs an integrator step's softfloat cost.
- (−) The cart cannot see a whole batch atomically; it must tolerate
  being observed (rendered) only after the batch, which runners
  guarantee by construction (render happens after stepping in all
  three runners today).

**Decision: Option C.** It is the only option satisfying R8 (no
churn) and R6 (hash covers inputs via state effects) without new
globals, and it matches how the descriptor already absorbs per-cart
variation (pacing/proof today, [core/toy_cart.h:26](../core/toy_cart.h)).
Option A remains the fallback if a future cart genuinely needs batch
atomicity; §14 names the trigger for revisiting.

## 4. Recommendation at a glance

- New header `core/toy_input.h`: `toy_event_t`, event types, key
  codes, caps, parameter descriptors, proof-script entries, limits.
- New `core/toy_input.c`: coordinate conversion, script/param
  validation helpers, the shared step-with-inputs helper.
- `core/toy_cart.h`: four appended descriptor fields (`apply`,
  `params`/`param_count`, `proof_inputs`/`proof_input_count`,
  `input_caps`). Nothing moves, nothing renamed.
- Events quantised to step boundaries; at most
  `TOY_MAX_EVENTS_PER_STEP` (16) applied per step; MOVE coalescing
  defined; runner live queues bounded at 64.
- Proof for interactive carts = descriptor proof script replayed at
  exact step indices (§6.5). Scriptless carts: byte-identical
  output to today.
- Settings = `SET_PARAM` events against a descriptor param table;
  defaults live in the table; `reset()` alone still means "defaults"
  (the proof baseline); runners apply user settings as step-0 events.
- Runner chrome (cart switching, pause, warp, sleep) never enters
  cartridge state; rim band reserved for chrome on glass (§9.1).

## 5. Concrete C interface

Illustrative code below is the proposed interface, not a patch.
Field order, sizes and zero-fill rules are part of the design;
constant *values* marked provisional may be tuned by the implementing
parcel without changing the contract.

### 5.1 `toy_event_t`

```c
/* core/toy_input.h (new) */
#ifndef TOY_INPUT_H
#define TOY_INPUT_H
#include <stdint.h>

/* Limits. A runner never applies more than MAX_EVENTS_PER_STEP
 * events before one step(); the live queue holds at most QUEUE. */
#define TOY_MAX_EVENTS_PER_STEP 16u
#define TOY_INPUT_QUEUE_LEN     64u
#define TOY_MAX_PARAMS          12u   /* per cart, keep UI on glass sane */
#define TOY_MAX_PROOF_INPUTS  4096u   /* entries; board flash is finite */

enum {
    TOY_EV_NONE = 0,
    TOY_EV_POINTER_DOWN = 1,  /* id=contact, x/y=milli-radius (§8) */
    TOY_EV_POINTER_MOVE = 2,
    TOY_EV_POINTER_UP   = 3,  /* also used for cancel/lift-outside */
    TOY_EV_KEY_DOWN     = 4,  /* id=TOY_KEY_*, v=1 if auto-repeat */
    TOY_EV_KEY_UP       = 5,
    TOY_EV_BUTTON_DOWN  = 6,  /* id=button index 0.. */
    TOY_EV_BUTTON_UP    = 7,
    TOY_EV_ENCODER      = 8,  /* id=encoder index, v=summed detents */
    TOY_EV_TILT         = 9,  /* x/y=milli-g quantised, see §9.1 */
    TOY_EV_SHAKE        = 10, /* v=intensity bucket 1..3 (provisional) */
    TOY_EV_FACE         = 11, /* v: 0=unknown 1=up 2=down (provisional) */
    TOY_EV_SET_PARAM    = 12  /* id=param index; v or bits=value (§7) */
};

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

/* Fixed 32 bytes, integers only, no pointers, no native float.
 * Producers zero the whole struct, then fill the fields named for
 * the type; every other field stays 0. Consumers ignore fields not
 * named for the type. There is deliberately no timestamp: the step
 * an event is applied to IS its time (§6.1). */
typedef struct {
    uint32_t type;   /* TOY_EV_* */
    uint32_t id;     /* contact / key / button / encoder / param idx */
    int32_t  x;      /* pointer: milli-radius, +right (§8) */
    int32_t  y;      /* pointer: milli-radius, +down (display space) */
    int32_t  v;      /* delta / bucket / repeat / int param value */
    uint32_t flags;  /* reserved, must be 0 in v1 */
    uint64_t bits;   /* SET_PARAM f64 payload: IEEE bits, else 0 */
} toy_event_t;
```

Design notes:

- **Integer-only, like state.** Events obey the same discipline as
  state structs ([CARTRIDGES.md:19](CARTRIDGES.md)): no native
  `float`/`double` anywhere on the producer→cart path, so a JS,
  driver or text-script producer cannot smuggle platform rounding
  into the sim. Floating settings travel as decimal-derived bits
  (§7), coordinates as milli integers (§8).
- **Fixed size, explicit zero fill.** Scripts and recordings must be
  byte-comparable across runners; "zero then fill" plus a layout of
  naturally aligned words (7×32-bit + 1×64-bit, no hidden padding on
  any current target) gives that. The implementing parcel adds a
  `_Static_assert(sizeof(toy_event_t)==32)`.
- **One contact model, room for two.** CST816-class parts are
  typically single-point (exact variant unconfirmed until kits
  arrive, §1.3); browser pointers can be multi. Runners assign small
  contact ids (0 = primary). Carts must behave sanely if only id 0
  ever appears; two-contact gestures are runner chrome, not cart
  input (§9.1).
- **No raw sensor samples.** There is no `TOY_EV_ACCEL_SAMPLE` by
  design (R3): samples need a sample clock and decimation contract
  owned by toy1-signals. `TILT/SHAKE/FACE` are runner-derived,
  rate-limited summaries (§9.1).

### 5.2 Parameters and proof script (descriptor payloads)

```c
enum { TOY_PARAM_I32 = 1, TOY_PARAM_ENUM = 2, TOY_PARAM_F64 = 3 };
#define TOY_PARAM_RESET_ONLY (1u<<0) /* live SET is queued but the
                                      * runner re-resets (§7.3)     */
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
 * step. step==0 means "after reset, before step 1" (§7.2). */
typedef struct {
    uint32_t step;
    toy_event_t ev;
} toy_proof_input_t;
```

`F64` bounds/defaults are decimal *strings* so they are parsed by
`f64_dec()` exactly like cart constants today
([core/toy_math.h:23](../core/toy_math.h),
[CARTRIDGES.md:30](CARTRIDGES.md)) — identical bits on all
runners, including the browser, whose JS never parses them as floats
for sim purposes (display-only parsing for slider placement is
allowed, mirroring the render rule,
[CARTRIDGES.md:36](CARTRIDGES.md)).

### 5.3 Descriptor diff (`core/toy_cart.h`)

Append-only. Existing eight fields keep order, types and meaning
([core/toy_cart.h:21](../core/toy_cart.h)):

```c
#include "toy_input.h"   /* new, beside toy_fb.h include */

typedef struct {
    /* ... existing 8 fields unchanged ... */
    void (*render)(const void *st, toy_fb_t *fb);
    /* --- input & settings seam (all zero for deaf carts) --- */
    void (*apply)(void *st, const toy_event_t *ev); /* or NULL */
    const toy_param_t *params;          /* or NULL */
    uint32_t param_count;               /* <= TOY_MAX_PARAMS */
    const toy_proof_input_t *proof_inputs; /* or NULL */
    uint32_t proof_input_count;         /* <= TOY_MAX_PROOF_INPUTS */
    uint32_t input_caps;                /* TOY_CAP_* mask */
} toy_cart_t;
```

`apply()` obeys the same rules as `step()` in
[CARTRIDGES.md:27](CARTRIDGES.md): softfloat only, no allocation,
no I/O, no wall clock, bounded work, and it writes only its own
state. It must ignore unknown event types, out-of-range ids and
out-of-range param values (after clamping per §7.1) — runners are
trusted to quantise, never to be bug-free.

Descriptor validity (checked by a core helper, asserted in debug
runner builds, listed in Appendix B): `proof_inputs!=NULL` or
`param_count>0` implies `apply!=NULL`; `param_count>0` implies
`params!=NULL` and `input_caps & TOY_CAP_PARAMS`; proof steps are
non-decreasing and `<= proof_steps` (step-0 entries first).

### 5.4 Core helpers (`core/toy_input.c`, new)

One implementation, used by every runner — this is what makes
"same inputs" mean the same thing everywhere:

```c
/* Display pixels -> milli-radius display space (§8).
 * size_px is the square size (240/360/canvas px). Integer math
 * only; round-to-nearest, ties away from zero; clamped to
 * +/-TOY_COORD_MAX (1500). */
int32_t toy_px_to_milli(int32_t px, int32_t size_px);
void toy_pointer_event(toy_event_t *ev, uint32_t type, uint32_t id,
                       int32_t px, int32_t py, int32_t size_px);

/* Apply proof-script entries for step s (s==0: post-reset setup).
 * cursor is runner-owned scratch (index into proof_inputs).
 * No-op when the cart has no script - the migration fast path. */
void toy_apply_proof_at(const toy_cart_t *cart, void *st,
                        uint32_t step, uint32_t *cursor);

/* Apply n live events (n capped internally, §6.3), then NOT step:
 * runners keep owning their loops (§10). Also coalesces MOVEs. */
uint32_t toy_apply_live(const toy_cart_t *cart, void *st,
                        const toy_event_t *evs, uint32_t n);

/* Value plumbing for SET_PARAM (§7): make a default/explicit
 * parameter event; validate+clamp is the cart's job in apply(). */
int toy_param_event(const toy_cart_t *cart, uint32_t idx,
                    int32_t ival, const char *fdec_or_null,
                    toy_event_t *out);

/* FNV-1a over canonical little-endian fields (not raw struct
 * padding): identifies a script in logs/checkpoint headers (§6.5). */
uint64_t toy_hash_inputs(const toy_proof_input_t *in, uint32_t n);

/* Returns 0 if the descriptor's input fields are consistent
 * (Appendix B), else a small nonzero code. Runners call once at
 * select/start; board proof prints TOY-BAD-CART and skips. */
int toy_cart_check_input(const toy_cart_t *cart);
```

Plus one `toy_math.h` addition, needed by every cart that turns
milli coordinates or int params into softfloat state:

```c
/* core/toy_math.h: beside f64_dec (uses msf64_from_i64) */
static inline f64 f64_from_i32(int32_t a)
{ msf_flags_t f = 0; return msf64_from_i64((int64_t)a,
                              MSF_ROUND_TIES_TO_EVEN, &f); }
static inline f64 f64_from_milli(int32_t milli) /* milli/1000 */
{ return f64_div(f64_from_i32(milli), f64_dec("1000")); }
```

Exact division by 1000 via one softfloat divide is deterministic;
carts cache derived `f64` values in state at `apply()` time rather
than recomputing per step where natural (§11).

## 6. Determinism contract

Invariants, restated with input added: *(a)* same reset + same
ordered sequence of (events-per-step, step) ⇒ identical state bytes
(or the round does not close); *(b)* `render()` never affects state
([CARTRIDGES.md:36](CARTRIDGES.md)); *(c)* nothing in
`reset/apply/step` reads wall time, platform, or uninitialised
memory ([CARTRIDGES.md:27](CARTRIDGES.md)). Everything below is
mechanism for *(a)*.

### 6.1 Quantisation to step boundaries

Step indices are 1-based, matching every runner's loop today (emu
`proof` counts `s=1..proof_steps`,
[emu/main_emu.c:60](../emu/main_emu.c); board likewise,
[board/main_board.c:27](../board/main_board.c); WASM checker
likewise, [site/check_wasm.mjs:30](../site/check_wasm.mjs)).

> An event assigned to step *s* is applied after step *s−1* has
> completed and before step *s* integrates. Step 0 is the setup
> slot after `reset()`, before step 1. Within one step, events apply
> in queue/script FIFO order. There are no sub-step timestamps, no
> interpolation, and no event may be applied "during" a step — a
> cart deriving drag velocity computes it from event positions and
> step counts stored in its own state.

Consequences a cart author must know: two events in either order at
the same step may give different results (e.g. DOWN then UP = tap;
UP then DOWN = new press) — order is semantics and is preserved by
scripts. Rapid physical motion samples collapse to what survives
coalescing (§6.3); carts needing true high-rate signals wait for
toy1-signals (R3).

### 6.2 Multi-step frames and batched stepping

Live runners step several times per displayed frame
(`demo_steps_per_frame`, e.g. 30 for pendulum,
[core/cart_pendulum.c:147](../core/cart_pendulum.c)) or in
wall-time-sized batches ([site/app.js:110](../site/app.js)).

> **Frame rule:** a live runner drains its queue once per displayed
> frame (or per step call batch, §10.3) and applies the drained
> events before the **first** step of that batch; remaining steps in
> the batch receive no events. The recorded step number of a drained
> event is `steps_done + 1` at drain time — the step it will precede.

Spreading events across a batch by arrival wall-time was rejected:
arrival times are runner- and load-dependent, so two runners would
assign different steps to the "same" physical gesture and neither
could replay the other. First-step assignment has a known feel cost —
at 30 steps/frame an interactive cart integrates up to 29 steps
before drawing the response — so interactive carts should choose
modest `demo_steps_per_frame` (guideline: small enough that
frames stay ≈30 fps *and* response lands within one frame of wall
time; pacing choice remains the cart's, as today,
[CARTRIDGES.md:42](CARTRIDGES.md)). Proof runs never batch:
they assign by explicit script step (§6.5).

### 6.3 Caps, coalescing, overflow

- At most `TOY_MAX_EVENTS_PER_STEP` (16) events are applied before
  any one step — from live queues *and* from scripts (a script
  exceeding the cap at a step is a descriptor bug caught by
  `toy_cart_check_input`, Appendix B). This bounds `apply()` work
  per step (R7).
- Live queues hold `TOY_INPUT_QUEUE_LEN` (64) events. On enqueue,
  a `POINTER_MOVE` for contact *i* replaces an un-drained
  `POINTER_MOVE` for *i* in place (latest position wins, order slot
  of the first is kept); other types never coalesce. This bounds
  drag floods while keeping DOWN/UP ordering intact.
- On queue-full enqueue: drop the *new* event, except `POINTER_UP`,
  which evicts the oldest queued `POINTER_MOVE` (if any) to make
  room — a lost release wedges cart state, a lost move does not.
  Runners count drops in runner-side diagnostics only (never in
  state). Proof/scripted runs must be authored under the caps, so
  the drop path — which is inherently timing-dependent — is
  unreachable in any replayed run. **Live overflow is therefore the
  one place where "what the user did" can be lost irreproducibly;
  what replays is what was applied and recorded (§6.6).**

### 6.4 What is stored in state (and what is not)

**No generic event log in state; no generic last-event field; no
settings blob.** Reasons: a log is unbounded against the 64 KiB cap
([core/toy_cart.h:19](../core/toy_cart.h)); a uniform header
would churn every hash (§3, B-in); and neither is needed, because:

- **Hashes already cover inputs through effects.** `apply()` writes
  ordinary state fields (cursor `f64`s, spawned bodies, effective
  param values) and the hash is over all state bytes
  ([core/toy_carts.c:18](../core/toy_carts.c)). Two runs that
  applied different inputs diverge in hash iff the difference
  mattered to the sim — the right granularity. Carts must not keep
  "input shadow state" outside their state struct (no globals):
  anything affecting `step()` must be in hashed state, exactly as
  trail history already is ([CARTRIDGES.md:55](CARTRIDGES.md)).
- **Replay is external.** Reproducibility comes from scripts
  (§6.5/§6.6), not from state remembering its inputs.
- **Hash/checkpoint times are fixed:** only immediately after a
  `step()` returns (never between `apply()` and `step()`), matching
  every current host ([emu/main_emu.c:61](../emu/main_emu.c),
  [board/main_board.c:28](../board/main_board.c),
  [site/wasm_glue.c:97](../site/wasm_glue.c)).

### 6.5 Proof runs for interactive carts

Today a proof run is `proof_steps` + `proof_report_every` from the
descriptor ([core/toy_cart.h:29](../core/toy_cart.h)), printed as
`TOY-RUN/step/TOY-DONE` and diffed host-vs-board
([board/run_toy.sh:54](../board/run_toy.sh)), and hash-pinned for
WASM ([site/expected-hashes.json:1](../site/expected-hashes.json)).

With the seam:

> **Interactive proof = reset (defaults) → apply `proof_inputs`
> entries at their steps (step 0 first) → step `proof_steps` times,
> hashing on the existing `proof_report_every` schedule.** The
> script lives in the cart's own `.c` file as a `static const`
> array referenced by the descriptor, so choosing proof inputs is
> part of authoring the cart — like choosing `proof_steps` today
> ([CARTRIDGES.md:45](CARTRIDGES.md)).

Authoring guidance: script a gesture that exercises the phenomenon
(drag across the dial, tap to spawn, tilt, param change), leave the
initial transient behind, keep board proof duration in the usual
≈3-minute envelope ([CARTRIDGES.md:47](CARTRIDGES.md)). A cart
with `apply!=NULL` but an empty proof script is legal only if inputs
are purely cosmetic to its proof phenomenon — reviewers should treat
an empty script on a gesture-driven cart as a finding.

Output discipline (protects R8 and the diff-based board gate):

- Carts with no proof inputs print **byte-identical** lines to
  today. `grep -E '^(TOY-RUN|step|TOY-DONE)'` in the board budget
  ([board/run_toy.sh:57](../board/run_toy.sh)) needs no change.
- Carts *with* inputs add fields to their own `TOY-RUN` line, e.g.
  `TOY-RUN cart=julia state_size=… hash=fnv1a64 inputs=37
  input_hash=0123456789abcdef`, where `input_hash` is
  `toy_hash_inputs` (§5.4). Host and board compute it from the same
  descriptor array, so the diff still proves "same script" before
  any step hashes are compared. Emu, board and WASM checker must
  all emit/verify it (§10).

Why a compiled-in array rather than a text script in the descriptor:
text would need a parser in the freestanding WASM/board core (today
deliberately libc-free, [core/toy_util.h:1](../core/toy_util.h)),
and three parsers could drift; an array is data the compiler checks.
Text exists only at the emu/host edge (§9.3, Appendix A) and is
converted to the same entry structs before entering the core.

### 6.6 Live sessions, recording and replay

A live runner can record a session by logging, for each drained
event, its effective step (§6.2) and the event fields — i.e. it
emits a `toy_proof_input_t` stream. Replaying that stream through
any runner's scripted path (emu §10.1, WASM proof mode §10.3, board
proof §10.2 with the stream compiled/host-fed) reproduces the
session hashes. Recording is *runner* functionality: whether/when
devices persist recordings is an owner/storage question (§15), and
the recording format must include cart name, `state_size`, total
steps, and `toy_hash_inputs` of the stream so a mismatched replay
fails loudly instead of drifting. The step counter to stamp events
with already exists in WASM glue (`toy_steps_done`,
[site/wasm_glue.c:18](../site/wasm_glue.c)); emu/board live
runners must keep the same counter (proof loops already count
steps explicitly).

### 6.7 Checkpoint interaction (interface points only)

Checkpoint format/versioning is deliberately undefined today beyond
"state bytes are the save file" ([CARTRIDGES.md:80](CARTRIDGES.md))
and is owned by the toy1-checkpoint study. This seam pins only what
that format must accommodate:

1. Checkpoints are taken after `step()` with the live queue drained
   (§6.4). Pending un-applied events are never part of a checkpoint;
   a runner wanting to checkpoint mid-gesture drains first (the
   gesture's applied prefix is in state via effects).
2. The checkpoint header must store `steps_done` and an input-stream
   identity (script name/`toy_hash_inputs` + next script cursor for
   scripted runs; "live" marker otherwise), so resume neither
   re-applies past events nor skips future script entries.
3. Effective parameter values are already inside state (§7.1), so a
   restored checkpoint needs no settings re-application; step-0
   `SET_PARAM` events before the checkpoint must not be replayed.
4. Restoring a checkpoint and then replaying the suffix of its
   recording (§6.6) from `steps_done` must reproduce the original
   hashes — the checkpoint study's acceptance test should include an
   interactive cart mid-drag.

## 7. Settings (per-cartridge parameters)

### 7.1 Model

A cart publishes `params[0..param_count)` (§5.2): name (stable key
for scripts and persistence), label (UI), kind, range and default.
Three kinds cover T2: `I32` (counts, seeds-as-integers — note a seed
param is just initial state, parsed/stored as integers), `ENUM`
(index + names; booleans are 2-value enums — no `_Bool` in state,
per the padding rule, [CARTRIDGES.md:20](CARTRIDGES.md)), and
`F64` (decimal-string bounded softfloat quantities). No string,
colour-picker or free-float params in v1; a cart needing one is a
study, not a smuggled kind.

Delivery is exclusively via `SET_PARAM` events: `id` = param index;
`I32/ENUM` value in `v`; `F64` value as IEEE bits in `bits`
(produced from a decimal string by `toy_param_event`, never from a
host `double`). The cart's `apply()` clamps to the declared range
(integer clamp, or softfloat compare via library calls for `F64`),
stores the *effective* value in its own state fields in whatever
form `step()` consumes, and derives cached quantities there. The
descriptor table is metadata outside state; there is no requirement
to store the raw last `SET_PARAM`, only effects (same rule as §6.4).

### 7.2 Reset and step-0 application

`reset()` keeps today's meaning and signature: full memset, then
defaults — explicitly including applying every param default to the
same state fields `apply()` would write (implementation guidance:
factor a `set_param(st, idx, val)` internal used by both). The
proof baseline is therefore always "defaults + proof script", with
no runner settings involved.

A runner starting a cart with user settings performs:

```c
cart->reset(st);                       /* defaults */
for (idx in 0..param_count)           /* index order, always */
    if (user_value[idx] != default)
        cart->apply(st, &set_param_ev(idx, user_value[idx]));
/* then step 1... */
```

These are exactly step-0 script entries (§6.1), so a recording that
includes the user's step-0 settings replays bit-identically, and a
proof script may itself use step-0 `SET_PARAM` to prove a
non-default configuration. Runners apply in param-index order so
cross-param derivations inside `apply()` observe a defined sequence;
carts should nevertheless make `apply(SET_PARAM)` order-tolerant
where cheap (recompute derived values from all stored raw values).

### 7.3 Live tuning and reset-only params

A live param change is a `SET_PARAM` enqueued like any event and
applied at the next step boundary — sliders therefore have
step-quantised effect and appear in recordings for free. Params
whose change only makes sense from a clean state (e.g. body count
allocating trail slots) set `TOY_PARAM_RESET_ONLY`: the cart's
`apply()` still accepts the event, but runners presenting the
control restart the cart (`reset` + step-0 reapplications of *all*
current settings) on change, discarding run progress. The flag is a
runner-presentation contract; hashes stay well-defined either way.

### 7.4 Persistence (boundary, not design)

This study does **not** design settings storage. References for the
checkpoint study ([WAVE-T1.md:32](WAVE-T1.md)): persistence keys
are `(cart name, param name)` — names, not indices, so reordering a
table degrades to "fall back to defaults" rather than silently
mis-assigning values; on fresh start the stored values are merely
the `user_value[]` of §7.2; on checkpoint resume they are ignored
(state already carries effect values, §6.7). Unknown/renamed params
in a stored blob are dropped. Browser `localStorage`, device flash
and emu files are runner choices behind that boundary.

## 8. Coordinate space

One space for all pointer-like events, mirroring the framebuffer's
centre/radius discipline ([core/toy_fb.h:23](../core/toy_fb.h),
[CARTRIDGES.md:52](CARTRIDGES.md)):

- **Display milli-radius:** origin at display centre, +x right,
  **+y down** (screen direction — chosen so runner conversion is
  mechanical; carts wanting math axes negate `y` once, as renderers
  already negate when placing, e.g.
  [core/cart_fig8.c:117](../core/cart_fig8.c)). Unit = 1/1000 of
  the inscribed radius `r = min(w,h)/2`. On-glass range is roughly
  ±1000 per axis (±1414 at square corners, which round glass clips);
  values clamp to ±1500 (`TOY_COORD_MAX`, provisional).
- **Conversion** from integer pixels happens once, in C, via
  `toy_px_to_milli(px, size)` using the same centre convention as
  `toy_cx` (`size/2`, [core/toy_fb.h:23](../core/toy_fb.h)):
  `milli = round((px - size/2) * 1000 / (size/2))` in 64-bit integer
  math, round-half-away-from-zero, then clamp. Browser and driver
  code pass *raw pixels + size* into glue (§10.3) — no JS or driver
  float scaling ever touches sim-bound values. Emu text scripts
  author milli values directly (App. A) and may also use an explicit
  `px@size` form converted host-side by the same helper linked into
  emu.
- Carts convert milli→`f64` with `f64_from_milli` (§5.4) at `apply()`
  time and store the result. A 240 vs 360 glass therefore yields the
  same event stream for the same *relative* gesture, up to the
  coarser pixel grid of the small panel — honestly different
  physical input, equally replayable once recorded (R6 is about
  recorded streams, not about two humans touching two kits alike).

## 9. Runner mappings

Universal rules first. **Caps gate:** a runner only enqueues event
kinds whose `TOY_CAP_*` bit the selected cart sets (§5.1); everything
else is runner-local (e.g. keys driving page chrome). **Chrome
never enters state:** cart switching, pause/play, warp/speed,
display mode, sleep/wake are runner actions — they exist as chrome
already in the browser ([site/app.js:176](../site/app.js)) and as
gestures in TOY-0 ([TOY-0.md:73](TOY-0.md)) — and must not be
delivered as cart events, or a replay would need to reproduce runner
UI state to reproduce hashes. **One writer:** on dual-core board
firmware ([TOY-0.md:76](TOY-0.md)) the input task only enqueues;
only the sim task calls `apply/step`, only at boundaries (§6.1).

### 9.1 Board — future live firmware (CST816 touch, QMI8658C-class IMU, GPIOs)

No such firmware exists yet (§1.2); this table is the contract its
drivers will be written against. Thresholds are provisional pending
kits (§1.3). Polling/IRQ cadence is a driver choice; queueing and
draining follow §6.2/§6.3 (sim never waits on I²C).

| Source (driver output) | Event | Notes |
|---|---|---|
| Touch contact appears (x,y px @240/360) | `POINTER_DOWN id=0` via `toy_pointer_event` | Rim test first (below) |
| Contact moves ≥1 px | `POINTER_MOVE id=0` | Coalesced per §6.3 |
| Contact ends / driver reports none | `POINTER_UP id=0` (last known pos) | Synthesised also on driver reset |
| Touch starting in outer rim band* | — (chrome) | Rim-swipe=warp, rim swipe L/R=prev/next cart per TOY-0 gestures; never queued |
| Accel tilt, quantised | `TILT x=ax_mg y=ay_mg` | See quantisation below; ≤1/frame |
| Shake detector fires | `SHAKE v=1..3` | Runner state machine + refractory (≈300 ms, provisional) |
| Face up/down (az sign, hysteresis) | chrome sleep/wake; `FACE` too iff `TOY_CAP_FACE` | Sleep policy itself: timebank study |
| Encoder detents since last frame | `ENCODER id=0 v=sum` | Summed, clamped ±32/frame (provisional) |
| GPIO button edge (debounced ≈20 ms) | `BUTTON_DOWN/UP id=n` | Debounce is runner-local; only debounced edges queue |
| Encoder push / long-press | `BUTTON_*` or chrome | Runner's fixed choice per button, documented in firmware |

\* Rim band: contacts starting with `max(|mx|,|my|)` beyond ≈830
milli (provisional) belong to chrome for their whole track. This *spatial*
partition is recommended over recognising "swipe" after the fact,
because events already forwarded to a cart cannot be un-applied —
buffering all pointer input until gesture classification would add
a frame+ of latency to every drag. Interior swipes are cart drags.
Owner confirmation requested (§15 Q1).

IMU quantisation (driver→event, integer only): with raw accel counts
at a configured full-scale (unknown until kits arrive), compute
milli-g by integer multiply/divide once in board code, apply a dead
band (≈15 mg) and emit `TILT` at most once per frame only when either
axis changed by ≥20 mg (both provisional). Gyro is not used by this
seam. Carts receive "gravity direction, coarsely" — enough for
tilt-gravity toys; attitude/seismometer fidelity is toy1-signals.

### 9.2 Browser — `site/app.js` over WASM glue

Canvases exist at 240 and 360 ([site/index.html:40](../site/index.html));
the same state renders to both ([site/app.js:54](../site/app.js)),
so either canvas may drive input with its own size as the conversion
reference.

| Browser source | Event | Notes |
|---|---|---|
| `pointerdown` on a canvas | `POINTER_DOWN` (glue: raw px,size) | `setPointerCapture`; px from `offsetX/Y` scaled by `canvas.width/rect.width` in JS *integers for pixel location only* — milli conversion stays in C (§8) |
| `pointermove` (captured, buttons≠0; hover iff cart asks*) | `POINTER_MOVE` | Coalesced §6.3; `getCoalescedEvents()` not used (runner variance) |
| `pointerup` / `pointercancel` / blur | `POINTER_UP` | Synthetic UP on blur/cart switch prevents stuck drags |
| `keydown/keyup` (when not typing in a control) | `KEY_DOWN/UP` per table below | `repeat` → `v=1`; `preventDefault` only for mapped keys |
| Mouse wheel over canvas (iff `CAP_ENCODER`) | `ENCODER v=sign steps` | Accumulate `deltaY` into detents runner-side |
| Param slider/select (generated, §10.4) | `SET_PARAM` via glue | `RESET_ONLY` → reselect-with-settings (§7.3) |
| `deviceorientation` (optional, permission-gated) | `TILT` | JS sends integer milli-deg-derived mg approximations to glue; off by default; absence must not break carts (emu script is the canonical tilt source) |

\* Hover moves (no button) let Julia-by-touch respond to a mouse
without press; on touch hardware hover does not exist. Carts must
treat MOVE-without-DOWN as cursor presence, and touch-only carts
remain fully usable — gate: runners send hover only if the cart sets
`CAP_POINTER`; carts distinguish drag via their own down-flag state.

Key table (JS `KeyboardEvent.key` → `TOY_KEY_*`): arrows→`LEFT/RIGHT/
UP/DOWN`; `a/A`→`A`, `b/B`→`B`, `x`→`X`, `y`→`Y`; space→`SPACE`;
`Enter`→`ENTER`; `Escape`→`ESC`; `+`/`=`→`PLUS`; `-`/`_`→`MINUS`.
WASD aliases for arrows are a *runner* choice (document in app.js);
carts see only canonical codes. Unmapped keys are page chrome.

### 9.3 Host emu — scripted / file input

The emu is the canonical scripted runner (no live devices): proof
mode replays `proof_inputs`; a new file-driven mode replays a text
script (grammar, Appendix A) for authoring proof scripts, rendering
interaction sheets, and replaying browser/board recordings. Text is
parsed host-side into `toy_proof_input_t` entries (using
`f64_dec`/`toy_pointer_event` from the linked core for any pixel or
decimal forms), then delivered through the identical apply path.

| Script line (App. A) | Event |
|---|---|
| `@step down <mx> <my>` / `move` / `up` | `POINTER_* id=0` |
| `@step keydown <NAME>` / `keyup` | `KEY_*` |
| `@step button <n> down|up` | `BUTTON_*` |
| `@step enc <delta>` | `ENCODER v=delta` |
| `@step tilt <mgx> <mgy>` · `shake <1..3>` · `face up|down` | `TILT/SHAKE/FACE` |
| `@step param <name> <value>` (value: int, enum name, or decimal) | `SET_PARAM` (name→index via descriptor; unknown name = hard error) |

Emu also gains record-out: file-driven and proof runs can echo the
*applied* stream in the same text grammar, which is how a proof
script is debugged and how §6.6 recordings are inspected.

### 9.4 Settings presentation per runner (generated, never per-cart code)

Browser: controls generated from `params` into the existing control
strip area ([site/index.html:18](../site/index.html)) — range
input for `I32/F64` (F64 slider steps are display quanta; the value
sent to glue is a decimal string formatted by JS from the slider's
integer position and the descriptor's decimal min/max/step so JS
float formatting never enters bits — implementing parcel must define
F64 slider quanta as integer thousandths of range), `<select>` for
`ENUM`. Emu: `param` script lines + a `toy_emu params <cart>` dump.
Board live firmware: a runner settings screen (rim-swipe to enter,
touch/encoder to adjust) is future firmware scope — flagged for
owner sequencing (§15 Q4); headless board proof needs no settings UI
(proof is defaults+script, §6.5).

## 10. Exact runner diffs implied

Sketches show the shape and insertion points against current lines;
the implementing parcels (§13) own final code.

### 10.1 Emu — `emu/main_emu.c`

All three modes keep current arities and output for scriptless
carts. In `proof` (today [emu/main_emu.c:54](../emu/main_emu.c)):

```c
void *st = xcalloc(cart->state_size);
if (toy_cart_check_input(cart)) { /* stderr + exit 2 */ }
cart->reset(st);
uint32_t cur = 0;
toy_apply_proof_at(cart, st, 0, &cur);          /* step-0 setup */
printf("TOY-RUN cart=%s state_size=%zu hash=fnv1a64", ...);
if (cart->proof_input_count)
    printf(" inputs=%u input_hash=%016llx", cart->proof_input_count,
           (unsigned long long)toy_hash_inputs(...));
printf("\n");
for (uint32_t s = 1; s <= cart->proof_steps; s++) {
    toy_apply_proof_at(cart, st, s, &cur);      /* NEW, no-op deaf */
    cart->step(st);
    /* hash reporting unchanged */
}
```

`run` ([emu/main_emu.c:94](../emu/main_emu.c)) gets the identical
two lines. `frames` ([emu/main_emu.c:70](../emu/main_emu.c))
tracks a global step counter across frames and applies proof entries
at their absolute steps, so contact sheets *show* the scripted
gesture (this is how interactive parcels produce the 240/360 visual
evidence [CARTRIDGES.md:63](CARTRIDGES.md) requires). New,
additive CLI (no existing arity changes):

```
toy_emu params <cart>                       dump param table
toy_emu play <cart> <script.txt> <steps> <report-every> [record-out.txt]
toy_emu frames ... (unchanged; uses proof script automatically)
```

`play` = `run` with file entries merged by `toy_apply_live` at their
steps (caps enforced; over-cap script = error, not silent drop, so
authored scripts are exactly replayable).

### 10.2 Board proof — `board/main_board.c`, `board/CMakeLists.txt`

`run_cart` ([board/main_board.c:21](../board/main_board.c)) gains
the same cursor + `toy_apply_proof_at(...,0,...)` after `reset` and
per-step call before `cart->step(state_buf)`, and prints the same
conditional `inputs=/input_hash=` fields — host and board lines stay
`diff`-comparable ([board/run_toy.sh:58](../board/run_toy.sh)).
Invalid input descriptors print `TOY-BAD-CART cart=<name>` and are
skipped like oversize states today
([board/main_board.c:57](../board/main_board.c)).
`board/CMakeLists.txt` gains exactly one reviewed source line,
`${TOY_ROOT}/core/toy_input.c`, beside the explicit cart lines
([board/CMakeLists.txt:29](../board/CMakeLists.txt)) — emu and
site pick the file up by glob ([tools/run.sh:19](../tools/run.sh),
[site/build.py:44](../site/build.py)) and need no build edit for
it. No drivers, no display code: this proves the seam headless on
the existing Pico before kits arrive (R5). The future *live* board
firmware is a separate binary/parcel (§13 P8), not an edit to the
proof runner's contract.

### 10.3 WASM glue — `site/wasm_glue.c`, `site/build.py`, checker

Glue keeps one cart/state ([site/wasm_glue.c:14](../site/wasm_glue.c))
and adds **runner-owned** (never in `toy_state`): a 64-entry live
queue with §6.3 coalescing, a proof cursor + proof-mode flag, and a
steps counter it already has (`toy_steps_done`,
[site/wasm_glue.c:18](../site/wasm_glue.c)).

`toy_wasm_step(count)` keeps its signature and cap
([site/wasm_glue.c:75](../site/wasm_glue.c)); internally each
sub-step becomes: *proof mode* → `toy_apply_proof_at` for
`steps_done+1`; *live mode* → on the first sub-step only, drain the
queue via `toy_apply_live`; then `step()`, `steps_done++`. Proof and
live sources are never merged in one run. `toy_wasm_select`
([site/wasm_glue.c:64](../site/wasm_glue.c)) additionally clears
queue/cursor, defaults to live mode, and applies step-0 proof
entries **only in proof mode** — live sessions must not receive
ghost proof gestures. New exports (added to the explicit list,
[site/build.py:19](../site/build.py)):

```c
uint32_t toy_wasm_input_caps(int index);
uint32_t toy_wasm_param_count(int index);
const char *toy_wasm_param_name(int index, uint32_t p);
const char *toy_wasm_param_label(int index, uint32_t p);
uint32_t toy_wasm_param_kind(int index, uint32_t p);
uint32_t toy_wasm_param_flags(int index, uint32_t p);
int32_t  toy_wasm_param_i(int index, uint32_t p, int which); /*0 min 1 max 2 def*/
const char *toy_wasm_param_f(int index, uint32_t p, int which); /* decimal str */
uint32_t toy_wasm_param_enum_count(int index, uint32_t p);
const char *toy_wasm_param_enum_name(int index, uint32_t p, uint32_t e);
uint32_t toy_wasm_proof_input_count(int index);
uint32_t toy_wasm_input_hash_lo/hi(int index);   /* for checker pin */
int  toy_wasm_set_proof_mode(int on);            /* checker only */
int  toy_wasm_queue_pointer(uint32_t type, uint32_t id,
                            int32_t px, int32_t py, int32_t size_px);
int  toy_wasm_queue_key(uint32_t type, uint32_t key, int32_t repeat);
int  toy_wasm_queue_encoder(uint32_t id, int32_t delta);
int  toy_wasm_queue_tilt(int32_t mgx, int32_t mgy);
int  toy_wasm_queue_set_param_i(uint32_t p, int32_t value);
int  toy_wasm_queue_set_param_f(uint32_t p); /* reads a JS-written
     decimal scratch buffer in WASM memory (see parcel notes), OR: */
int  toy_wasm_param_scratch(void); /* returns ptr JS fills, N-term */
uint32_t toy_wasm_pending(void);             /* diagnostics */
```

(String-in-WASM scratch for F64 decimals avoids JS float formatting;
final split between the two `param_f` rows is the WASM parcel's to
settle — the fixed contract is: decimals as strings into C, §7.1.)
Queue functions return 0 when dropped/cap-gated so JS can show
honest diagnostics.

Checker ([site/check_wasm.mjs:23](../site/check_wasm.mjs)): after
select, call `toy_wasm_set_proof_mode(1)` (and re-run the step-0
application inside glue on mode set), then its existing
`toy_wasm_step(1)` loop and hash compares are unchanged; it also
verifies `input_hash` against a pinned value per cart in
`expected-hashes.json` (new optional key; absent = expect 0 inputs,
preserving today's file for deaf carts,
[site/expected-hashes.json:1](../site/expected-hashes.json)).

### 10.4 Browser app — `site/app.js`, `site/index.html`

- Pointer/key/wheel listeners on both canvases (§9.2), attached once
  at init beside current chrome listeners
  ([site/app.js:176](../site/app.js)); handlers no-op unless the
  selected cart's caps include the kind. `selectCartridge`
  ([site/app.js:46](../site/app.js)) rebuilds generated param
  controls from the new WASM getters into a new container element in
  the control strip ([site/index.html:18](../site/index.html)),
  and synthesises `POINTER_UP` before switching.
- Tick loop ([site/app.js:99](../site/app.js)) unchanged except it
  already calls `toy_wasm_step(wholeSteps)` — glue drains on the
  first sub-step (§6.2). Status line may show pending/dropped counts
  for honesty; must never imply a recorded hash includes unapplied
  queue contents.
- Deep links: `?steps=` pre-stepping ([site/app.js:155](../site/app.js))
  stays live-mode with an empty queue (deterministic input-free prefix);
  a future `?proof=1` could load proof mode for sharing exact
  interactive states — named here, parcel's choice, not required.
- F64 slider quanta rule from §9.4 applies: JS formats decimals from
  integer slider positions only.

## 11. Worked examples (illustrative cartridge code)

### 11.1 Julia-by-touch — pointer + one live param

State carries cursor and Julia constant as `f64`s (effects, §6.4).
`apply` is short; `step` (escape-time refinement/progressive zoom —
the cart parcel designs the stepping cost) is untouched by input
plumbing. Descriptor uses designated style for the new fields while
the first eight keep today's positional form; equivalently, trailing
positional values work. (Mixing styles as below is illustrative; the
core parcel sets the house style in CARTRIDGES.md.)

```c
typedef struct {
    f64 c_re, c_im;      /* Julia constant, math axes */
    f64 scale;           /* display-milli -> plane factor */
    uint64_t steps;
    uint32_t touching;   /* 0/1 - derived from DOWN/UP, in state */
    uint32_t iters;      /* effective "iters" param */
    /* ... pixel working set / row cursor ... */
} julia_state;

static void julia_apply(void *st_, const toy_event_t *ev)
{
    julia_state *s = (julia_state *)st_;
    switch (ev->type) {
    case TOY_EV_POINTER_DOWN: s->touching = 1; /* fall through */
    case TOY_EV_POINTER_MOVE:
        /* display y-down milli -> math y-up plane, once */
        s->c_re = f64_mul(f64_from_milli(ev->x), s->scale);
        s->c_im = f64_mul(f64_from_milli(-ev->y), s->scale);
        break;
    case TOY_EV_POINTER_UP: s->touching = 0; break;
    case TOY_EV_SET_PARAM:
        if (ev->id == 0) { /* iters, declared 16..512 */
            int32_t v = ev->v; if (v < 16) v = 16; if (v > 512) v = 512;
            s->iters = (uint32_t)v;
        }
        break;
    default: break; /* ignore the rest */
    }
}

static const toy_param_t julia_params[] = {
    { "iters", "Iterations", TOY_PARAM_I32, 0, 16, 512, 128, 0,0,0, 0 },
};
static const toy_proof_input_t julia_proof[] = {
    {   0, { TOY_EV_SET_PARAM, 0, 0,0, 96, 0, 0 } },
    {  10, { TOY_EV_POINTER_DOWN, 0, -400, 120, 0, 0, 0 } },
    { 200, { TOY_EV_POINTER_MOVE, 0,  350,-260, 0, 0, 0 } },
    { 600, { TOY_EV_POINTER_UP,   0,  350,-260, 0, 0, 0 } },
};
/* descriptor: ..., julia_reset, julia_step, julia_render,
 * julia_apply, julia_params, 1, julia_proof, 4,
 * TOY_CAP_POINTER|TOY_CAP_PARAMS */
```

Proof story: defaults, `iters=96` at step 0, press-and-drag sweeps
*c* across the dial; hashes at `proof_report_every` now certify the
*same gesture* on host, Pico and WASM. Live, a hover/drag in the
browser and a finger on glass produce different streams (different
steps/positions) — both recordable and cross-replayable (§6.6).

### 11.2 Gravity sandbox — drag/fling + tilt + reset-only param

Sketch of the semantics the cart parcel refines: `DOWN` records grab
point + step in state; `MOVE` updates the aim point; `UP` spawns a
body at the grab point with velocity from (aim−grab)/(steps held),
computed in softfloat inside `apply(UP)` from stored step counts
(never wall time — carts already store step counters, e.g.
[core/cart_fig8.c:24](../core/cart_fig8.c)). `TILT` overwrites a
stored gravity vector (`f64`s from milli-g). Params: `bodies`
(`I32`, `RESET_ONLY`), `softening` (`F64`, live), `palette` (`ENUM`,
live, render-only effect but still stored in state so hashes are
unambiguous about what was shown — render reads state, as today).
Caps: `POINTER|TILT|SHAKE|PARAMS|KEYS` (keys: `SPACE` pause-sim is a
*cart* decision if defined so; runner pause remains chrome and is
distinct — carts must document which they mean; default: runner
pause only).

### 11.3 What a deaf cart writes

Nothing. No `apply`, no params, no script — fields zero. That is the
point of §12.

## 12. Migration: existing carts stay identical

**Source identity.** Descriptor fields are appended only (§5.3).
C11 partial initialisation zero-fills the rest, so fig8's and
pendulum's eight-field positional initialisers
([core/cart_fig8.c:135](../core/cart_fig8.c),
[core/cart_pendulum.c:144](../core/cart_pendulum.c)) — and the
ten T1 carts' ([WAVE-T1.md:12](WAVE-T1.md)) — compile unchanged
with `apply=NULL, param_count=0, proof_input_count=0, input_caps=0`.
Caveat to handle in the core parcel: the emu build uses
`-Wall -Wextra` ([tools/run.sh:17](../tools/run.sh)), which can
warn (`-Wmissing-field-initializers`) on partial initialisers. No
build uses `-Werror` today (emu [tools/run.sh:17](../tools/run.sh),
wasm [site/build.py:45](../site/build.py), board
[board/CMakeLists.txt:39](../board/CMakeLists.txt)), so warnings
are cosmetic; the parcel should either accept them temporarily or
add the one-line trailing-zeros edit to *merged* carts at lead's
discretion. Hash identity does not depend on that choice.

**Hash identity.** State structs, `state_size`, `reset/step` code and
the hash function ([core/toy_carts.c:18](../core/toy_carts.c)) are
untouched; the new per-step call is a no-op when
`proof_input_count==0`; output lines gain fields only when inputs
exist (§6.5). Therefore `run.sh check` streams (fig8
`d8fda0444c2da4ef`, pendulum `3a378e9b9e40055b` at 10 000 steps,
[RECORD.md:17](RECORD.md)), board `TOY-RUN/step/TOY-DONE` lines,
and `expected-hashes.json` pins
([site/expected-hashes.json:2](../site/expected-hashes.json)) are
all expected byte-identical across the seam core parcel.

**Verification of "no churn" (gate for parcel P1, §13):** capture
`toy_emu proof`/`run` streams + WASM checker output on the base
commit, re-capture after P1, `diff`/`cmp` (the check budget already
byte-compares repeat runs, [tools/run.sh:42](../tools/run.sh));
board budget for P1/P2 is lead-run per existing practice (this study
parcel does not run it). Any diff fails the parcel — there is no
"expected hash update" for deaf carts, ever, under this design.

**Sequencing vs T1 branches.** P1 touches `toy_cart.h` at the tail
of the struct plus two new files. T1 cart branches touch the
`extern` block ([core/toy_cart.h:37](../core/toy_cart.h)) and
registry ([core/toy_carts.c:5](../core/toy_carts.c)) — different
lines, so either merge order is low-conflict; lead sequencing per
[WAVE-T1.md:6](WAVE-T1.md) applies as usual.

**CARTRIDGES.md update (in P1).** Move input/settings from "seams
not yet parcel-ready" ([CARTRIDGES.md:76](CARTRIDGES.md)) into
the contract: add `apply()` rules mirroring `step()` rules
([CARTRIDGES.md:34](CARTRIDGES.md)), proof-script authoring under
descriptor fields ([CARTRIDGES.md:40](CARTRIDGES.md)), an
evidence line (script visible in 240/360 sheets), and fix the
sentence "`step()` … does not … read input" to keep meaning that
*step itself* is input-free while `apply()` owns input.

## 13. Wave T2 parcel breakdown

Small, single-owner parcels; order is dependency order. "Gate" uses
existing budgets ([CARTRIDGES.md:61](CARTRIDGES.md)); board budget
only where named and lead-run as today.

| # | Parcel (branch) | Owned files | Delivers | Gate |
|---|---|---|---|---|
| P1 | `toy2-input-core` | `core/toy_input.h`, `core/toy_input.c` (new), `core/toy_cart.h` (append + include), `core/toy_math.h` (two helpers), `board/CMakeLists.txt` (1 line), `docs/CARTRIDGES.md` (seam section) | Types, helpers (§5.4), validation; **no runner behaviour change** beyond linking | `check` streams byte-identical pre/post (§12); host sheets unchanged |
| P2 | `toy2-input-board-proof` | `board/main_board.c`, `emu/main_emu.c` (proof/run lines only) | Proof-script replay + conditional `TOY-RUN` fields (§10.1–2) | Board budget (lead) with only deaf carts: streams identical; keeps working when P6 lands |
| P3 | `toy2-input-emu` | `emu/main_emu.c` (`play`, `params`, frames scripting), `emu/` docs if any, App. A parser | Text scripts, record-out, interactive sheets (§10.1, §9.3) | `check` + a scripted replay fixture replays to same hashes twice |
| P4 | `toy2-input-wasm` | `site/wasm_glue.c`, `site/build.py` (EXPORTS), `site/check_wasm.mjs`, `site/expected-hashes.json` (optional keys) | Queue, proof mode, param getters (§10.3) | `tools/run.sh site` PASS; deaf pins unchanged |
| P5 | `toy2-input-site-ui` | `site/app.js`, `site/index.html` (param container), `site/style.css` (control layout only) | Pointer/keys/wheel + generated param UI (§10.4); needs P6 (or P3 fixture) to be visible — sequence after P4, verify with P6 | Site budget + manual browser check named in report |
| P6 | `toy2-julia` | `core/cart_julia.c` (new) + four registration touches ([CARTRIDGES.md:7](CARTRIDGES.md)) | First interactive cart: pointer + `iters` param + proof script (§11.1) | Full cart evidence ([CARTRIDGES.md:61](CARTRIDGES.md)) incl. board budget (lead) |
| P7 | `toy2-gravity` | `core/cart_gravity.c` (new) + registration | Drag/fling + tilt + params (§11.2), after P6 proves drag feel/latency | Full cart evidence |
| P8 | `toy2-board-live-input` | new `board/` driver + live-firmware files only (names at dispatch); **blocked on kit arrival** | CST816/IMU/(encoder) drivers → queue per §9.1, display loop, rim chrome | On-device: record session, replay in emu to identical hashes (R6 demo) |

Notes: P2–P5 can overlap once P1 merges (disjoint files except
P2/P3 both touching `main_emu.c` — sequence P3 after P2 or split
`main_emu.c` hunks by mode at dispatch). Parameterised *existing*
carts (e.g. exposing pendulum `g`/`dt` as params) are **not** part
of the seam: adding params changes a cart's state/reset and thus its
proof hashes; each such change is its own cart revision parcel with
new pins, owner's call, after P6 sets the pattern. Timebank
(warp/banking interactions with input latency) and checkpoint
format remain sibling studies; P8 must consume their decisions where
§6.7/§9.1 point.

## 14. Non-goals, risks, revisit triggers

- **Not this seam:** raw IMU/mic sample streams + FFT carts
  (toy1-signals/toy1-fft, [WAVE-T1.md:30](WAVE-T1.md)); audio
  output; wall-clock/timebank pacing (toy1-timebank); checkpoint
  *format* (toy1-checkpoint, §6.7 pins interfaces only); network/multiplayer
  input; text entry; per-cart bespoke settings screens on glass.
- **Latency feel** at high steps/frame is quantised by §6.2; if P6/P7
  feel laggy, first lever is cart `demo_steps_per_frame`, second is
  runner frame rate — not sub-step input, which would reopen §3.
- **Gesture misclassification** (rim rule, §9.1) can annoy until kits
  arrive to tune the band; the band is runner-side, so tuning never
  invalidates recorded cart streams (recordings store post-partition
  events).
- **Descriptor growth** is a compatibility commitment: v1 event
  type numbers and struct layout are forever once recordings exist
  (§6.6). New kinds append; `flags` stays 0 until a study spends it.
- **Revisit Option A** (`step(st,ev,n)`) only if a concrete cart
  needs whole-batch atomic reasoning that ordered `apply` cannot
  express (none in the current T2 list); migrating then is a
  mechanical `apply`-loop move into `step` prologues, scripts/hashes
  preserved if per-step semantics are kept identical.
- **RP2350 hardware-float trap** ([TOY-0.md:49](TOY-0.md)) applies
  to driver/glue scaling code too: coordinate and mg conversions on
  the producer path are integer-only in this design partly for that
  reason; any producer-side float must be treated as display-only.

## 15. Open questions for the owner (kept short)

1. **Rim chrome partition (§9.1):** reserve the outer rim band for
   runner gestures (switch/warp), interior always to the cart? (§9.1
   recommends yes; alternative — buffered gesture recognition —
   costs drag latency on every interaction.)
2. **Physical controls:** will an encoder/buttons actually be fitted
   ([TOY-0.md:155](TOY-0.md))? The seam supports them either
   way; only P8's scope changes.
3. **Session recording:** in T2, should live sessions be persistable
   (device storage / browser download, §6.6), or is compiled-in proof
   script + emu file replay sufficient until checkpoint storage lands?
4. **On-glass settings UI (§9.4):** browser/emu-generated controls
   first (P5), device settings screen deferred to P8-or-later — or is
   on-device tuning part of the desk-toy promise for the first
   interactive carts?

Everything else in this study (event layout, quantisation, proof
script placement, caps numbers) is presented as decidable by the
implementing parcels/lead without owner input; provisional constants
are marked where they appear.

## Appendix A — emu text script grammar (host-side only)

Line-oriented; `#` comments; blank lines ignored. Steps are absolute,
non-decreasing; multiple lines may share a step (file order = apply
order, ≤16/step enforced). Coordinates are display milli-radius
(§8) unless suffixed `px@240`/`px@360` (both numbers are then pixels
at that size, converted by `toy_px_to_milli`).

```
# julia smoke - converts 1:1 to proof entries; emu can emit C array
@0    param iters 96
@10   down -400 120
@200  move 350 -260
@600  up 350 -260
@600  keydown SPACE
@601  keyup SPACE
@700  tilt 120 -40
@800  param palette ember
@900  enc -3
```

Directives: `down|move|up <mx> <my> [id]`, `keydown|keyup <KEY>`,
`button <n> down|up`, `enc <delta> [id]`, `tilt <mgx> <mgy>`,
`shake <1..3>`, `face up|down`, `param <name> <value>` where value
is an integer, an enum name from the descriptor, or a decimal string
for `F64` (parsed via `f64_dec` host-side). Unknown cart param names,
bad steps, or cap violations are hard errors with line numbers.
Record-out (§9.3) writes this same grammar (milli form only), header
lines giving cart, `state_size`, total steps and `input_hash`.

## Appendix B — descriptor validation checklist (`toy_cart_check_input`)

1. `param_count ≤ TOY_MAX_PARAMS`; `param_count>0 ⇔ params!=NULL`;
   names non-empty and unique within the cart.
2. `I32/ENUM`: `i_min ≤ i_def ≤ i_max`; `ENUM`: `enum_names` has
   `i_max−i_min+1` non-NULL entries (indices are `v−i_min`… parcel
   fixes the convention; recommended: `i_min=0` required for ENUM).
3. `F64`: `f_min/f_max/f_def` non-NULL and parse finite via
   `f64_dec` with min ≤ def ≤ max (checked with library compares at
   first use on host; board checks non-NULL + parses at reset path).
4. `proof_input_count ≤ TOY_MAX_PROOF_INPUTS`; entries sorted by
   `step`, all `≤ proof_steps`; per-step count ≤16; event types in
   known range; `flags==0`; `SET_PARAM` ids `< param_count`.
5. `proof_inputs!=NULL || param_count>0 ⇒ apply!=NULL`;
   `apply==NULL ⇒ input_caps==0`.
6. Caps coherence: `TOY_CAP_PARAMS ⇔ param_count>0`; caps for kinds
   the script uses should be set (warning-level, not fatal —
   proof-only input kinds are legitimate, e.g. proving TILT on a
   runner without an IMU).





