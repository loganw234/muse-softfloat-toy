# Study: time layer — wall clock, banking, playback, morning report

Study parcel `toy1-timebank` (Wave T1, base `8f63c78`). Design only:
no code in this parcel changes. Code blocks below are illustrative
interface sketches for the T2 parcels named in §8.

Scope from [WAVE-T1.md](WAVE-T1.md): "Time layer: wall-clock
abstraction, pacing/banking buffers, morning-report data — unblocks
overnight banking, active hours, morning report." The concept of
record is [TOY-0.md](TOY-0.md) §3 layer 2: pacing ("K steps per
displayed frame, user warp incl. 'hold during my night hours'") and
banking ("the integrator keeps stepping while the display sleeps,
writing decimated trajectory samples to a ring buffer; playback at
wake is sped up"), under the honesty rule that stepped states are
exact and any interpolation between banked samples is drawing, never
a state — restated for cartridges in
[CARTRIDGES.md](CARTRIDGES.md) ("If interpolation is ever added for
banked playback, it belongs to the presentation layer and must be
labelled drawing, not a state").

**Recommendation in one paragraph.** Put one small wall-clock
interface in the *runner* (board/host/JS), readable only by a new
time-layer module that also lives outside `core/`. The time layer's
entire output to the sim is non-negative integer step counts plus
run/hold scheduling decisions. For banking, do not store dense full
states: store sparse full-state **keyframes** (default one per
60 s of banking) with their step indices and hashes, and reproduce
every intermediate state at playback by re-stepping — bit-identical
by the toy's own determinism gate, not an approximation. Literal
dense banking is sized in §3 and rejected on arithmetic; it remains
the documented fallback for a cartridge too slow to re-step at warp
speed, on microSD hardware only. The morning report is a time-layer
record built from keyframe hashes and step ranges; per-cartridge
"notable events" are deferred to an opt-in, non-hashed sidecar hook
so no existing proof hash moves.

## 1. What exists today (evidence)

The three-layer rule is already partially embodied; there is no time
layer yet. Precisely:

- **Sim core.** `core/toy_cart.h:21-35` defines the whole cartridge
  contract: `state_size`, `demo_steps_per_frame`, `proof_steps`,
  `proof_report_every`, and `reset`/`step`/`render` pointers.
  `step()` takes only `void *st` — there is no time, dt, or input
  parameter anywhere to hang wall time on, which is the correct
  shape and must stay so. `core/toy_cart.h:19` caps state at
  `TOY_MAX_STATE_SIZE` = 65,536 B. The state rules in
  [CARTRIDGES.md](CARTRIDGES.md) already forbid wall clocks in
  `reset()`/`step()` and native float/host-derived values in state.
- **Hash = identity.** `toy_hash_state` (`core/toy_carts.c:18-27`)
  is FNV-1a 64 over the raw state bytes; the header comment
  (`core/toy_cart.h:45-47`) states the gate: "identical on host,
  RP2040 and RP2350 for identical step counts." Pinned values live
  in `site/expected-hashes.json`. Anything that makes state bytes a
  function of *when* steps ran breaks this gate (§2.3).
- **Host emu** (`emu/main_emu.c`) has no clock at all: `frames` mode
  steps a fixed count per rendered frame (`emu/main_emu.c:84-86`,
  defaulting to `demo_steps_per_frame` at line 75), `run`/`proof`
  step exact totals. It is a step-count machine — a good host for a
  time layer, currently without one.
- **Board** (`board/main_board.c`) is a headless proof runner: it
  runs each cartridge's proof stream once (`board/main_board.c:63-64`)
  into a single static `TOY_MAX_STATE_SIZE` buffer (line 19) and
  idles. Its only time source is `sleep_ms` for USB-attach waiting
  and LED blink (lines 46-50, 67) — pacing of *I/O*, never of the
  sim. No RTC, no schedule, no persistence.
- **WASM/site** is the only runner with anything like a time layer,
  and it is already shaped correctly. `site/app.js:99-118` converts
  elapsed browser milliseconds into a fractional `stepDebt` at
  `demo_steps_per_frame × 30 fps × speed multiplier`, clamps the
  debt (250 ms elapsed cap, 10,000-step debt cap), and passes only
  an integer count to `toy_wasm_step(count)`, which itself clamps
  to 100,000 and loops `step()` (`site/wasm_glue.c:75-81`). Wall
  time is read in JS (`requestAnimationFrame` timestamps); the C
  core sees counts. This study generalises that pattern to all
  runners and adds banking/playback to it; it does not replace it.
- **Pacing fields.** `demo_steps_per_frame` is defined as "live
  pacing at a nominal 30 fps" ([CARTRIDGES.md](CARTRIDGES.md),
  Descriptor fields). Current values: fig8 = 4
  (`core/cart_fig8.c:138`), pendulum = 30
  (`core/cart_pendulum.c:147`). Hence demo rates of 120 and
  900 steps/s respectively at 1× — the basis of §3. There are no
  other pacing/schedule fields in any descriptor today, and this
  study proposes adding **none** to `toy_cart_t`: pacing policy is
  the time layer's, parameterised by the one existing field.
- **Diagnostics in state today.** Both cartridges carry a
  `uint64_t steps` counter (`core/cart_fig8.c:24`,
  `core/cart_pendulum.c:20`); fig8 also carries `tperiods`
  ("whole periods completed (display only)", `core/cart_fig8.c:26`,
  derived as `steps / 633` at line 99). There is no accessor API
  for these — a runner can only hash or render state. §5.3 builds
  the morning report without needing one in v1.
- **No step-cost measurements exist in-repo.** [CARTRIDGES.md](CARTRIDGES.md)
  asks parcel reports for "measured board step time," and
  [RECORD.md](RECORD.md) records host/board hash equality at
  125 MHz but no µs/step figure. Every step-time number in this
  study is therefore an explicitly labelled estimate (§4.2), and
  measuring it is the first T2 parcel.

## 2. Wall-clock abstraction

### 2.1 Placement rule

Three layers, per TOY-0 §3. This study pins down who may read time:

| Layer | May read wall clock? | Receives from time layer |
|---|---|---|
| 1. Sim core (`core/`) | **Never.** No clock calls, no time-typed values, no schedule enums in state | Integer step counts (via the runner's existing `step()` loop); implicitly, *whether* to run at all |
| 2. Time layer (new, per-runner dir, e.g. `time/` or inside each runner) | **Only reader.** Owns schedule, banking, playback warp, morning-report assembly | Clock readings (§2.2); settings via the `toy1-input` seam (§5.1); checkpoint store via `toy1-checkpoint` (§6) |
| 3. Presentation (display/JS UI) | May *display* clock-derived labels handed to it by layer 2 (e.g. "banked 02:14", "DRAWN — interpolated"); must not feed them back | States or labelled drawings + their step indices and hashes |

Concretely: code under `core/` must not include the clock header,
and the clock header must not be includable from `core/` (put it
outside `core/`, e.g. `time/toy_clock.h`, with per-runner
implementations in `board/`, `emu/`, and JS for the site). The
existing runners already demonstrate the boundary: emu passes
counts (`emu/main_emu.c:85`), board passes counts
(`board/main_board.c:28`), WASM glue accepts counts
(`site/wasm_glue.c:75-81`). The time layer slots *in front of*
those loops; no runner's core-facing call shape changes.

### 2.2 Interface sketch (illustrative, T2-owned)

Two distinct time concepts are needed, and conflating them is the
classic bug: **monotonic elapsed time** (for pacing — "33 ms have
passed, owe N steps") and **civil/wall time** (for scheduling —
"it is 23:40, inside the user's night window"). Pacing must use
only the monotonic source so that clock-setting, DST and NTP steps
can never inject or swallow sim steps. Scheduling uses only civil
time, and only to make run/hold decisions.

```c
/* time/toy_clock.h — runner-implemented. NEVER included from core/. */
#ifndef TOY_CLOCK_H
#define TOY_CLOCK_H
#include <stdbool.h>
#include <stdint.h>

/* Monotonic milliseconds since some runner-local epoch (boot).
 * Must never run backwards within a power cycle; wraps at 2^64.
 * The ONLY source pacing arithmetic may use. */
uint64_t toy_clock_mono_ms(void);

/* Civil time snapshot for scheduling and report labels. */
typedef struct {
    uint16_t year;        /* e.g. 2026 */
    uint8_t  month, day;  /* 1-12, 1-31 */
    uint8_t  hour, minute, second; /* local civil, see §7.2 */
    bool     valid;       /* false = unset/never set/lost on reset */
} toy_civil_t;
toy_civil_t toy_clock_civil(void);

/* Optional: set civil time (host push / touch entry). Runners
 * without a setter return false. Setting NEVER touches the sim. */
bool toy_clock_set_civil(const toy_civil_t *t);
#endif
```

The time layer proper (shared logic, runner-instantiated) then
looks like:

```c
/* time/toy_timebank.h — pure logic over toy_clock + settings.
 * Knows cartridges only through toy_cart_t fields (name,
 * state_size, demo_steps_per_frame) and a step/hash callback
 * pair supplied by the runner — it never calls step() itself
 * with anything but a count, and never writes state. */
typedef enum { TOY_TB_LIVE, TOY_TB_BANKING, TOY_TB_PLAYBACK,
               TOY_TB_HOLD } toy_tb_mode_t;

typedef struct {
    toy_tb_mode_t mode;
    uint32_t steps_due;      /* what the runner must step NOW    */
    bool     take_keyframe;  /* copy state out after stepping     */
    bool     present_banked; /* show banked/keyframe stream, §4.3 */
} toy_tb_decision_t;

/* Called once per runner tick (frame, loop iteration).
 * `steps_done_total` is the runner's monotone step counter for
 * the loaded cartridge (cf. site/wasm_glue.c toy_steps_done). */
toy_tb_decision_t toy_tb_tick(toy_tb_t *tb, uint64_t steps_done_total);
```

Note what is *absent*: no `double warp`, no seconds-since-epoch, no
schedule struct crosses into the runner's stepping loop — only
`steps_due`. Warp factors and schedules are layer-2-internal
arithmetic in native types, exactly as `site/app.js:107-116`
already keeps `stepDebt` in JS numbers and exports only
`wholeSteps`.

### 2.3 Why nothing time-shaped may enter state

State bytes are the toy's identity: hashed verbatim
(`core/toy_carts.c:18-27`), pinned across host/board/WASM
(`site/expected-hashes.json`), and — per the checkpoint seam —
"state bytes are the save file" ([CARTRIDGES.md](CARTRIDGES.md)).
If any wall-derived value (a timestamp, an elapsed-ms accumulator,
a warp factor, a "banked at" field) entered state, then:

1. The same step count reached at different wall times, in
   different timezones, before/after a clock-set, or under
   different user schedules would hash differently — host == board
   == WASM (`core/toy_cart.h:45-47`) would become unprovable in
   principle, not just failing in practice.
2. Proof streams would cease to be reproducible by the existing
   harnesses, which know only counts (`emu/main_emu.c:54-69`,
   `board/main_board.c:21-37`).
3. Checkpoints would smuggle one runner's clock into another's
   resume, making "re-derive bit-for-bit on a desktop" (TOY-0 §1)
   false.

The existing in-state `steps` counters (`core/cart_fig8.c:24`,
`core/cart_pendulum.c:20`) are the model: they count *steps*, the
one time-like quantity that is a pure function of the step
sequence. Sim-internal `dt` values are likewise decimal-string
constants set at reset (`core/cart_fig8.c:76`,
`core/cart_pendulum.c:82`) — functions of the cartridge, not of
the wall. Everything else time-shaped lives in layer 2 records
(keyframe index, morning report) *alongside* state bytes, never
inside them.

### 2.4 Per-runner clock sources

**Board with RTC — RP2350 kit.** TOY-0 §2 lists an "RTC chip (real
wall-clock for active-hours scheduling and event timestamps)" on
the Waveshare RP2350-Touch-LCD-1.85C. Implementation: monotonic =
Pico SDK `time_us_64()`/1000 (the board already links
`pico/stdlib.h`, `board/main_board.c:9`); civil = RTC chip over
I2C, read at schedule-evaluation cadence (≤1 Hz), cached between
reads. `valid` = RTC oscillator-stop flag clear *and* time has
been set at least once (a persisted "time is set" marker via the
`toy1-checkpoint` store, §6). Battery-backed RTC survives main
power loss; that is precisely the battery-vs-plugged fork in §8.

**Board without RTC — RP2040 kit.** TOY-0 §2 lists no RTC for the
RP2040-Touch-LCD-1.28-B, and the RP2040's on-chip RTC is not
battery-backed (it stops when the chip unpowers and has no
separate VBAT domain). Posture: monotonic works from boot; civil
starts `valid=false` on **every** power-on/reset and becomes valid
only when set — by USB host push (the proof firmware already has a
CDC channel and waits for host attach, `board/main_board.c:44-50`)
or by a future touch-entry settings screen (input seam, §5.1).
While invalid, the schedule engine must degrade to **always-active
live mode** (§7.1) — never guess a time, never hold. A warm reset
that preserves SRAM may optionally carry civil time + a boot-count
check forward, but v1 should treat *any* reset as unset: simpler,
and the failure direction (toy awake when it might have slept) is
benign. Consequence to state plainly: on the RP2040 kit,
"overnight banking by the clock" only works across nights where
power was continuous and time was set that power cycle — or the
schedule is driven by non-clock triggers (face-down = sleep,
pickup = wake, TOY-0 §3 layer 3), which the mode machine in §4.1
accepts as equivalent events.

**Host emu.** Monotonic = `clock_gettime(CLOCK_MONOTONIC)` /
platform equivalent; civil = `localtime_r`. The emu is also where
*schedule simulation* belongs for testing: a fake-clock mode (civil
time and mono advanced by a script, e.g. "simulate 22:00→07:00 in
60 s") lets every banking/playback path be exercised in CI without
waiting a night. This fake clock is a runner feature behind the
same `toy_clock_*` interface — the time-layer logic under test is
identical.

**WASM/site.** Already correct in shape: JS owns time
(`site/app.js:99-118`). Monotonic = the `requestAnimationFrame`
timestamp / `performance.now()`; civil = `new Date()` local time.
No new WASM exports are needed for pacing — `toy_wasm_step(count)`
(`site/wasm_glue.c:75`) is already the count-only seam. Two
browser realities bound the design: background tabs throttle or
suspend rAF, so real overnight banking in a tab is fictional
(the existing 250 ms elapsed clamp at `site/app.js:103` already
concedes this); and page reload loses the in-memory state
(`site/wasm_glue.c:14`). The site therefore gets *simulated*
nights (compressed schedule demo + playback + report rendering),
which also makes it the cheapest place to prototype the morning
report UI. Persisting site state (e.g. localStorage keyframes) is
possible later via new glue exports to copy state bytes out —
noted as T2-TB5 optional scope, not required for parity.

## 3. Banking buffer design — the arithmetic

### 3.1 Inputs (measured/derived from current source)

State sizes are compile-time struct sizes, exposed as
`state_size` in each descriptor (`core/toy_cart.h:25`) and printed
by every proof stream (`emu/main_emu.c:59`,
`board/main_board.c:25-26`). Derived here from the struct
definitions (`f64` is an 8-byte bit pattern; all fields 8-byte, so
no padding beyond the structs as written):

fig8 (`core/cart_fig8.c:19-27`, `TRAIL=288`):

```text
x[3][2] + v[3][2]      = 12 f64        =    96 B
w1, w0, dt             =  3 f64        =    24 B
steps, tperiods        =  2 uint64     =    16 B
trail[3][288][2]       = 1728 f64      = 13,824 B
total sizeof(fig8_state)               = 13,960 B  (13.63 KiB)
```

pendulum (`core/cart_pendulum.c:14-22`, `PTRAIL=220`):

```text
th[2][2] + w[2][2]     =  8 f64        =    64 B
dt, g                  =  2 f64        =    16 B
steps                  =  1 uint64     =     8 B
tip[2][220][2]         = 880 f64       =  7,040 B
total sizeof(pend_state)               =  7,128 B  (6.96 KiB)
```

Two facts drive everything below:

1. **~99% of both states is trail/tip history** (fig8:
   13,824/13,960 = 99.0%; pendulum: 7,040/7,128 = 98.8%). The
   integrative core is 136 B / 88 B. But the trail is hashed state
   and rendering reads it, so a stored state that omits or
   reconstructs the trail differently is a *different state* with
   a different hash. Stripping trails from banking (or moving them
   out of state) is therefore **rejected**: it would re-pin every
   proof hash and split "state" into two classes. Banking stores
   `state_size` bytes verbatim, or stores a step index and
   recomputes (§3.3). (A future cartridge *contract revision*
   could separate presentational history from integrative state;
   that is a CARTRIDGES.md-level owner decision, not this study's
   to make — see §8.)
2. **Demo rates differ 7.5×.** nominal 30 fps ×
   `demo_steps_per_frame`: fig8 4×30 = **120 steps/s**; pendulum
   30×30 = **900 steps/s**. One 8 h night (28,800 s) at 1× demo
   rate is therefore 120×28,800 = **3,456,000 steps** (fig8) or
   900×28,800 = **25,920,000 steps** (pendulum).

Per-entry overhead for a banked sample is small but counted where
it matters: a keyframe record needs step index (8 B) + hash (8 B)
+ wall label/flags (~16 B) ≈ **32 B header** per entry, i.e.
+0.2% on a fig8 state and +3% on a hypothetical 1 KiB state.
Rendered-frame banking is dismissed immediately: one 240×240
RGB565 frame is 115,200 B (360×360: 259,200 B) — 8–36× larger
than the states that produce it, resolution-bound, and
non-resumable. States strictly dominate frames as the banked unit.

### 3.2 Literal banking (store full states) — sized, then rejected as the default

Bytes for one 8 h night = samples × state_size, samples =
28,800 s / interval. (Headers excluded; add 32 B/sample — e.g.
+26 MiB at frame rate, +0.9 MiB at 1/s, negligible below that.)

| policy | samples/8 h | fig8 13,960 B | pendulum 7,128 B | 1 KiB state | 64 KiB cap |
|---|---|---|---|---|---|
| every step (fig8 120/s, pend 900/s) | 3.46 M / 25.9 M | 48,261 MiB ≈ **44.9 GiB** | 184,749 MiB ≈ **172 GiB** | rate-dependent | rate-dependent |
| every frame (30/s) | 864,000 | **11,503 MiB** | **5,873 MiB** | 844 MiB | 54,000 MiB |
| 1/s | 28,800 | **383 MiB** | **196 MiB** | 28.1 MiB | 1,800 MiB |
| 1 per 10 s | 2,880 | 38.3 MiB | 19.6 MiB | 2.81 MiB | 180 MiB |
| 1 per 60 s | 480 | 6.39 MiB | 3.26 MiB | 0.47 MiB | 30.0 MiB |

Sample arithmetic, shown once: fig8 at 1/s = 28,800 × 13,960 B =
402,048,000 B = 383.4 MiB. Fig8 every step = 3,456,000 × 13,960 B
= 48.2×10⁹ B ≈ 44.9 GiB — and that is merely to *store* a night
whose live computation produced each of those states anyway.

Against the hardware budgets (TOY-0 §2):

- **RAM ring.** RP2040 264 KiB SRAM holds 264×1024/13,960 = 19 raw
  fig8 states before *anything else*; reserving the 240²
  framebuffer (115,200 B), the live state, and 32 KiB misc/stack
  leaves ≈108 KiB ≈ **7 fig8 states ≈ 7 s at 1/s**. RP2350 520 KiB
  with the 360² framebuffer (259,200 B) leaves ≈227 KiB ≈ 16 fig8
  states. A RAM ring is a seconds-to-minutes jitter buffer (useful
  in front of SD/flash writes), never an overnight store. (Note
  the framebuffer figures are the full-square allocations runners
  use today — `emu/main_emu.c:81`, `site/wasm_glue.c:15` — a
  device renderer may do better with banded flush, but banking
  must not depend on that.)
- **Onboard flash, 16 MB on both kits**, minus firmware/SDK:
  assume a generous 14 MiB bank budget. That holds 14×2²⁰/13,960
  = 1,051 fig8 states → an 8 h night needs interval ≥ 27 s; for
  pendulum 2,059 states → ≥ 14 s. At a 4 MiB budget: ≥ 96 s /
  ≥ 49 s. A 64 KiB-cap cartridge fits only 224 states in 14 MiB →
  ≥ 129 s interval, and its 1/min night (30 MiB) **exceeds total
  flash at any budget**. Flash literal-banking also inherits erase
  wear (§6): appending 383 MiB/night at 1/s is not a flash workload
  at all.
- **microSD (RP2350 kit only**, "banked trajectories … stop being
  storage-constrained", TOY-0 §2): 1 GiB holds fig8 at 1/s for
  2³⁰/13,960/3600 ≈ 21 h, and even frame-rate fig8 for ≈ 0.7 h.
  SD makes 1/s literal banking feasible *for current-size carts* —
  on one of the two kits, with a card present. It cannot be the
  cross-kit default.

Verdict: every-step and every-frame banking are absurd on all
targets; 1/s is SD-and-RP2350-only; flash forces ≥ ~30–130 s
intervals depending on state size, at which point the stored
stream is already so sparse that playback must synthesise the
in-between *somehow* — which is exactly what §3.3 does honestly
by re-stepping instead of interpolating.

### 3.3 Recommended: sparse keyframes + re-stepping ("virtual banking")

The toy's own thesis dissolves the storage problem. A trajectory
is a pure function of (cartridge, starting state bytes, step
count): the proof harnesses exist to demonstrate exactly this
(`emu/main_emu.c:94-111`, `board/main_board.c:21-37`,
`core/toy_cart.h:45-47`). Therefore a night need not be *stored*
to be *replayed exactly* — it can be recomputed, and the
recomputation is verifiable against stored keyframe hashes. This
is not interpolation and not an approximation; every replayed
frame is a stepped state, satisfying the TOY-0 honesty rule in its
strongest form.

**Banking record** (layer 2, per night/session):

```c
/* Illustrative. Persisted via the toy1-checkpoint store (§6);
 * the state payload format itself is checkpoint's, not ours. */
typedef struct {
    uint64_t step_index;   /* steps since cartridge reset        */
    uint64_t state_hash;   /* toy_hash_state at this keyframe    */
    toy_civil_t wall;      /* label only; may be invalid         */
    /* followed by state_size raw state bytes (checkpoint blob) */
} toy_keyframe_hdr_t;
```

**Policy.** While banking, the device steps at 1× demo rate (or a
user warp, §4) with the display asleep, and appends a keyframe
every `K` steps, where `K` is chosen per cartridge as
`K = demo_steps_per_frame × 30 × keyframe_interval_s`. Default
`keyframe_interval_s = 60`: fig8 K = 7,200 steps, pendulum
K = 54,000 steps → 480 keyframes/night → **6.39 MiB (fig8) /
3.26 MiB (pendulum)** per night including full states — fits a
14 MiB (even 8 MiB: interval 48 s/24.5 s) flash budget for current
carts, trivial on SD. The final state of the night is just the
last keyframe plus ≤ K−1 re-steps.

Two degenerate settings of the same policy are worth naming,
because owner decisions (§8) select between them:

- **Start-only banking** (`interval = ∞`): store one keyframe at
  banking start (13,960 B) + the target step count. Overnight the
  CPU may sleep entirely; at wake the device fast-steps the whole
  night (§4.2 sizes the wait). Storage ≈ one state; cost = wake
  latency and zero overnight observability (no intermediate hashes
  to show in a report until recomputed — which playback does
  anyway). This is the battery-friendly form.
- **Dense-keyframe banking** (`interval = 1–10 s`, SD only): for a
  cartridge whose warp re-stepping cannot keep up (§4.2),
  keyframes get dense enough that playback becomes mostly
  *reading*. At 1/s this converges to §3.2's literal banking —
  same format, same code path, just a smaller K. That the two
  "designs" are one policy knob is the point: T2 implements
  keyframes+restep once, and density is tuned per cartridge/kit
  from TB1's measured step costs.

**RAM structure.** Live: the existing single state allocation
(`board/main_board.c:19`, `site/wasm_glue.c:14`) plus optionally
*one* playback state (so replay never disturbs the live head —
TOY-0's "core 1 renders from immutable snapshots" is the same
idea). Worst-case extra RAM = one `state_size` ≤ 64 KiB; on RP2040
that argues for reusing the live buffer during pure playback
(live head = last keyframe, restorable by definition). The
keyframe *index* (480 × 32 B ≈ 15 KiB/night headers) can live in
flash/SD with the blobs; RAM caches only the current segment.

**What if the user switches cartridge mid-night?** Banking is
per-cartridge: a switch closes the current bank (final keyframe +
report record) and starts a new one from `reset()`. Storage
accounting in §3.2 is per active cartridge; a multi-cart night
multiplies keyframe counts but each bank is independently
replayable. Autoswitching carts (e.g. the T1 carousel) bank fine
under the same rule — their switching logic is inside `step()`,
hence inside the deterministic stream.

## 4. Playback and pacing algorithm

### 4.1 Mode machine

Four modes, evaluated by layer 2 on each runner tick from
(monotonic now, civil now, settings, input events):

```text
LIVE      stepping at warp x demo rate, rendering live head.
          --(enter night window | face-down)--> BANKING
BANKING   stepping at 1x (or CPU-asleep under start-only
          banking), display off/asleep, keyframes appended.
          --(leave night window | pickup | tap)--> PLAYBACK
PLAYBACK  presenting the banked interval at catch-up warp (§4.3)
          while the live head continues (or is already final).
          --(caught up | user skips)--> LIVE (+ morning report
          shown on entry, §5)
HOLD      stepping paused, display shows a frozen STEPPED state
          (its hash labelled). Entered only explicitly (user
          pause; cf. site/app.js play/pause) or when unable to
          sustain even banking (thermal/power — future).
```

Schedule evaluation needs civil time only at minute granularity;
evaluate at ≤1 Hz and on wake events. All transitions are layer-2
decisions; the core observes only their step-count consequences.

### 4.2 Can live sim keep up? Can playback? (labelled estimates)

There is **no measured µs/step in this repository** (§1). The
only in-repo bound is qualitative: proof runs should take
"roughly under three minutes on RP2040" ([CARTRIDGES.md](CARTRIDGES.md))
for proof horizons of 2,000 (fig8) and 1,000 (pendulum) steps —
far too loose to derive a rate. So the table below is
**ESTIMATES ONLY**, spanning plausible soft-float-binary64 step
costs on a 125 MHz Cortex-M0+ (the board proof ran at 125 MHz per
[RECORD.md](RECORD.md)); T2-TB1 replaces it with measurements
before any policy default is frozen. Read it as the *shape* of
the decision, not as data:

| assumed step cost (ESTIMATE) | max sustained steps/s | fig8 live need 120/s | pendulum live need 900/s | full-night restep CPU time (fig8 3.456 M / pend 25.92 M steps) |
|---|---|---|---|---|
| 50 µs/step | 20,000/s | 167× headroom | 22× headroom | 2.9 min / 0.36 h |
| 200 µs/step | 5,000/s | 42× | 5.6× | 11.5 min / 1.44 h |
| 1 ms/step | 1,000/s | 8.3× | 1.1× (marginal!) | 57.6 min / 7.2 h |

Decision rules that follow *whatever* TB1 measures:

- **Live slower than real time** (sustained steps/s < demo rate):
  the toy must not silently accumulate unbounded step debt — the
  site already models the honest posture with its debt clamps
  (`site/app.js:103,111`). Layer 2 caps debt at a small bound
  (e.g. 2 s worth), drops the excess, and the *presentation* shows
  actual achieved rate (the site shows fps; device shows step rate
  in its status/hash corner, TOY-0 §3). Sim time then runs slower
  than wall time, stated, never hidden by interpolation. A
  cartridge in this class is also a [CARTRIDGES.md](CARTRIDGES.md)
  violation-in-spirit ("verify the slowest target can sustain"
  `demo_steps_per_frame`) — its descriptor, not the time layer,
  is what gets fixed.
- **Banking** at 1× has the same budget as live; a cartridge that
  cannot sustain 1× cannot bank a full night in a night. It banks
  what it can (keyframes are still exact), and the morning report
  states steps achieved vs the 1× expectation (§5.2).
- **Playback warp ceiling** = sustained steps/s ÷ demo steps/s.
  E.g. (ESTIMATE row 2) fig8 could replay at up to ~42×, pendulum
  ~5.6×, by pure re-stepping. If the user's desired warp (§4.3)
  exceeds the ceiling, layer 2 first densifies future keyframes
  (needs SD headroom, §3.3), and for the current bank presents at
  the achievable warp — again stated, not faked. Interpolated
  frames may *additionally* be drawn between stepped frames for
  smoothness, labelled as drawing (§4.4); they never substitute
  for steps in accounting.

### 4.3 Catch-up playback arithmetic

Replaying an 8 h bank in `T` minutes of viewing = warp
`W = 480/T`: T=60 → 8×, T=30 → 16×, T=10 → 48×. At 30 fps
presentation, the viewer sees `1800·T` frames; each presented
frame corresponds to `demo_steps_per_frame × W` sim steps
(fig8 at 16×: 64 steps/frame; pendulum at 16×: 480 steps/frame).
Playback between keyframes is a budgeted restep: per presented
frame, layer 2 issues exactly that many steps to the *playback*
state (seeded from the segment's keyframe), renders it, and every
keyframe boundary **verifies** `toy_hash_state` against the stored
keyframe hash — a mismatch means storage corruption or a
nondeterminism bug, and playback must stop and say so rather than
present unverified states as banked truth. This verification is
nearly free (480 hashes/night over ≤14 KB states) and turns the
morning replay into a nightly self-test of the determinism gate.

Scrubbing (jump to arbitrary banked step `S`): load nearest
keyframe ≤ S, restep ≤ K−1 steps (≤ 7,199 fig8 / 53,999 pendulum
at default K — at the 200 µs ESTIMATE, ≤ 1.4 s / 10.8 s worst-case
seek latency; at 50 µs, 0.36 s / 2.7 s). If that latency is
unacceptable, the knob is smaller K on SD (§3.3), not a different
design. A "highlights" playback (only segments the report flags,
§5.3) is just scrub+play composed by layer 3.

### 4.4 Interpolation posture

With re-stepping, interpolation is *unnecessary* for correctness
at any warp the CPU can sustain: every presented frame can be a
real stepped state (warp just steps more per frame). Interpolation
would only ever smooth *sub-frame* motion at low presentation
rates. If it is ever added, the existing rules already govern it
and this study adds nothing: presentation layer only, labelled
drawing, never a state ([CARTRIDGES.md](CARTRIDGES.md); TOY-0 §3
honesty rule), and the on-screen hash must then show the hash of
the nearest *stepped* state with a drawn-frame indicator, never a
hash of drawn pixels. Recommendation: **do not build it in T2**;
revisit only if a specific cartridge looks bad at achievable warp.

### 4.5 Pacing mechanics (live), generalised from the site

The site's loop (`site/app.js:99-118`) is the reference
implementation of layer-2 pacing and should be ported in spirit to
emu/board rather than reinvented: accumulate
`debt += elapsed_mono_ms × demo_steps_per_second × warp / 1000`
in layer-2 native arithmetic, clamp both elapsed (250 ms) and
debt, issue `floor(debt)` steps. Two changes for the general
design: (a) debt accounting must use the monotonic clock only
(§2.2) — rAF timestamps already are monotonic in practice; (b) the
clamps become policy: elapsed clamp = "ignore hitches/sleeps
longer than X" (a laptop lid-close must not demand an hour of
catch-up steps), debt clamp = the slower-than-real-time honesty
bound of §4.2. Warp is a user setting (rim-swipe dial, TOY-0 §3;
site speed slider `site/app.js:184-186`); its *value* never enters
state (§2.3) — only the step counts it produces do work.

## 5. Schedule model and morning report

### 5.1 Where active-hours config lives

**Not designed here.** Settings storage, editing UI, and
deterministic injection of user input are the `toy1-input`
study's seam ("Input & settings seam", [WAVE-T1.md](WAVE-T1.md));
this study only names the interface it needs from that seam:

```c
/* Consumed by layer 2; owned/persisted by the input/settings
 * seam. Times are local civil minutes-of-day (§7.2). */
typedef struct {
    uint16_t active_start_min;  /* e.g. 07:00 = 420            */
    uint16_t active_end_min;    /* e.g. 23:00 = 1380; < start
                                   means window crosses midnight */
    uint16_t warp_x100;         /* live warp, 100 = 1.00x       */
    uint16_t playback_warp_x100;
    uint16_t keyframe_interval_s; /* §3.3 knob, 0 = start-only  */
    bool     banking_enabled;
} toy_schedule_t; /* shape illustrative; field ownership = input seam */
```

Requirements on that seam from the time layer's side: read at
≤1 Hz without allocation; changes take effect at the next
schedule evaluation (never retroactively — a mid-night edit
shortens/lengthens *future* banking only); defaults must be sane
with no settings ever written (banking on, active 07:00–23:00 is a
*product* default — owner call, §8); and on runners without
persistence (site today), settings are session-local, which is
acceptable because site nights are simulated anyway (§2.4).

### 5.2 Morning report — data model (v1, no cartridge changes)

The report is a layer-2 record assembled when BANKING ends,
rendered by layer 3 (device screen on pickup, emu stdout/file,
site panel). v1 fields, all sourced without touching `core/`:

```c
/* Illustrative, layer-2 owned, persisted with the bank (§6). */
typedef struct {
    char     cart_name[16];     /* toy_cart_t.name               */
    uint64_t steps_at_start;    /* first keyframe step index     */
    uint64_t steps_at_end;      /* final step index              */
    uint64_t steps_expected_1x; /* demo_rate x banked wall secs  */
    uint64_t hash_start;        /* keyframe hashes: the proof    */
    uint64_t hash_end;          /*   the night is re-derivable   */
    uint32_t keyframe_count;
    bool     playback_verified; /* §4.3 hash chain checked       */
    toy_civil_t wall_start, wall_end; /* labels; may be invalid  */
    uint8_t  end_reason;  /* SCHEDULE / PICKUP / POWER / FULL §7 */
} toy_morning_report_t;
```

Rendered in prose the device/site way, e.g. (illustrative):
"fig8 · banked 22:58→06:31 · steps 412,000 → 3,868,000 of
3,868,000 expected at 1× · final hash `9f…` · 480 keyframes,
replay verified ✓ · playback 16× ≈ 30 min". The two hashes are
the substance: anyone can re-derive the night on desktop from
`(cart, hash_start's state, step range)` and check `hash_end` —
the report is a *claim with its own proof attached*, in the
project's idiom. `steps_expected_1x` vs actual exposes
slower-than-real-time banking honestly (§4.2). Wall labels are
display strings; if `valid=false` the report says "clock unset —
banked by face-down/pickup" instead of inventing times.

### 5.3 Notable events — opt-in without breaking proof hashes

"Notable events/diagnostics cartridges record in state" (parcel
brief) conflicts head-on with §2.3 if read naively: a new
"events" struct *inside* hashed state changes `state_size` and
every byte after it, re-pinning all of
`site/expected-hashes.json` and invalidating recorded board proof
streams. Three routes, in order of preference:

1. **v1 (recommended): no per-cartridge events.** The report
   (§5.2) plus keyframe hashes plus deemed-interesting *step
   ranges* computed by layer 2 during playback restep — layer 2
   can hash/sample states as it re-steps and flag e.g. hash-stable
   stretches, without any cartridge cooperation. Existing
   in-state counters (`steps`; fig8 `tperiods`) already let a
   report say "≈5,460 periods elapsed" (3,456,000/633) *if* read — but there is
   no accessor, and `tperiods` is derivable from step count
   (`steps/633`, `core/cart_fig8.c:99`), so even that needs no
   core change: layer 2 knows the step range.
2. **v2 opt-in sidecar (own T2 parcel, owner-gated):** an
   *optional* descriptor addition — e.g. a nullable
   `summarise(const void *st, toy_diag_t *out)` callback plus a
   schema version — where `toy_diag_t` is a fixed small POD
   (counts/extrema as integers/f64 bit patterns) that layer 2
   samples at keyframes and stores **outside** state, in the bank
   record. Cartridges that don't opt in get v1 reports.
   Non-negotiables: the sidecar is never hashed into
   `toy_hash_state`, never fed back into `step()`, and adding the
   (null) field to `toy_cart_t` must be done once, deliberately,
   knowing descriptor layout is *not* hashed (only state bytes
   are, `core/toy_carts.c:18-27`) — so proof hashes survive;
   what changes is every descriptor literal gaining a field,
   a one-line-per-cart mechanical edit owned by that parcel.
3. **Rejected: event fields inside state.** Even "harmless"
   counters appended to a state struct change its size and hash
   stream, couple reporting richness to the determinism gate, and
   invite wall-shaped data across the §2.3 line. If a phenomenon
   *is* the simulation (e.g. a divergence counter a cartridge's
   own rendering needs), it belongs in state by the cartridge's
   own design — as `tperiods` already is — not by the time
   layer's request.

## 6. Power loss and checkpoint interplay

**Not designed here either:** the persisted state+header format,
flash layout, and versioning are the `toy1-checkpoint` study
("Checkpoint format & versioning (state bytes + header, flash
layout), unblocks power-loss resume", [WAVE-T1.md](WAVE-T1.md));
today's posture is "state bytes are the save file" with
versioning "deliberately undefined" ([CARTRIDGES.md](CARTRIDGES.md)).
The time layer is a *consumer* of that format — a keyframe (§3.3)
is precisely a checkpoint blob plus layer-2 header — and imposes
these requirements on it:

- **Append/ring friendliness with bounded erases.** At default
  policy a night appends 480 blobs ≈ 6.4 MiB (fig8). The layout
  must support append-and-roll without erasing per keyframe;
  worst-case erase budget should be stated by checkpoint (target:
  ≤ 2 full-region erase cycles/night, i.e. the bank region is
  ≥ 2 nights deep or SD-backed, else flash wear dominates the
  design — 10⁵-erase-cycle flash erased in full nightly is a
  ~century problem, erased per-keyframe at 480/day is a
  ~7-month one; the layout decides which).
- **Atomicity at blob granularity:** a power cut mid-write may
  lose at most the tail keyframe, detectable via the stored hash
  (§4.3 verification doubles as corruption detection).

What the **time layer must persist** (its own small record,
checkpoint-stored or alongside): current mode + bank identity
(cartridge name, bank start step index, keyframes written,
wall labels), the schedule-relevant "civil time was set" marker
for RTC validity (§2.4), and the last morning report (§5.2) until
superseded. What it must **recompute, never persist**: step debt,
warp phase, playback position *within* a segment (recompute by
restep from the segment keyframe), expected-steps accounting
(recompute from persisted bank header + clock at wake), and of
course every stepped state itself.

Resume semantics per runner on power return: **RP2350+RTC** —
clock still valid; layer 2 re-evaluates the schedule, loads the
newest valid keyframe as live head, resumes BANKING or enters
PLAYBACK-with-report (`end_reason=POWER` for the interrupted
bank; the gap in wall labels is shown, the step stream has *no*
gap — banking resumes from the keyframe, and the missed wall
hours simply never had steps, which the report's
expected-vs-actual field exposes). **RP2040 (no RTC)** — civil
invalid → always-active LIVE from newest keyframe (§2.4); the
interrupted bank is closed as POWER and its report offered once
time is set or via USB readback. **Emu** — overnight process death
is equivalent (bank = files); **site** — reload loses session
state (`site/wasm_glue.c:14`), accepted for simulated nights;
optional localStorage keyframes are T2-TB5 scope.

One hard rule crosses both studies: the checkpoint header (theirs)
and keyframe/morning-report records (ours) may contain wall labels
and step indices; the state payload must remain byte-identical to
live state. The moment persistence writes *into* state bytes, §2.3
is violated from the storage side.

## 7. Failure modes

### 7.1 Clock unset or wrong

- **Unset** (`valid=false`: RP2040 every boot, RP2350 before first
  set / RTC battery dead): schedule engine behaves as
  always-active. Banking can still be entered/exited by
  non-clock events (face-down/pickup, explicit user action), and
  reports omit wall labels rather than fabricate them (§5.2). The
  sim is entirely unaffected — by construction (§2.3), a clock
  fault can cost convenience, never correctness. This is the
  load-bearing payoff of the layer split and should be stated as
  an acceptance test in T2: *fuzz the clock (unset, jumping,
  backwards) through a full bank/playback cycle; proof hashes
  must be identical to the clock-free run at equal step counts.*
- **Wrong but valid-looking** (user set 14:00 at 02:00): toy banks
  during the user's day. Mitigations are product-level: show civil
  time wherever the schedule is shown; on host/USB attach, offer
  re-sync (host runners always have a trustworthy clock). No
  technical guard can distinguish "wrong" from "night-shift user"
  — and §5.2 reports make the behaviour legible after the fact.
- **Clock steps mid-bank** (set/sync during BANKING): pacing is
  monotonic-only (§2.2) so no steps are injected/swallowed;
  schedule re-evaluates at next ≤1 Hz tick; keyframe wall labels
  may be non-monotone — labels, explicitly not ordering keys
  (ordering is by `step_index`, always).

### 7.2 DST / timezone posture (recommendation)

**Store and compare schedule in naive local civil time
(minutes-of-day); embed no timezone/DST database on device.** The
device RTC holds local time; a host sync pushes local time.
Consequences, accepted deliberately: on a spring-forward night
the banked wall interval is 7 h of labels for 8 h of monotonic
banking (or the reverse in autumn, including a repeated hour of
labels) — harmless because banking volume is governed by steps
and keyframe counts, labels only annotate. Travel across zones =
user/host re-sets the clock, same as a wrong-clock event (§7.1).
The alternative (UTC + offset + DST rules) buys label accuracy
the product never uses — nothing in playback, reporting or proof
depends on absolute time — at the price of a tz database on a
264 KiB device and a new class of "schedule fired an hour off"
bugs. Host emu and site inherit the OS/browser local time for
their civil source, so all runners agree on naive-local by
construction. If a future feature ever needs absolute instants
(e.g. correlating with external sensor logs — the signals study's
territory), it should add a UTC *label* field then; the schedule
comparison stays naive-local.

### 7.3 Buffer/storage exhaustion

Bank region full mid-night (flash budget consumed, SD removed or
full, emu disk full — same policy shape, runner-detected):
**never block or drop steps.** Escalation, in order: (1) double
the keyframe interval for the remainder of the bank (halves write
rate; playback restep segments double — degraded seek, exact
states); repeat up to start-only density; (2) if even the *next*
single keyframe cannot be written, close the bank cleanly
(`end_reason=FULL`, report generated from what exists), keep
stepping LIVE-dark (display stays asleep; the toy is still a sim,
just no longer recording), and surface "bank storage full" in the
morning report and on next wake. Overwriting the *oldest*
keyframes of the current bank (ring-within-night) is rejected as
default: it silently destroys the start of the night, the exact
thing the morning report's `hash_start` attests to; if the owner
prefers "keep the most recent N hours" semantics, that is a
settings-visible policy choice for T2, and the report must then
state the truncation. A failed/corrupt keyframe write is handled
by §4.3's hash verification at playback: truncate the bank at the
last verified keyframe, say so, never present past corruption.

### 7.4 Smaller modes, named for T2 test lists

- **Cartridge switched during PLAYBACK:** playback bank is closed
  (report kept); live head is the new cartridge from `reset()`.
  The old bank remains replayable until storage reclamation —
  reclamation policy: only on new-bank space pressure or explicit
  user clear, never silently on switch.
- **Reset() by user during BANKING:** ends the bank
  (`end_reason` needs a USER value — added to §5.2's enum in T2),
  starts fresh. Old bank retained as above.
- **Monotonic wrap:** 2⁶⁴ ms ≈ 5.8×10⁸ years; use unsigned
  subtraction throughout and stop worrying. (A 32-bit ms counter
  would wrap in 49.7 days — the sketch in §2.2 is 64-bit
  deliberately; runners with 32-bit hardware ticks must extend
  them in the runner, not leak wrap into layer 2.)
- **Step-counter width:** runner-side totals (`toy_steps_done`,
  `site/wasm_glue.c:18`) and in-state `steps` are 64-bit; at
  900 steps/s continuous that is 6.5×10⁸ years. Keyframe
  `step_index` is 64-bit to match. The 32-bit proof parameters
  (`proof_steps`, `core/toy_cart.h:30`) are run-lengths, not
  indices — unaffected.

## 8. T2 parcel breakdown and owner decisions

Parcels are small, single-concern, and ordered so each lands
green alone. "Owned files" = files the parcel may create/edit;
everything else is read-only to it. All parcels inherit the
fences of this study: `core/` cartridge math untouched unless
named; board budget only where named.

| # | Parcel | Owned files | Depends on | Acceptance sketch |
|---|---|---|---|---|
| T2-TB1 | **Step-cost measurement.** Host timing print in emu (`run` mode reports steps/s + µs/step); board proof adds a timing line per cartridge (cycles/step at stated clock). Replaces §4.2 ESTIMATES with measured table in this doc. | `emu/main_emu.c`, `board/main_board.c`, `docs/study-timebank.md` (§4.2 table only) | — | Measured µs/step for every registered cart, host + RP2040; hashes unchanged (timing is runner-side) |
| T2-TB2 | **Clock seam (host first).** `time/toy_clock.h` per §2.2; emu impl (real + scripted fake clock); build wiring. No behaviour change yet. | `time/toy_clock.h`, `time/clock_emu.c`, emu build files in `tools/run.sh` (compile lines only) | — | `tools/run.sh check` green; fake-clock demo prints scripted civil times |
| T2-TB3 | **Timebank engine (host).** Mode machine §4.1, pacing §4.5, keyframed banking to files, verified playback §4.3 in emu (new `bank`/`replay` modes); consumes schedule struct with compiled-in defaults until `toy1-input` lands. | `time/toy_timebank.{h,c}`, `emu/main_emu.c` (new modes only) | TB1 (for default K), TB2; soft-depends `toy1-checkpoint` file format — until it lands, keyframe blob = header + raw state bytes, format versioned locally | Scripted night (fake clock): bank 8 h fig8+pendulum, replay at 16×, all keyframe hashes verify; clock-fuzz run reproduces clock-free proof hashes at equal step counts (§7.1 test) |
| T2-TB4 | **Morning report v1.** Report record §5.2 assembly + emu text rendering; no cartridge changes. | `time/toy_timebank.{h,c}` (report part), `emu/main_emu.c` (report print) | TB3 | Report for scripted nights incl. FULL/POWER end reasons; desktop re-derivation check (start blob + step range → `hash_end`) in test |
| T2-TB5 | **WASM/site parity.** JS clock (real + simulated-night control), playback + report panel; optional state-bytes export in glue for localStorage keyframes. | `site/app.js`, `site/index.html`, `site/style.css`, `site/wasm_glue.c` (exports only, optional part) | TB3/TB4 semantics | Simulated night in browser replays with verified hashes vs `site/expected-hashes.json` methodology; no change to live-mode behaviour |
| T2-TB6 | **Board bring-up.** Board `toy_clock` impls (RP2040: mono + USB-set civil; RP2350: + RTC chip when kit in hand), timebank engine integrated into board runner, keyframes to checkpoint store/SD. | `board/main_board.c`, `board/clock_board.c`, `board/CMakeLists.txt` | TB2–TB4, `toy1-checkpoint` landed, display layer exists (TOY-0 layer 3) | Board overnight-style scripted run: bank→power-cycle→resume→replay verifies; host==board hashes throughout; board budget named for this parcel only |
| T2-TB7 | *(Optional, owner-gated)* **Diagnostics sidecar.** §5.3 route 2: nullable `summarise` descriptor field + two pilot cartridges. | `core/toy_cart.h`, `core/toy_carts.c` (if needed), pilot `core/cart_*.c`, `time/*` sampler | TB4 + owner decision #6 | Proof hashes byte-identical before/after (descriptor-only change); report shows pilot diagnostics |

Sequencing note: TB1 should land first and alone — §3.3's default
keyframe interval, §4.2's warp ceilings and TB3's defaults are
all provisional until it reports. If TB1 shows a cartridge class
near the 1 ms/step estimate row, revisit default K (denser, SD)
for that class before TB3 freezes policy.

### Owner decisions (genuine; none inferable from code)

1. **Battery vs always-plugged** (carried from TOY-0 §8, now with
   time-layer consequences priced): battery → start-only banking
   (§3.3) with CPU asleep overnight and a wake-time fast-step
   costing, per §4.2 estimates, minutes-to-hours of CPU before
   playback completes (mitigable by playing back *while*
   catching up — playback only needs segments in order);
   always-plugged → literal overnight stepping, keyframes written
   live, zero wake latency. The mode machine supports both; the
   *default* and the product story differ.
2. **Primary kit.** RP2350 (RTC + microSD + 520 KiB) makes every
   feature in this study work as described. RP2040-only means:
   clock-unset after every power loss is normal operation (§2.4),
   non-clock triggers carry the schedule, and flash caps keyframe
   density (§3.2) — a reduced but coherent product. Time-layer
   code is identical; expectations are not.
3. **How time gets set** on first boot / RTC-less hardware: USB
   host push (cheapest, exists via CDC), touch entry (needs
   input seam UI), or "no clock schedule until set" with
   face-down/pickup only. Recommendation: host push + unset
   sentinel; touch entry deferred.
4. **Playback ambition:** lean-back catch-up (8 h in ~30 min at
   16×), quick skim (48×, ~10 min — step budget 48× demo rate,
   RP2040-feasibility per TB1), or full scrub-anywhere (§4.3 seek
   latencies apply)? Sets default warp and whether K must shrink
   on SD. Interpolation remains unrecommended regardless (§4.4)
   unless the owner wants sub-frame smoothing *labelled as
   drawing*.
5. **Timezone posture:** adopt §7.2 (naive local, no DST db —
   recommended) or invest in UTC+rules. Pure product-simplicity
   call; zero sim impact either way.
6. **Morning-report richness:** hash/step-range report v1 (§5.2,
   free, TB4) or fund the diagnostics sidecar (TB7) for
   per-cartridge "notable events"? Also: are reports kept beyond
   one night (history depth = storage policy, §7.3 interacts)?
7. **Full-bank semantics:** escalate-and-close (recommended,
   §7.3) vs ring-overwrite-oldest ("always keep the last N
   hours"). Affects what "overnight" promises the user.
8. *(Flagged, not required now)* **Trail/state split.** §3.1's
   99%-trail observation means a future cartridge-contract
   revision separating integrative state from presentational
   history would shrink keyframes/checkpoints ~100×. It would
   also re-pin every hash and rewrite every cartridge — a
   deliberate, owner-called migration if ever, explicitly not
   smuggled in by banking.





