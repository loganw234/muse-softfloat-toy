# Cartridge guide — the parcel-ready contract

One cartridge is one self-contained deterministic simulation and one
renderer. A new cartridge should enter the emulator, board proof and
WebAssembly site by adding:

1. `core/cart_<name>.c`, defining `const toy_cart_t toy_cart_<name>`;
2. one `extern` line in `core/toy_cart.h`;
3. one registry line in `core/toy_carts.c`;
4. one source line in `board/CMakeLists.txt` until that file learns
   to glob `core/cart_*.c` deliberately.

It must not require edits to the emulator, board runner logic, WASM
glue or website. If it does, that missing seam belongs in its own
study/parcel first.

## State rules (non-negotiable)

- State is one POD struct containing only bit-pattern integers
  (`f64`, `uint32_t`, `uint64_t`, fixed arrays). No pointers, heap,
  native `float`/`double`, booleans with unspecified padding bytes,
  or host-derived values.
- State must fit `TOY_MAX_STATE_SIZE` (64 KiB) and require no more
  than 8-byte alignment. Hosts zero the allocation before `reset()`;
  `reset()` still memsets the whole struct so every padding byte is
  defined before hashing.
- All state floating-point goes through `toy_math.h` wrappers over
  muse-softfloat. Native arithmetic, `<math.h>`, `rand()` and wall
  clocks are forbidden in `reset()` and `step()`.
- Constants come from decimal strings via `f64_dec()` at reset.
  Hand-typed IEEE bit patterns are allowed only for structural
  constants whose bits are the point (zero, sign, masks, infinities,
  named payload patterns) and must say so.
- `step()` advances one fixed internal dt and does bounded work.
  It does not allocate, print, read input, or branch on platform.
  Input reaches a cartridge only through the optional `apply()`
  callback at step boundaries (see "Input & settings seam");
  sensor samples reach it only through the optional `feed()`
  callback at the same boundaries (see "Signal feed seam").
  `step()` itself stays input-free. `apply()` and `feed()` obey
  the same rules as `step()`: softfloat only, no allocation, no
  I/O, no wall clock, bounded work, writes only its own state.
- `render()` is `const` in effect as well as signature. It may use
  native `double` only to place already-computed state on screen;
  a value converted for display must never flow back into state.

## Descriptor fields

- `demo_steps_per_frame`: live pacing at a nominal 30 fps. Choose it
  from what a viewer should see, then verify the slowest target can
  sustain it; report measured board step time in the parcel result.
- `proof_steps` / `proof_report_every`: the standard host/WASM/board
  comparison run. Pick a horizon long enough to leave the initial
  transient and exercise the phenomenon, short enough for a routine
  board gate (aim under roughly three minutes on RP2040).

## Input & settings seam

Design and rationale: [study-input.md](study-input.md) (section 5
is the interface, section 6 the determinism contract). This is
the short contract summary.

- Six descriptor fields are appended after `render`: `apply`,
  `params`, `param_count`, `proof_inputs`, `proof_input_count`,
  `input_caps`. Existing positional initialisers zero-fill them,
  so a cart that sets none of them is a *deaf cart*
  (`apply == NULL`) and its proof hashes are unchanged.
- Runners convert every input source into one 32-byte
  integer-only `toy_event_t` (`core/toy_input.h`) and quantise it
  to step boundaries: an event assigned to step *s* is applied
  after step *s−1* and before step *s*; step 0 is the
  post-`reset()` setup slot. Events within a step apply in FIFO
  order; at most `TOY_MAX_EVENTS_PER_STEP` (16) apply before one
  step and any excess **carries** to the next step boundary in
  order (never dropped or reordered at drain). Drops happen only
  on a full 64-entry live queue at enqueue time; proof/replayed
  runs must be authored under the caps. Hashes are taken only
  after `step()`, never between `apply()` and `step()`.
- Event kinds: pointer (milli-radius coordinates, +y down,
  converted from raw pixels in C by `toy_pointer_event`), keys,
  buttons, encoder, tilt, shake, face, `SET_PARAM`, and `DIAL`
  (integer milliturn delta, IMU-derived turn-the-device gesture;
  producer lands later). `input_caps` declares what the cart
  listens to; runners queue only those kinds. `apply()` must
  ignore unknown types and out-of-range ids.
- Settings are a per-cart parameter table (`toy_param_t`: I32,
  ENUM, or F64 with decimal-string bounds/defaults). Values enter
  state only as `SET_PARAM` events (`v` for I32/ENUM, IEEE bits
  in `bits` for F64, built by `toy_param_event`, never from a
  host float). `reset()` means defaults; a runner applies user
  settings in param-index order as step-0 events. The cart clamps
  to the declared range and stores only effects in state, so the
  state hash covers inputs.
- Interactive proof runs replay the descriptor's compiled-in
  `proof_inputs` script at exact step indices on host, board and
  WASM. Scriptless carts print byte-identical `TOY-RUN` lines;
  scripted carts add `inputs=`/`input_hash=` so diffs prove the
  same script ran. Author the script to exercise the phenomenon,
  keep it within the per-step cap, and use emulator frame sheets
  (proof entries apply at absolute steps) for visual evidence.
- Integer event values become softfloat state via
  `toy_f64_bits_from_i32` / `toy_f64_bits_from_milli`
  (`core/toy_input.h`), never native conversion. Runners keep
  owning their step loops; the shared helpers
  (`toy_apply_proof_at`, `toy_apply_live`, `toy_apply_queue`)
  only apply. `tools/input_test.c` is a runnable example of the
  whole seam with a minimal listening cart.

## Signal feed seam

Design and rationale: [study-signals.md](study-signals.md)
(section 4 is the seam, section 7 the labelling rule; SIG-1,
landed in Wave T2). Raw sensor streams are **not** events:
they enter through this separate optional feed. This is the
short contract summary.

- Two descriptor fields are appended **after `state_version`**
  (the WAVE-T2 tail order): `feed` and `sig_spec`
  (`toy_sig_spec_t` in `core/toy_signal.h`: device, proof kind,
  scenario, sample rate, samples-per-step, seed - integers
  only). Both zero/NULL means no signal need (a *signal-deaf*
  cart; proof hashes unchanged). A set `feed` requires a full
  spec and vice versa; `toy_cart_check_signal` enforces it at
  select on every runner. Rates bind statically - the source
  must match device/channels/rate exactly, no resampling in
  v1 - and one step's frame is bounded
  (`samples_per_step <= TOY_SIG_MAX_SAMPLES_PER_STEP`).
- At every step boundary the order is **feed -> apply ->
  step**: the runner polls exactly `samples_per_step` frames
  (a fixed, index-derived count, never clock-derived) and
  calls `feed()` before that boundary's input events and
  before `step()` integrates; there is no step-0 feed, the
  first feed precedes step 1. `feed()` copies plain int16
  sensor counts into POD state (conversion to softfloat is
  the cart's job, through `toy_math.h`/`toy_f64_bits_*`); the
  source itself is host-owned runner state, never hashed.
  Hashes/checkpoints stay post-`step()` only.
- Proofs are synthetic by construction in v1: every runner
  (emu proof/run/frames, board proof, WASM `step`) instantiates
  the descriptor's seeded synthetic spec through the shared
  helpers (`toy_sig_proof_source_init` + `toy_feed_step` in
  `core/toy_cart.h`), so a feed cart's proof stream is
  host == board == WASM like any other. Only scenario 1
  exists (IMU desk, MIC 440 Hz tone). Signal carts add
  `sig=/device=/scenario=/seed=/rate=/per_step=/stream=` to
  their `TOY-RUN` line; deaf lines are byte-identical.
  `toy_sig_proof_source_init` is the single source-creation
  point a future live driver swaps - the poll/feed loop does
  not change - and a live run is never a proof run. Underrun
  or end in a proof **fails the run**; samples are never
  zero-filled or repeated into a proof.
- Labelling is non-negotiable (study section 7): embed a
  `toy_sig_label_t` in state, `toy_sig_label_reset` it in
  `reset()` (kind `NONE` = visible "NO SIG", never a fake zero
  trace) and `toy_sig_label_feed` it in `feed()`, so stream
  kind, origin kind and `stream_id` sit inside the hashed
  bytes. `render()` shows the provenance (`SYNTH`, `REPLAY-*`,
  `LIVE`, `NO SIG`). `stream_id` for synthetic sources is the
  parameter-block hash `toy_sig_param_stream_id`; runs are
  comparable only if kind **and** id match.
- The turn-the-device gesture's producer is in core:
  `toy_dial_t` + `toy_dial_feed`/`toy_dial_feed_event`
  (`core/toy_signal.h`) integrate IMU gyro-z into `TOY_EV_DIAL`
  milliturn deltas with an integer accumulator (sub-milliturn
  remainder kept, full turns never wrap). Embed it in state
  to derive dial effects from your feed, or let a future live
  runner own it and queue the events to `apply()`.
- Checkpoints: the source position is host-owned and not in
  `TOYCKPT1`, so `toy_emu save`/`load` currently **refuse**
  feed carts loudly rather than resume unfed; a restore
  parcel can later fast-forward a fresh synthetic source by
  header steps x `samples_per_step`. Board firmware lists
  `core/toy_signal.c` explicitly for on-device proof
  instantiation. `tools/sigcore_test.c` is a runnable example
  of the whole seam with a minimal IMU-consuming cart.

## Rendering rules

- Read `fb->w`, `fb->h`, `toy_cx/cy/r`; never assume 240 or 360.
- Draw in centre/radius fractions. Keep labels inside the inscribed
  circle's safe area and check them at both sizes.
- Trails belong in state if they are simulation history; purely
  cosmetic fading of the framebuffer is presentation and must not be
  fed back into the simulation.
- If interpolation is ever added for banked playback, it belongs to
  the presentation layer and must be labelled drawing, not a state.

## Checkpoints

State bytes are the save file, wrapped in the `TOYCKPT1` image:
a 64-byte little-endian header (magic, envelope version, cart
name, `state_size`, `state_version`, runner step count, the
proof FNV hash, and header/payload CRC-32s) followed by the
state verbatim. The format is specified in
[study-checkpoint.md](study-checkpoint.md) section 2 and
implemented once in `core/toy_ckpt.{h,c}` for every runner;
`toy_emu save`/`load` is the host reference and
`tools/ckpt_test.c` pins the bytes (golden header, rejection
matrix, resume-continuity). A checkpoint taken on any runner
must resume bit-exact on every other.

Rules that follow for cartridge authors:

- `state_version` is the payload-layout version in the
  descriptor (last field; existing cartridges are version 0).
  Any state-layout change MUST bump it: field
  added/removed/reordered/resized, trail length changed, a
  constant folded into state changed meaning, `sizeof` changed
  for any reason, or `step()` computing something different
  from the same bytes. Loaders reject version mismatches
  loudly and never migrate or guess; a forgotten bump is a
  silent fork of every saved history.
- Because the payload is verbatim, the struct's byte layout
  must mean the same thing on every target: order fields by
  descending alignment (8-byte, then 4, 2, 1), group same-width
  fields, and rely on `reset()`'s whole-struct memset to keep
  any padding 0x00 (study section 3). If a change alters
  `sizeof` or any field offset, that is the bump above.
- Cartridge names are capped at 15 ASCII characters (the
  header's 16-byte name field needs its NUL), enforced by
  `tools/ckpt_test.c` over the registry.
- Everything `step()` will ever read must live in the state
  bytes at every step boundary - an input or setting that
  affects stepping from outside state silently voids resume.
  Do not embed a step counter the runner already keeps; the
  header carries the runner's count.

## Time layer, banking & morning report

Design and rationale: [study-timebank.md](study-timebank.md)
(sections 2–5 are the interface and the arithmetic). Owner
posture ([WAVE-T2.md](WAVE-T2.md)): always plugged in; time and
zone offset are set manually (no network time); unset is a
first-class state; RP2350 is primary but its RTC waits for the
kit. This is the short contract summary.

- **Placement rule (enforced by construction).** Wall time is
  runner-owned (`core/toy_clock.h`): `toy_clock_now()` returns
  int64 UTC seconds + zone offset minutes + a valid flag, and
  `toy_clock_mono_ms()` is the only pacing source (clock-setting
  can never inject or swallow steps). Nothing time-shaped may
  enter cartridge state — state is hashed verbatim, so a
  wall-derived byte would fork host/board/WASM hashes. The
  header refuses to compile without a runner-side opt-in
  (`TOY_CLOCK_ALLOW`, defined only in `emu/`, `board/`, site
  glue, host tests), does not include `toy_cart.h`, and no
  function in it touches state. The time layer's whole output
  to the sim is integer step counts. Implementations are
  per-runner: host env/CLI (`emu/clock_emu.c`), JS-provided
  imports (`site/wasm_glue.c`), and an unset stub on board
  (`board/clock_board.c`) until the RP2350 kit lands. Unset
  disables schedule features (helpers degrade to always-active)
  and never blocks simulation. Warp is parameterised
  (`warp_x100`): **defaults live 1.00×, playback 16×** (an 8 h
  bank in ~30 min), keyframe interval 60 s of demo time.
- **Banking is sparse keyframes + re-stepping, never dense
  states and never interpolation.** A `TOYBANK1` bank
  (`core/toy_bank.{h,c}`) is a 96-byte header + a 32-byte-per-
  keyframe index (step index, state hash, wall *label*) +
  complete `TOYCKPT1` checkpoint images — a keyframe is a
  checkpoint, and wall labels live beside state bytes, never
  inside them (ordering is by step index, always). Playback
  (`toy_emu replay`, `toy_bank_verify_replay`) seeds from the
  first keyframe and re-steps to each later one, requiring the
  state hash to match at every keyframe: every replayed frame
  is an exact stepped state, and a mismatch stops playback
  loudly. Budget is fixed at 8 MiB per bank (480 fig8 keyframes
  at the 60 s default = ~6.4 MiB; wider spacing for larger
  states — writers refuse past-budget counts). Host modes:
  `toy_emu bank <cart> <steps> <keyframe-every> [file]`,
  `replay <file>`, `report <file>`, `clock`.
- **Morning report v1 needs no cartridge changes**
  (`toy_morning_report_t`): cart, step range, keyframe
  count/spacing, start/end hashes (the claim carries its own
  re-derivation proof), wall span + expected-at-1× steps when
  the clock was set (shortfalls print honestly), end reason,
  and whether replay verified the hash chain. Per-cartridge
  "notable events" are deliberately not in v1 — anything
  reporting-shaped inside hashed state would re-pin every
  proof hash (study section 5.3).
- What still waits: board RTC + manual-set UI and on-device
  banking (kit), flash/SD bank storage, schedule-driven
  live/bank/playback modes on device and site banking UI —
  the formats and verification above are runner-independent,
  so those parcels add plumbing, not format changes.
  `tools/clock_test.c` pins the layer host-side, including
  the determinism firewall (identical step streams hash
  identically under unset/set/jumping clocks).

## Web mock device

The live site (`site/`) is a mock of the physical toy, used to
rehearse driver-UX decisions before the kits exist: one live
state presented on one round panel of selectable size, driven
by the producers a real driver would own. It is a UI driver
and timing model over the **unchanged** sim core — it does not
simulate panel hardware, and neither display size nor browser
pacing can touch sim state.

- **Panel model.** Presets 240 (1.28″ RP2040), 360 (1.85″),
  466 (1.75″ AMOLED), 480 (1.46″), or a custom square size
  clamped to 64..1024 px. The canvas backing store is exactly
  panel px (CSS scales the glass to fit) behind a round mask.
  Switching size only re-renders the same state — no reset, no
  step — and the always-visible step/state-hash line proves
  continuity. The only size cap in the WASM runner is its
  static framebuffer (`TOY_WASM_MAX_FB` 1024 in
  `site/wasm_glue.c`, 2 MiB of RGB565); the render path itself
  is size-parametric (`toy_fb_t` w/h, centre/radius drawing).
- **Pointer producer.** Unchanged: presses inside the glass
  circle queue `POINTER_*` as raw integer panel pixels; C
  converts to milli-radius. Where a press lands fixes its
  kind for the whole drag (see dial).
- **Dial producer** (`TOY_EV_DIAL`, id 0, integer milliturns,
  clockwise positive; only whole milliturns are emitted and
  sub-milliturn remainders stay in the producer): mouse wheel
  over the device, with `deltaY` normalised to pixels
  (deltaMode lines ×16, pages ×400) and accumulated in
  milli-pixels at **4800 px of travel per turn** (one emitted
  milliturn consumes exactly 4800 milli-px); and bezel
  rim-drag — a press outside the glass circle turns the
  device, the unwrapped angle about the centre accumulating
  at **2π/1000 rad per milliturn**. The page shows the
  accumulated total.
- **Tilt producer** (`TOY_EV_TILT`, id 0). A 2D pad whose knob
  marks the direction things fall, in milli-g in the wire's
  display space: `x` right, **`y` down** (carts flip to
  sim-up where needed; the pinned gravity proof's
  `TILT (300, 400)` is "toward display lower-right"). Each
  axis is `round((2·(pos/extent) − 1) · 1000)`, clamped
  ±1000; events emit on change only (deduped by milli value
  in the producer). "Level" sends (0, 0).
- **Settings.** Controls are built from the selected cart's
  param table through the glue introspection exports
  (`toy_wasm_param_*`): I32 slider+number, ENUM select, F64
  slider+number over the declared decimal bounds (there is no
  bool kind; a 0/1 ENUM covers it). A change queues
  `SET_PARAM` via `toy_wasm_queue_param` (I32/ENUM in `v`;
  F64 as the IEEE bits of the number the control shows — the
  session log records the bits, so replays are exact; proofs
  still build F64 from decimal strings via `toy_param_event`).
  `RESET_ONLY` params are labelled, held pending, and queued
  in param order on the next reset so they land in the step-0
  slot; user-set live values re-queue the same way, so
  settings persist across Reset. Untouched params are never
  queued: declared defaults and compiled-in step-0 proof
  settings are exactly what an untouched session runs. Carts
  without params show "no settings". Producers a cart does
  not advertise in `input_caps` are dimmed, and the glue
  refuses their events regardless.
- **Perf overlay.** Measured browser ms/step (exponential
  rolling average over `toy_wasm_step` calls) and steps
  behind the requested speed (requested − executed, which is
  where the dropped-past-cap pacing debt shows). Labelled on
  the page: browser timing only — board rates are records,
  not scaled from this. No board numbers are invented here.

### TOYSESSION1 session format (normative)

The glue records every **drained live event** — recorded when
`toy_apply_queue` applies it, not when it was queued, so
carried events appear at the boundary that really applied
them. `toy_wasm_session_log()` returns the text; the page's
"Download session" saves it verbatim. Proof-script entries
are not recorded (they are compiled into the cart and replay
themselves); a replay therefore runs the named cart exactly
as a proof run does (same proof script, same synthetic feed)
and additionally applies the logged events at their steps,
sharing the per-step cap after proof entries, exactly as the
live drain did.

```
TOYSESSION1 cart=<name>
<step> <type> <id> <x> <y> <v> <bits>
...one line per drained live event, in drain order...
[TRUNCATED dropped=<n>]
END <total_steps_at_export>
```

- LF line endings, single spaces, no trailing spaces; the
  final `END` line ends with LF.
- `<step>` is the absolute step the event applied *before*
  (runner steps completed + 1 at drain time), unsigned
  decimal. Events at one step appear in FIFO drain order.
- `<type>`, `<id>` unsigned decimal (`TOY_EV_*` values);
  `<x>`, `<y>`, `<v>` signed decimal; `<bits>` the u64 as
  unsigned decimal. `flags` is not recorded (producers always
  queue 0 in v1; replay writes 0).
- The header resets on every cart select/reset. A session
  never spans carts.
- The buffer is capped at 512 KiB (roughly 13 000 events).
  Past the cap, event lines stop and are counted; export then
  inserts `TRUNCATED dropped=<n>` before `END`. A truncated
  session replays exactly only up to its last recorded event.
- `END` is written at export time with the steps completed
  as of that call, so an exported prefix of a still-running
  session is itself well-formed. Events queued but not yet
  drained are not in the log; replaying to `END` steps
  reproduces the exported state hash.

- **Honesty boundary.** Live producers exist **only** in
  runners (here: page JS + glue exports); cartridges still
  receive input solely through `apply()`/`feed()` at step
  boundaries, and proof runs are unaffected — the Node
  checker queues nothing, so pinned hashes cover the mock
  build unchanged. Feed carts on this page still run their
  synthetic proof streams (`toy_sig_proof_source_init`); the
  mock adds no live sensor source and labels nothing LIVE.
  Tilt/dial here rehearse the *events* a future IMU driver
  will emit, not the IMU or its signal path.

## Required parcel evidence

- `tools/run.sh check` passes (host determinism).
- Emulator sheets at 240 and 360 are visually inspected, not merely
  generated; the report says what phenomenon is visible and when.
- `tools/run.sh board` passes for every registered cartridge
  (the new descriptor's proof run joins automatically).
- The WASM checker prints the same proof stream as native `toy_emu`.
- Any caption/README claim about the phenomenon is backed by either
  the proof run, a measured value quoted from it, or a cited source.

## Current seams not yet parcel-ready

These are known before dispatching the first wave:

- Input/settings seam landed (see "Input & settings seam"
  above). Interactive carts are parcel-ready.
- Signal feed seam landed (see "Signal feed seam" above),
  including the IMU dial producer in core. Sensor carts are
  parcel-ready against synthetic proof streams; LIVE drivers,
  REPLAY proof bytes, further synthetic scenarios and a
  gyro-coherent attitude model still wait for the kits and
  their own parcels. Checkpoint save/load for feed carts is
  refused until source-position restore lands.
- Checkpoint core exists (`core/toy_ckpt.{h,c}`, host
  `save`/`load` - see [Checkpoints](#checkpoints) above), but
  the board flash-slot and browser IndexedDB backends are later
  parcels. Migration between `state_version`s is deliberately
  undefined while states are still changing: loaders reject,
  they never convert.
- Time layer v1 exists host-side (runner clock, `TOYBANK1`
  keyframe banking with verified replay, morning report - see
  [Time layer](#time-layer-banking--morning-report) above).
  The board clock is an unset stub and on-device/site banking
  UIs are later parcels gated on the RP2350 kit.
- No audio *output* abstraction yet. Sensor *input* exists
  (signal feed seam above) for synthetic streams; live
  microphone/IMU drivers land with the kits as a new source
  kind behind the same feed.
- `board/CMakeLists.txt` enumerates cartridge sources explicitly.
  That last manual line is intentional for now: it makes a new
  board-visible cartridge a reviewed change, not an accident of a
  glob.
