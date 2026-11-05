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
  host == board == WASM like any other. Scenarios 1 (IMU
  desk, MIC 440 Hz tone) and 2 (IMU6 chest: modelled breathing
  + 75 BPM heart doublet, Wave T3) exist; the allowlist is
  {1, 2} and a device/scenario combo with no generator (MIC
  scenario 2) fails loud at source init. New scenarios are new
  proof vectors, never edits to a pinned stream. Signal carts add
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
- Checkpoints and banks: the source position is host-owned
  and not in `TOYCKPT1`, but for a **synthetic** source it
  needs no bytes - every runner feeds exactly
  `samples_per_step` before each step, so at runner step N
  the source sits at sample `N * samples_per_step`.
  `toy_emu save`/`load` and `TOYBANK1` banking/replay support
  feed carts whose `sig_spec.proof_kind` is `SYNTHETIC`:
  restore instantiates the descriptor's fresh proof source
  and fast-forwards it with `toy_sig_seek_samples` (no
  format change), then continues feed -> apply -> step, and
  the resumed hash stream is the uninterrupted `run` stream.
  `LIVE`/`REPLAY` proof kinds stay **refused loudly** at
  save, load and bank begin/verify: their next sample is not
  a function of a step count, and resuming them unfed (or
  from sample 0) would silently fork the stream. Board
  firmware lists `core/toy_signal.c` explicitly for
  on-device proof instantiation. `tools/sigcore_test.c` is a
  runnable example of the whole seam with a minimal
  IMU-consuming cart.
- Seek exactness: `toy_sig_seek_samples(src, n)` leaves a
  synthetic generator in the state of n discarded
  single-sample polls - byte-exact, proven in
  `tools/sigcore_test.c` for desk (across the t=30 s tap),
  chest and MIC at many `n` up to 1e6, fresh and mid-stream.
  Phase accumulators advance closed-form (`phase += n*inc`)
  and the SplitMix64 state jumps by draws x gamma; the desk
  tap envelope needs only its ~70 decay steps. The
  band-noise IIRs have a per-step floor and no closed form,
  so seek replays just their recurrence (never a sample:
  no sine/scale/clamp) over a bounded 16384-sample tail
  window from both reachable extremes - the poles are
  monotone contractions, so when the extremes meet the
  bracket is the exact state (with an exact full-replay
  fallback if they ever did not); chest has no IIR state
  and is pure O(1). Non-synthetic kinds return an error
  with the source untouched. Cost is independent of `n`
  past the window: measured on the host at n=1e6, desk seek
  ~2 ms vs ~155 ms for discarded polling and chest seek
  below timer resolution vs ~45 ms (MinGW host build; on
  RP2040-class parts expect the same shape at tens-of-ms
  scale for the windowed desk/MIC seek - a bounded ~16k
  sample recurrence, ~100k RNG draws - and O(1) for chest).
- TSIG replay import/export (host emu): two modes re-run a
  cart on recorded bytes. `toy_emu tsigout <cart> <steps>
  <file>` exports exactly the synthetic proof samples a
  `<steps>`-step run would consume (`steps x
  samples_per_step`), as TSIG v1 via the format builder in
  `core/toy_signal.c` with `orig_kind` SYNTHETIC - the file
  never claims to be a recording. `toy_emu feedrun <cart>
  <file.tsig> <steps>` feeds the cart from that file (or any
  TSIG capture, e.g. a real IMU recording) through
  `toy_sig_source_replay` + the bytes-carrying
  `toy_feed_step_bytes` in `core/toy_cart.h`, with the proof
  boundary order, proof script, and step semantics unchanged.
  Validation is strict and happens before any step: a
  signal-deaf cart, a malformed header, a file length that is
  not exactly header + claimed payload, a payload whose
  FNV-1a 64 does not equal the header's payload-hash field
  (the format's integrity check - TSIG v1 has no separate CRC
  field; `toy_sig_parse_header` checks structure and bounds,
  `feedrun` adds the hash pass), or a header disagreeing with
  `sig_spec` on device, channels, or sample rate each exit 2
  with no hash line. The rate must **equal** the spec:
  resampling is forbidden, because per-sample filters and the
  step dt are built for the spec rate, so a re-timed stream
  would run different physics under the same cart name. A
  stream that ends before `<steps>` steps aborts mid-run with
  exit 2 naming the starved step; samples are never invented.
  Identity and determinism: the **file bytes are the stream**.
  `TOY-RUN` prints `sig=replay` and `stream=` the verified
  payload hash, and a `TOY-TSIG` line adds `orig=` (synthetic
  or live), sample count, and the header scales, so two runs
  are comparable exactly when their bytes are. One labelling
  consequence, by design (study section 7): the hashed label
  records REPLAY + the payload stream id, so a `feedrun` final
  hash is its own pinned hash and **differs from the synthetic
  proof hash even when `tsigout` exported those very
  samples** - the sample path round-trips bit-exactly (the
  exported payload regenerates to the same payload hash, and
  synthetic-vs-replay final states differ only in the nine
  label bytes: 8 of `stream_id`, 1 of `kind`), while the
  provenance words in hashed state honestly do not. Board
  and WASM replay sources are still later parcels; the emu
  modes are the place a recorded capture is first re-proven
  against the synthetic file with the same detector.

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
- Feed carts restore through that count, not through stored
  source bytes: the header's step count times
  `samples_per_step` is the synthetic source's sample
  position, rebuilt by `toy_sig_seek_samples` at load (see
  "Signal feed seam"). `toy_emu save` for a feed cart
  therefore runs the same fed, proof-scripted stream as
  `run` - never an unfed NO-SIG run - and `load` continues
  that stream bit-exact (`tools/ckpt_test.c` proves gpend,
  heartbeat and seismo resumed hashes equal the
  uninterrupted run). Only `SYNTHETIC` proof kinds resume
  this way; `LIVE`/`REPLAY` refuse, because no count
  reconstructs them.

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
  loudly. Synthetic feed carts bank on the same terms: the
  banking loop feeds exactly as `run` does, and replay seeks
  a fresh proof source once to the first keyframe's
  `step * samples_per_step` and then feeds continuously
  between keyframes (`TOY_BANK_ERR_FEED_CART` now means only
  a non-synthetic proof kind, whose position no step count
  reconstructs; a tampered feed keyframe still fails on
  replay hash, proven in `tools/ckpt_test.c`). Budget is fixed at 8 MiB per bank (480 fig8 keyframes
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

## Product shell

The web mock's UX as shared runner-side C: `core/toy_shell.{h,c}`
owns which cart is showing, the gallery picker, the per-cart
settings screen, the clock-set screen and the idle clock
screensaver, all rendered through `toy_fb` and driven by
`toy_event_t`. It is presentation-layer code like `toy_clock`:
never inside a cartridge, enforced by the same placement-guard
pattern (`toy_shell.h` will not compile without
`TOY_SHELL_ALLOW` + `TOY_CLOCK_ALLOW`, which only runner code
defines; header location in `core/` would prove nothing, since
carts legitimately include `toy_fb.h`/`toy_cart.h` from there).
The shell holds no cartridge state and never reads wall time:
the runner passes a `toy_time_t` into `toy_shell_render()`
every frame. It is proved host-side by `toy_emu shell <size>
<outdir>`, a compiled-in scripted tour (gallery → settings →
running → clock-set → clockface at a fixed fake now) printing
the action log and a per-frame FNV-1a 64 over the framebuffer
bytes; `board/CMakeLists.txt` links `toy_shell.c` into the
firmware so Friday's board work is drivers + wiring only.

- **Screens.** `GALLERY` (scrolling list of every registered
  cart's name+title, selected cart's title+blurb detail, plus
  `SET CLOCK` / `CLOCK FACE` virtual rows), `RUNNING` (the
  cart owns the glass; the shell draws and steps nothing),
  `SETTINGS` (the current cart's param table), `CLOCKSET`
  (local hh:mm + UTC offset in 15-minute steps), `CLOCKFACE`
  (analog + digital from the runner-supplied time, integer
  hands; unset shows `--:--  SET CLOCK`). Rendering is
  size-parametric (centre/radius fractions, milli-radius hit
  bands, two text tiers) and allocation-free; all shell state
  is one POD-ish `toy_shell_t` the runner allocates.
- **Runner loop.** Every event goes to `toy_shell_event()`
  first; the returned `TOY_SH_ACT_*` mask says what happened
  (`START_CART`, `EXIT_TO_GALLERY`, `PARAM_QUEUED`,
  `CLOCK_COMMIT`, `SCREEN`, `FORWARD_TO_CART`). Only
  `FORWARD_TO_CART` events are queued to the cart, so shell
  gestures can never leak into a cart's input stream. On
  non-`RUNNING` screens the runner renders the shell and does
  not step (editing pauses the sim; state bytes are
  untouched). `toy_shell_tick()` once per frame idles
  `GALLERY` into `CLOCKFACE` after `idle_limit_frames`
  (default 1800 ≈ 60 s at 30 fps, 0 disables); `RUNNING`
  never auto-hides the sim.
- **Exit gesture.** `TOY_EV_KEY_DOWN`/`TOY_KEY_ESC` (or
  `BUTTON_DOWN` id `TOY_SHELL_BUTTON_EXIT`), using only
  existing event types. A corner long-press was rejected:
  `toy_event_t` has no timestamp, so the shell cannot time a
  press, and a stolen rim zone would eat genuine cart pointer
  input. Touch-only drivers therefore own press timing and
  synthesise the ESC event into the shell (ESC in the gallery
  itself goes to the clockface). Settings is
  `TOY_KEY_X` / button 1 / the gallery `SET >` footer; dial
  (25 milliturns = one row), encoder, arrow keys and tap-a-row
  navigate everywhere; tap the focused settings row's edge or
  turn the dial (10 milliturns = one detent) to adjust.
- **Settings/reset semantics** mirror the site panel exactly:
  params live in shell state per current cart, seeded from
  descriptor defaults. A live change queues one `SET_PARAM`
  into `shell->param_queue` (`PARAM_QUEUED`; the runner drains
  it with `toy_apply_queue` at the next step boundary).
  `RESET_ONLY` params — and any param edited while its cart is
  not running — are staged and labelled `*` / "STAGED: APPLIES
  ON (RE)START", queueing nothing. On any (re)start
  (`START_CART`) the shell refills the queue with every
  user-touched param in descriptor order; the runner resets
  the cart and drains that queue before step 1 (the step-0
  slot). Untouched params are never queued, so defaults and
  compiled-in step-0 proof settings are what an untouched
  start runs. I32 steps are 1 (or range/100 on coarse ranges),
  ENUM wraps, F64 edits move on a per-mille-of-range grid
  through the softfloat path (never a host double); an
  untouched F64 keeps its exact declared default bits and
  shows the declared decimal string. Panel independence is
  preserved: shell screens are not input — screen, selection
  and param bookkeeping never enter state, and proof hashes
  are unchanged.
- **Clock ownership.** The runner owns time end to end.
  `CLOCKSET` edits hh:mm + offset in shell fields (seeded from
  the last time the runner passed to render, or
  `toy_shell_clockset_begin`); commit emits `CLOCK_COMMIT`
  and the runner — which owns the date — calls
  `toy_clock_set()`. The shell holds a `toy_schedule_t` copy
  inside `toy_shell_t` (runner-allocated, so runner-owned;
  the shell only defaults it, schedule windows are not edited
  on glass in v1). Nothing time-shaped reaches a cartridge,
  per the placement rule in "Time layer" above.

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
  parcel-ready against synthetic proof streams (scenarios 1
  desk and 2 chest), and host TSIG replay (`feedrun`/`tsigout`
  above) has landed; LIVE drivers, board/WASM REPLAY proof
  bytes, further synthetic scenarios and a gyro-coherent
  attitude model still wait for the kits and
  their own parcels. Checkpoint/bank restore for synthetic
  feed carts has landed (seek by step count, above); LIVE
  and REPLAY sources still refuse checkpoint/bank restore
  by design.
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
