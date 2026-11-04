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
  `step()` itself stays input-free. `apply()` obeys the same
  rules as `step()`: softfloat only, no allocation, no I/O, no
  wall clock, bounded work, writes only its own state.
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
  above). Interactive carts are parcel-ready; the IMU dial
  *producer* and raw sensor streams still wait for the signals
  seam and kits.
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
- No audio output or sensor input abstraction; FFT/microphone/IMU
  cartridges need a device-input study after the kits arrive.
- `board/CMakeLists.txt` enumerates cartridge sources explicitly.
  That last manual line is intentional for now: it makes a new
  board-visible cartridge a reviewed change, not an accident of a
  glob.
