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

## Rendering rules

- Read `fb->w`, `fb->h`, `toy_cx/cy/r`; never assume 240 or 360.
- Draw in centre/radius fractions. Keep labels inside the inscribed
  circle's safe area and check them at both sizes.
- Trails belong in state if they are simulation history; purely
  cosmetic fading of the framebuffer is presentation and must not be
  fed back into the simulation.
- If interpolation is ever added for banked playback, it belongs to
  the presentation layer and must be labelled drawing, not a state.

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

- No input event, settings, or parameter interface: interactive
  cartridges that need touch/IMU or user-tunable parameters need a
  small interface study first. Non-interactive and internally varied
  simulations are ready now.
- No checkpoint serialization beyond "state bytes are the save file":
  versioning/migration is deliberately undefined while states are
  still changing.
- No audio output or sensor input abstraction; FFT/microphone/IMU
  cartridges need a device-input study after the kits arrive.
- `board/CMakeLists.txt` enumerates cartridge sources explicitly.
  That last manual line is intentional for now: it makes a new
  board-visible cartridge a reviewed change, not an accident of a
  glob.
