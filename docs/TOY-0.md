# TOY-0 — a deterministic gallery desk toy on RP2040

Study/plan of record for muse-softfloat-toy. Written 2026-11-02 by the
lead after the muse-softfloat hackathon submission closed. The sealed
library repository is not modified by this project; the library is
consumed as a pinned submodule (`libs/muse-softfloat` at tag `v1.0.0`,
commit `957fdae`).

## 1. What this is

A small desk object that runs exact simulations forever: a gallery of
"cartridges" (orbits, chaos, precision demos, later FFT/IMU instruments)
sharing one deterministic simulation core built on muse-softfloat's
binary64 arithmetic, so any state the toy reaches can be re-derived
bit-for-bit on a desktop. The toy's thesis is the library's thesis:
exactness you can watch.

## 2. Target hardware

Two candidate kits, same product family; the sim core is written
chip- and resolution-independent so both stay targets:

Primary candidate: Waveshare RP2350-Touch-LCD-1.85C (owner is
weighing this against the RP2040 kit below):
- RP2350 (dual Cortex-M33 @150 MHz, 520 KB SRAM, hardware SP FPU +
  double coprocessor — deliberately unused by the sim core, §3),
  16 MB flash, USB-C, lithium battery support, speaker-box variants
- 1.85" round capacitive-touch IPS LCD, 360x360 (ST77916 over QSPI;
  CST816 touch over I2C)
- 6-axis IMU, RTC chip (real wall-clock for active-hours scheduling
  and event timestamps), audio codec + microphone (a real audio
  source for the FFT cartridge; speaker for sonification ticks),
  microSD slot (banked trajectories and event snippets stop being
  storage-constrained)

Also ordered (2026-11-02): Waveshare RP2040-Touch-LCD-1.28-B
($27.99 kit, CNC metal case):
- RP2040, 264 KB SRAM, 16 MB flash, USB-C, battery header
- 1.28" round touch LCD, 240x240 (GC9A01A over SPI, CST816-class)
- QMI8658C 6-axis IMU (accelerometer noise ~150-200 ug/sqrt(Hz))

Until either arrives: the six existing boards (genuine Pico + five
RP2040-Zero clones) run the sim core headless, and the host screen
emulator (§5) stands in for the display. Display/touch/IMU drivers
live in a per-board layer; nothing chip-specific enters `core/`.
Because the library's arithmetic is integer C, the same sim core
produces identical state hashes on host, RP2040 and RP2350 — the
cross-chip hash equality is itself a gate once both kits exist.
The one RP2350 trap to keep shut: no hardware float anywhere near
the sim (soft-float ABI for board builds; the library uses no
float types, but toy code must not either).

## 3. Architecture: three layers that never touch

1. **Sim core** (`core/`): pure deterministic computation. All
   floating-point through muse-softfloat `msf64_*` calls (ties-to-even
   default), all constants parsed from decimal strings via
   `msf64_from_decimal` so they are correctly rounded identically on
   every platform. No heap, no platform headers, no wall clock. A
   cartridge is: reset, step (fixed internal dt), render into a
   framebuffer, expose its state bytes for hashing/checkpointing.
   Bit-identical host == board is the acceptance gate, as in the
   library project.
2. **Time layer**: decouples sim time from wall time. Pacing mode
   (K steps per displayed frame, user warp incl. "hold during my night
   hours") and banking mode (the integrator keeps stepping while the
   display sleeps, writing decimated trajectory samples to a ring
   buffer; playback at wake is sped up). Honesty rule inherited from
   the library project: stepped states are exact; any interpolation
   between banked samples is drawing, and the UI never lets a drawn
   frame claim to be a stepped state. A scrolling short state hash in
   the corner of the display is the visible proof hook.
3. **Display/input** (device only, later): GC9A01 framebuffer flush,
   touch gestures (swipe = next cartridge, rim-swipe = warp dial,
   tap = interact), IMU input (tilt, shake, face-down = sleep,
   pickup = wake + morning report). On device, core 0 integrates and
   core 1 renders/inputs from immutable snapshots (double-buffered
   state copy); the sim never waits on the display.

Framebuffer contract (shared by device and emulator): square
RGB565 whose size is a runtime property of the framebuffer object
(240 or 360 today), origin top-left, drawing clipped to the
inscribed circle by the presentation layer; cartridges draw in
framebuffer-relative coordinates (centre + radius fractions), so a
cartridge never knows which glass it is on.

## 4. Cartridges v0 (this study's scope)

1. **fig8** — the Chenciner–Montgomery figure-8 three-body
   choreography (initial conditions as published, cited in code),
   Yoshida 4th-order symplectic composition of leapfrog, fixed dt.
   Acceptance: state hash after 10,000 steps identical host vs board;
   return error after one period (T ~ 6.3259) printed by the emulator
   harness (record, not a gate).
2. **pendulum** — double-pendulum twins integrated by RK4, initial
   angle differing by one ulp via `msf64_next_up`; divergence made
   visible. Trig via the library's `sinPi`/`cosPi` with the angle held
   in cycles (the library has no radian trig by design; SF-2 deferred
   it, and this toy does not reopen that).

Deferred cartridges (named, not built): choreography carousel,
solar-system orrery, divergence cloud (N perturbed copies), Lorenz,
harmonograph, pendulum wave, Mandelbrot f32|f64 split, Julia-by-touch,
function microscope, stagnation/augmented counters, interval lens,
IMU seismometer + KissFFT spectrum (KissFFT itself is a future
fetch-by-hash workload like the library project did it — never
vendored), orbital spectrum, attitude ball, morning report.

## 5. Screen emulator

`emu/` builds the same core for the host and renders frames to PPM
plus a PNG contact sheet, printing the per-frame state hash. It is
the display before the display exists: if a frame is wrong in the
emulator it is wrong on glass, because both consume the same
framebuffer contract. When the kit arrives, the emulator stays as
the regression surface for layout (what fits a circle, what text
survives 240 px).

## 6. Licensing posture

- This repo: MIT, Copyright (c) 2026 Logan W. (to settle at first
  release; private during development).
- muse-softfloat submodule: MIT, pinned at the sealed tag.
- No REBOUND code: integrators here are textbook compositions
  (leapfrog/Yoshida, RK4) written for this repo; REBOUND remains an
  offline reference in the library project only. This keeps a future
  product path GPL-free.
- Figure-8 initial-condition constants are the published values of
  Chenciner & Montgomery (2000), cited at the definition site.

## 7. Verification plan (per round)

- `tools/run.sh host` — build core + emulator, run both cartridges,
  print hashes, render frames (determinism: two runs, identical
  hashes).
- `tools/run.sh board` — build the core-only firmware, flash the
  genuine Pico by serial under the shared board lock, read the state
  hash stream over CDC, compare to the host's. Host == board or the
  round does not close.
- Parcel/verifier discipline carries over from the library project
  for anything beyond lead-sized changes, with the case-report
  amendments applied from the start: provenance gate for verifiers,
  delta checks for post-copy fixes, report-file-first.

## 8. Open questions for the owner (none blocking v0)

- Battery or always-plugged desk object? (drives sleep policy)
- Case buttons: the -B case exposes BOOTSEL/reset only; is touch-only
  input acceptable, or add a rotary encoder on the SH1.0 GPIOs later?
- Sound: none onboard; a piezo on GPIO could tick orbital periods.
  Wanted?
