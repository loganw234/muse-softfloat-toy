# Study: lightweight fluid simulation for the gallery

Status: study only. No cartridge, no core change. Four honest
kernel prototypes were built under `build/fluid/` (regenerable,
uncommitted; build lines in the appendix), measured on the host,
and projected to the RP2040 as labelled ESTIMATEs. The single
recommendation is section 7.

Question: what lightweight fluid simulation can this gallery
actually run — softfloat-only maths, cartridge state <= 64 KiB
(hard cap, `TOY_MAX_STATE_SIZE`), deterministic fixed-step, on
RP2040 (proof target today; existing carts measure 2–475 ms/step
class on board, live demo wants ~<35 ms/step class), tilt/touch
interactive, round glass.

Short answer: one softfloat candidate is in the gallery's
existing cost class (the wave/shallow-water heightfield, f64,
48x48) and it is the recommendation. No softfloat candidate is in
the ~35 ms/step board-live class; the only candidate in that
class is the integer falling-water cellular automaton, which
uses zero softfloat and is therefore recommended as the first
*follow-on*, not the first fluid. Sections 2–5 are the evidence;
section 6 is the uncomfortable arithmetic behind that split.

## 1. Ground rules and calibration

- Contract: [CARTRIDGES.md](CARTRIDGES.md) state rules. One POD
  struct of bit-pattern integers (`f64`, `uint32_t`, `uint64_t`,
  fixed arrays), <= 65,536 B, all state floating point through
  `core/toy_math.h`, constants from `f64_dec()`, fixed dt,
  bounded work, input only via `apply()` at step boundaries.
- **`core/toy_math.h` is binary64-only today.** There is no f32
  wrapper and no f32 state precedent. The library itself has a
  full `msf32_*` API (`libs/muse-softfloat/include/
  muse_softfloat.h`, incl. `msf32_from_decimal`), so an f32 cart
  is a *new seam* (a `toy_math32.h` + a state-rules amendment),
  not a free choice. The prototypes below measure f32 through
  direct `msf32_*` calls behind wrappers identical in shape to
  `toy_math.h`, so the seam's value is measured, not assumed.
- Board projection factors, measured this study: host step times
  for three landed carts (same host, same `-O2` mingw32 gcc
  build as `tools/run.sh`, `toy_emu run` wall time) against the
  on-device records in [RECORD.md](RECORD.md):

  | cart | host ms/step (measured) | board ms/step (RECORD) | factor |
  |---|---|---|---|
  | cloud | 3.140 (1000 steps) | 434.4 | 138x |
  | fig8 | 0.257 (2000 steps) | 35.9 | 140x |
  | pendulum | 0.510 (1000 steps) | 109.7 | 215x |
  | pendwave | 0.229 (3000 steps) | not recorded | — |
  | mandelbrot | — | "~181x host" (RECORD) | ~181x |

  Working band: **x140 for add/mul/fma-dominated kernels,
  x215 for div/sqrt/trig-heavy ones**, interpolated per kernel
  below. Every RP2040 number in this study is an ESTIMATE from
  this band; nothing fluid-shaped has run on the board. Host
  numbers are records of the prototypes listed in the appendix
  (median of 5 reps; two full runs taken, ranges quoted —
  run-to-run spread was up to ~25% on the smallest kernel).
  Timings are records of these prototypes, never performance
  claims about a future cartridge.

## 2. Candidate A — shallow-water / wave heightfield

Prototype `build/fluid/wave.c`. One height grid `h`, one
velocity grid `v`, symplectic-Euler wave step with fixed
coefficients:

```
lap(i)  = h[l]+h[r]+h[u]+h[d] - 4*h[i]
v[i]    = damp * (v[i] + k*lap(i) + tiltK*(plane(x,y) - h[i]))
h[i]   += v[i]                       (second pass, in place)
```

`plane(x,y) = gx*xn[x] + gy*yn[y]` is the tilt term: the free
surface relaxes toward the plane gravity would make flat, so
tilt both sloshes the pond and re-levels it. Touch is a fixed
3x3 droplet added to `h` in `apply()`. The round glass is a
per-row integer span table (`span0[y]..span1[y]` inside the
inscribed disc, built at `reset()`); loop bounds come from the
spans, so the domain is a disc and the work is data-independent.
12 soft ops per interior cell (3 add + 1 sub + 1 mul for `lap`,
1 mul + 1 fma for `plane`, 2 fma + 1 sub + 1 mul for `v`, 1 add
for `h`), **zero div, zero sqrt, zero trig** — the cheapest
softfloat op mix there is.

State arithmetic (f64 = 8 B, cap 65,536 B):

| size | h + v | + xn/yn + spans + scalars | total | fits? |
|---|---|---|---|---|
| 48x48 f64 | 2*2304*8 = 36,864 | 768 + 384 + ~88 | **~38,104 (58%)** | yes |
| 64x64 f64 | 2*4096*8 = 65,536 | 1,024 + 512 + ~88 | ~67,160 | **no — two grids alone equal the cap** |
| 64x64 f32 | 2*4096*4 = 32,768 | 512 + 512 + ~48 | ~33,840 (52%) | yes, needs f32 seam |
| 48x48 f32 | 18,432 | ~820 | ~19,252 (29%) | yes, needs f32 seam |

Measured (disc interior cells actually stepped: 1,488 at 48,
2,792 at 64):

| config | host ms/step (2 runs) | soft ops/step | RP2040 ESTIMATE (x140–170) |
|---|---|---|---|
| 48 f64 | 3.11 / 2.42 | ~17.9k | **340–530 ms** |
| 48 f32 | 2.69 / 2.71 | ~17.9k | 380–460 ms |
| 64 f64 | 5.09 / 4.62 | ~33.5k | (state does not fit) |
| 64 f32 | 5.52 / 4.86 | ~33.5k | 680–940 ms |

Sanity: after 1,500 steps from a unit droplet under tilt,
max|h| = 0.0632 (48) / 0.0999 (64), identical to 4 decimals in
f32 and f64 and across runs — bounded, decaying, deterministic
on host.

Stability/determinism: explicit scheme, stable iff
`k * lambda_max(-lap) <= 4`, i.e. `k <= 0.5`; prototyped at
k = 0.18 (wave speed ~0.42 cells/step) with damp = 0.996 and
tiltK = 0.002 (a small restoring term, does not tighten the
limit materially). Fixed two passes, no iterations, no
data-dependent branches at all — lockstep-perfect. Failure
modes: k >= 0.5 (or a SET_PARAM that allows it) grows
checkerboard modes exponentially; unbounded droplet pumping in
`apply()` can in principle overflow toward inf/NaN over very
long horizons — bounded by capping the per-event droplet and
the parameter ranges, which the recommendation does. There is
no division, so no divide-by-zero path exists.

Interaction: TILT -> (gx, gy) via `toy_f64_bits_from_milli`,
scaled at apply time; POINTER_DOWN/MOVE -> droplet at the
milli-radius position mapped to the nearest cell; DIAL ->
damping or tilt-gain trim (optional); SET_PARAM for k /
damping / tilt gain.

Rendering: cell blocks scaled to the panel — 48 cells across
the diameter is 5 px/cell at 240, 7.5 at 360, ~9.7 at 466.
`render()` shades by height plus a fake-light slope term
(`h[r]-h[l]`, `h[d]-h[u]`) computed in native display arithmetic
from state, which the rendering rules allow. Blocks at this
size read as chunky-relief water; the slope term does most of
the visual work.

## 3. Candidate B — lattice Boltzmann D2Q9

Prototype `build/fluid/lbm.c`. Two-grid pull-streaming + BGK
collision, equilibrium `feq_q = w_q * rho * (1 + 3(e.u) +
4.5(e.u)^2 - 1.5 u^2)` (second-order expansion of the Maxwellian
in the lattice velocities, derived in the prototype comments,
no third-party code), disc obstacle with bounce-back, left-edge
inflow u = 0.05, omega = 1.25 (tau = 0.8). ~112 soft ops per
cell including 2 divisions (macroscopic velocity), plus 18
copies.

State arithmetic (9 distributions/cell):

| size | two-grid | single-grid (AA parity pattern) | fits? |
|---|---|---|---|
| 32x32 f32 | 73,728 | 36,864 + solid bits | two-grid no; AA yes |
| 32x32 f64 | 147,456 | 73,728 | **no either way** |
| 24x24 f64 | 82,944 | 41,472 + bits | two-grid no; AA yes |
| 24x24 f32 | 41,472 | 20,736 | yes / yes |
| 48x48 f32 | 165,888 | 82,944 | **no either way** |

(AA pattern = in-place stream/collide whose read/write
directions swap with step parity, halving state at equal
arithmetic. Named as a design option; the prototype measures
the two-grid kernel's compute, which AA does not change.)

Measured:

| config | host ms/step (2 runs) | soft ops/step | RP2040 ESTIMATE (x160–215) |
|---|---|---|---|
| 24 f64 | 13.83 / 12.98 | ~65k | **2.1–3.0 s** |
| 24 f32 | 12.28 / 10.41 | ~65k | 1.7–2.6 s |
| 32 f64 | 22.89 / 21.07 | ~115k | (state does not fit) |
| 32 f32 | 19.71 / 18.77 | ~115k | 3.0–4.2 s |

max|f| = 0.4485 after 1,000 steps at both sizes/formats —
stable at these settings. Stability: tau > 0.5 hard limit
(omega < 2); lattice Mach must stay low (|u| < ~0.15);
aggressive inflow forcing or omega near 2 produces negative
populations and blow-up. Work is fully data-independent
(fixed stencil, fixed loops), so determinism is excellent —
but at ~2–4 s/step ESTIMATE on RP2040 it is a proof/banking
artefact, not a demo. Interaction would be lovely (pointer =
movable obstacle or momentum source, dial = viscosity via
omega, tilt = body force), and a 24x24 Karman street is
visually real fluid behaviour nothing else here offers — which
is why it survives as a named follow-on (section 8), gated on
banking playback or RP2350 numbers, not on hope.

## 4. Candidate C — particle fluid (position-based lite)

Prototype `build/fluid/pbf.c`. Predict under gravity,
counting-sort into a 20x20 uniform hash grid held in state,
**fixed 2 solver iterations** of pairwise separation toward
the smoothing radius (one sqrt + one div per interacting
pair), disc-wall projection, velocity from displacement.
Disclosed prototype shortcut: hash cell indices are computed
from a native-double read of the bit patterns in the harness;
a cartridge would pay 2 softfloat->int conversions per
particle per iteration instead (small vs the pair work, but
the measured times are therefore a slight underestimate).

State arithmetic (6 field arrays x,y,vx,vy,px,py + hash
start/count/cursor/order ~6.4 KB):

| config | fields | total | fits? |
|---|---|---|---|
| 200 f64 | 9,600 | ~16.0 KB (25%) | yes |
| 400 f64 | 19,200 | ~25.6 KB (39%) | yes |
| 400 f32 | 9,600 | ~16.0 KB (25%) | yes, needs f32 seam |

Measured:

| config | host ms/step (2 runs) | RP2040 ESTIMATE (x180–215) |
|---|---|---|
| 200 f64 | 18.63 / 16.93 | **3.0–4.0 s** |
| 200 f32 | 15.13 / 13.83 | 2.5–3.3 s |
| 400 f64 | 41.27 / 41.49 | 7.4–8.9 s |
| 400 f32 | 27.74 / 26.23 | 4.7–6.0 s |

Op count is occupancy-dependent (estimated 150–300 soft
ops/particle/iteration at these packings; the measured time is
the record, not this estimate). max|v|^2 stayed O(1) over 750
steps — but note f32 and f64 runs diverge from each other
(0.036 vs 0.280 at 200): expected, they are different
arithmetics, each internally deterministic.

Determinism is the subtle part: *values* are deterministic
(fixed particle order, fixed cell scan, fixed iteration
count — the Gauss-Seidel flavour of updating predicted
positions in place is order-dependent but the order is fixed),
while *work* varies with occupancy, so step time breathes.
That is allowed by the contract (bounded work: the 9-cell
stencil bounds pairs per particle by local packing, worst case
all-in-one-cell is N^2-ish locally and must be capped by a
per-particle pair budget in a real cart — another fixed
constant). Failure modes: tunnelling through the wall at high
velocity (mitigate: clamp displacement per step), explosive
decompression from an over-packed reset, jitter at rest
(separation-only solvers never fully settle), and the
divide-by-r singularity for coincident particles (guarded).
Interaction is the most tactile of all candidates (pointer =
stirrer/attractor, tilt = gravity vector — a literal sloshing
cup), and particles render beautifully at any panel size
(points/discs, no upscale blocks). It is also 6–17x the cost
of the heightfield per step. Not first.

## 5. Candidate D — falling-water cellular automaton (+ spectral, rejected)

Prototype `build/fluid/ca.c`. One byte per cell (empty / wall /
water / sand + moved-flag bit), integer comparisons and swaps
only, **zero softfloat calls**. Water rules per cell, gravity-
relative: fall forward; fall to a forward diagonal (tie-break
by an in-state xorshift32); disperse sideways up to 3 cells
over support. Sand: fall/diagonal only, sinks through water.
Scan runs from most-forward plane backward; lateral scan order
flips with the parity of the in-state step counter (kills the
one-sided bias, stays deterministic). Prototype implements the
4 axis gravity directions and rotates through them to exercise
every scan; an 8-direction version is the same code with
diagonal forward/right tables.

State: 96x96 = 9,216 B + rng/steps ~16 B = **9.2 KB (14%)**;
120x120 = 14.4 KB (22%); even 160x160 = 25.6 KB fits. State
rules already permit integer-only state ("bit-pattern
integers"), so unlike f32 this needs no seam — but it also
exercises nothing in the library this toy exists to show.

Measured: 96x96 **0.050 / 0.046 ms/step host**; 120x120 **0.070
/ 0.070**. The softfloat board factor does not apply to integer
code; cycle-count ESTIMATE instead: 9,216 cells at ~30–60
Cortex-M0+ cycles/cell ≈ 0.28–0.55M cycles ≈ **2–5 ms/step at
125 MHz** — the only candidate in the <35 ms board-live class,
by an order of magnitude. Cell counts (water 1,910 / sand 217
at 96) were identical across runs and across 3,000-step
continuations: mass-conserving by construction (moves are
swaps) and run-to-run deterministic. It cannot blow up — the
state space is finite and every rule is a bounded local swap;
its failure modes are purely visual (staircase surfaces,
scan-order shimmer if parity flipping is omitted, water that
levels one cell per step at dispersion 1). Tilt maps to the
8-direction gravity quantisation (from milli-g at apply time),
pointer is a material brush, dial selects material. Rendering
is 1 cell = 2 px at 240 (120 grid), 3 px at 360 — crisp
pixel-water. Honest assessment: as a *fluid demo* it beats
everything here on cost and interactivity; as a *softfloat
gallery* piece it is off-mission, which is a values call for
the owner, priced in section 7.

Spectral water (FFT-based) is rejected on arithmetic alone,
no prototype: a 64x64 2-D transform is 128 length-64
transforms per field per step through the softfloat FFT of
[study-fft.md](study-fft.md) (which already prices one
transform), times at least an inverse pair, before any
physics — one to two orders of magnitude past the heightfield
for a result (periodic ocean chop) that fits a round closed
glass worse than every candidate above.

## 6. The two hard facts the recommendation is built on

1. **64 KiB is not the binding constraint for grids; board
   step time is.** Everything recommended or follow-on below
   fits state with room to spare except 64x64 f64 (exactly
   capped out) and two-grid LBM past 24 (AA pattern required).
   What actually rations fluid on RP2040 is ~140–215x host
   step time: the <35 ms live class needs a host step under
   ~0.2 ms, and the cheapest softfloat kernel measured is
   10x that. Existing carts live with this (cloud 434 ms/step
   board record; pendwave's 75 steps/frame is a deliberate
   time-lapse, its board-live rate unmeasured per RECORD) —
   a first fluid in the cloud cost class is consistent with
   the gallery, a first fluid claiming board-live would not
   be honest. On the *browser* (where the mock device's perf
   overlay measures real ms/step), a ~2.5 ms host kernel is
   comfortably live at 1–2 steps/frame; that expectation is
   labelled, not measured, until a cart exists.
2. **f32 buys state, not speed — measured.** Across all three
   softfloat prototypes, msf32 kernels ran 0.63–1.12x the msf64
   time on this host run-paired (fastest relative win PBF-400
   at ~0.63x; occasionally slower: wave 64 run 1, 5.52 vs 5.09;
   wave-48/LBM cluster ~0.8–1.1x). [Range corrected at merge:
   verifier recomputation of the tables above.] The library's shared wide internals mean
   binary32 is not the 2–4x lever intuition suggests. Caveat,
   stated plainly: this is a host measurement; the RP2040's
   32-bit datapath might favour msf32 relatively more, and
   nobody has measured that. Until someone does (one small
   board bench, named in section 9), the f32 seam — new
   wrapper header, state-rules amendment, second arithmetic
   for proofs to pin — is not worth buying for speed. Its
   real product would be 64x64 grids and single-grid LBM at
   32, i.e. resolution, later.

## 7. Recommendation: `pond` — wave heightfield, 48x48, f64

Exactly one first fluid cartridge:

- **Binding.** Candidate A at 48x48 in binary64 through the
  existing `toy_math.h`. Why f64: no new seam, format parity
  with every landed cart, and f32 measured no faster (fact 2).
  Why 48: 64 f64 does not fit state (section 2), and 48 leaves
  headroom the follow-ons spend. Why this candidate: cheapest
  softfloat fluid by 4–17x, zero div/sqrt (no NaN path),
  data-independent work, natural tilt + touch, disc-native.
  Decision rule, stated so the owner can flip it knowingly:
  *if board-live <35 ms/step is the binding constraint, the
  answer is the section-5 automaton and `pond` waits*; this
  study reads the gallery's own records (cloud-class costs,
  browser mock device, banking) as saying it is not.
- **Name**: `pond` (4 chars, within the 15-char checkpoint
  cap). Title e.g. "Pond"; blurb: tilt to slosh, touch to
  drop a stone.
- **State layout sketch** (descending alignment, whole-struct
  memset at reset, `state_version` 0):

  ```
  f64      h[48*48];          18,432 B
  f64      v[48*48];          18,432 B
  f64      xn[48], yn[48];       768 B   (reset constants, -1..1)
  f64      k, damp, tilt_k;       24 B   (from f64_dec / SET_PARAM)
  f64      gx, gy;                16 B   (last tilt, in state)
  f64      four;                   8 B
  uint64_t steps;                  8 B
  uint32_t span0[48], span1[48]; 384 B   (disc spans, reset-built)
  uint32_t drops;                  4 B   (droplet counter, display)
  total                        ~38,076 B of 65,536 (58%)
  ```

  Reserved headroom (~27 KB) is deliberate: it is exactly a
  third f64 grid (18,432 B) for the dye follow-on with 9 KB
  to spare, without a state-size crisis.
- **Step algorithm** (fixed counts, no iterations): pass 1
  over span-interior cells updates `v` from `lap(h)`, the
  tilt-plane term and damping, exactly as prototyped
  (k = 0.18, damp = 0.996, tiltK = 0.002 defaults); pass 2
  `h += v`. Boundary cells (span edge) are never written —
  they stay at reset height 0, which is the wall. 12 soft
  ops/cell, ~17.9k/step.
- **Input** (`input_caps = POINTER | TILT | PARAMS | DIAL`):
  TILT stores (gx, gy) (milli-g -> f64 via
  `toy_f64_bits_from_milli`, y flipped from display-down to
  sim-up once, in apply); POINTER_DOWN adds the fixed 3x3
  droplet (centre 1, ring 0.4, clamped to span interior) and
  increments `drops`; POINTER_MOVE adds a half-strength
  droplet (dragging stirs); DIAL trims damping within its
  param range. Params: `wavespeed` F64 "0.05".."0.30" def
  "0.18" (hard-capped below the 0.5 stability limit by the
  declared range — the clamp *is* the blow-up guard),
  `damping` F64 "0.98".."0.9995" def "0.996", `tilt_k` F64
  "0".."0.01" def "0.002". All live (stored in state, applied
  at boundaries — no RESET_ONLY needed).
- **Proof-script concept** (400 steps, report every 100;
  ESTIMATE 400 x 0.34–0.53 s ≈ 2.3–3.5 min board — inside the
  "roughly three minutes" guidance at the band's middle; if
  the first board timing lands high, trim to 300 like cloud's
  horizon was trimmed, per-step maths untouched): step 0
  defaults (no params queued — declared defaults are the
  proof); step 20 `TILT (300, 400)` (slosh toward lower
  right, re-levels on the tilted plane); step 120
  `POINTER_DOWN` at milli-radius (-333, 0) (droplet, rings
  cross the pond in ~60 steps at 0.42 cells/step); step 220
  `TILT (-400, -100)` (slosh back across); step 300 second
  droplet at (250, -250) (interference with the returning
  slosh — the money frame); step 340 `TILT (0, 0)` (settle
  toward flat through step 400). Sheets at 240/360 should
  show: tilted level (frame ~100), clean expanding rings
  (~150), interference cross-hatch (~320), damped near-flat
  gleam (400).
- **Demo pacing**: `demo_steps_per_frame = 2` (at nominal
  30 fps the droplet rings visibly breathe; wave crossing
  ~1 s of wall time when sustained). Honest sustain note for
  the parcel report to carry: browser expected live (host
  kernel 2.4–3.1 ms/step measured; WASM to be read off the
  site perf overlay, not scaled from this); RP2040 ESTIMATE
  340–530 ms/step is ~1–1.5 s/frame at 2 steps/frame — a
  slow-motion pond on today's proof board, in the same
  honesty class as cloud (434 ms/step record) rather than
  the live class. RP2350 (primary target per WAVE-T2) is
  unmeasured for everything in this study.

## 8. What the same capacity unlocks (follow-on carts)

Each reuses the pond pattern: disc-span field(s) in a
<= 64 KiB POD state, tilt/pointer/dial via the existing
input seam, proof script, block render.

1. **`sandbox`** — the section-5 automaton at 120x120:
   water, sand, wall (and fire/steam if rules stay swap-
   local). The gallery's first board-live interactive fluid
   (ESTIMATE 2–5 ms/step class at 96, ~3–7 at 120, integer
   cycle count), tilt = 8-direction gravity (turn the device
   and the sand pours — the DIAL/IMU story made physical),
   pointer = brush. Disclosed on its face as integer
   simulation: it demonstrates the *device*, pond
   demonstrates the *library*.
2. **`inkwell`** — pond plus a dye grid (the reserved third
   grid, total ~56.5 KB / 86%): dye advects semi-Lagrangian
   on the height-gradient velocity derived from pond's own
   `v` field, pointer injects dye instead of droplets.
   ~2x pond's step cost ESTIMATE. Marbled colour in sloshing
   water; same proof shape.
3. **`coral`** — Gray-Scott reaction-diffusion, two f64
   fields + two Laplacians at 48x48 (state ~38 KB like pond,
   ~26 ops/cell ≈ 2.2x pond ESTIMATE, board ~0.8–1.2 s/step):
   a deliberate banking/morning-report cart — run overnight
   plugged in, replay keyframes at 16x ([study-timebank.md]
   (study-timebank.md)); pointer seeds spots, dial walks
   (F, k) through the pattern map. Browser-live, board-banked.
4. **`rapids`** — LBM D2Q9 24x24 f64 AA-pattern (41,472 B),
   Karman vortex street behind a disc: the "real" fluid,
   gated honestly on its 2.1–3.0 s/step board ESTIMATE —
   lands as a proof/banking showcase or waits for RP2350
   measurements. Pointer drags the obstacle, dial sets
   viscosity (omega), tilt is a body force.

Particle fluid (candidate C) unlocks later if a board bench
ever shows the f32/board ratio breaking its way; at measured
host costs it is dominated on every axis by pond + sandbox
together, and this study does not recommend spending a parcel
on it now.

## 9. Open measurements and risks (named, not buried)

- **No fluid kernel has run on the RP2040.** Every board
  figure is a band ESTIMATE from three landed carts' records.
  First pond parcel must take a timestamped on-device
  ms/step (the cloud trim precedent) before its proof horizon
  is final.
- **The msf32-on-RP2040 ratio is unmeasured** and is the one
  measurement that could reopen the f32 seam (state *and*
  speed). A one-afternoon board bench of `wave.c`'s two
  kernels would settle it; worth doing before any 64x64 or
  LBM-32 parcel is briefed.
- Prototype shortcuts, disclosed where they occur: PBF cell
  indices via native read (§4); LBM inflow edge reflects
  instead of a Zou-He boundary (kernel cost representative,
  wake physics indicative only); CA gravity exercised on 4
  axis dirs (8-dir is table work, same cost class); wave
  tilt plane uses 48/64-agnostic coordinates. None of these
  touches a timing conclusion at the band widths quoted.
- Host timings came from `clock()` medians on a 32-bit mingw
  host build; absolute host ms/step will differ on other
  hosts. The board *factors* were derived on this same host
  against this same build, which is why they are quoted as a
  band and applied only to same-build prototype numbers.

## Appendix: prototypes (build/ only, regenerable)

All under `build/fluid/`, none committed (.gitignore covers
`build/`), linked read-only against `libs/muse-softfloat`.
Clean-room: every kernel derived in-file from the equations
stated above; no third-party fluid code consulted or copied.

```
gcc -O2 -std=c11 -Wall -Wextra -Ilibs/muse-softfloat/include -Icore \
    core/*.c emu/*.c libs/muse-softfloat/src/*.c -o build/emu/toy_emu
    # calibration: toy_emu run cloud 1000 250 | fig8 2000 1000 |
    #              pendulum 1000 500 | pendwave 3000 1500 (wall-timed)

gcc -O2 -std=c11 -Wall -Ilibs/muse-softfloat/include -Icore \
    build/fluid/wave.c libs/muse-softfloat/src/*.c -o build/fluid/wave
gcc -O2 -std=c11 -Wall -Ilibs/muse-softfloat/include -Icore \
    build/fluid/lbm.c  libs/muse-softfloat/src/*.c -o build/fluid/lbm
gcc -O2 -std=c11 -Wall -Ilibs/muse-softfloat/include -Icore \
    build/fluid/pbf.c  libs/muse-softfloat/src/*.c -o build/fluid/pbf
gcc -O2 -std=c11 -Wall build/fluid/ca.c -o build/fluid/ca
```

| file | candidate | configs run |
|---|---|---|
| `build/fluid/wave.c` | A heightfield | 48/64, f64+f32, 300 steps x 5 reps |
| `build/fluid/lbm.c` | B LBM D2Q9 BGK | 24/32, f64+f32, 200 steps x 5 reps |
| `build/fluid/pbf.c` | C position-based lite | 200/400, f64+f32, 2 iters, 150 steps x 5 reps |
| `build/fluid/ca.c` | D water/sand automaton | 96/120, integer, 3000 steps x 5 reps |

Raw outputs (both runs) are quoted in the parcel report;
re-running the lines above regenerates them.
