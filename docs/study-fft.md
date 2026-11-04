# Study: FFT utility for spectrum cartridges (toy1-fft)

Status: utility delivered and host-verified; no cartridge uses it
yet. Deliverables are `core/toy_fft.h`, `core/toy_fft.c` and the
standalone host test `tools/fft_test.c`. This study sizes the
transform for Wave T2 spectrum carts (orbital spectrum, synth
spectrum, desk seismometer) and records what is measured versus
what is only estimated.

No third-party FFT code is involved and none was fetched or
consulted. TOY-0 §4 once named KissFFT as a future workload; this
parcel supersedes that for the sim core: the transform below is
written from the textbook DFT definition, in muse-softfloat
binary64, so spectra join the toy's central promise — identical
bits on host, RP2040, RP2350 and WebAssembly. A native-float FFT
library could never offer that.

## 1. Algorithm

Iterative radix-2 decimation-in-time FFT, in place over
caller-provided split real/imaginary `f64` arrays:

1. Bit-reversal permutation, integer ops only.
2. `log2(N)` butterfly stages of length `len = 2, 4, ..., N`.
   Each butterfly is one complex multiply by a twiddle factor
   plus an add/subtract pair, every operation a `toy_math.h`
   softfloat call (ties-to-even, as everywhere in the core).

Forward convention (unscaled):

```
X[k] = sum_{n=0}^{N-1} x[n] * exp(-2*pi*i*k*n/N)
```

`toy_fft_inverse` uses conjugate twiddles and applies the `1/N`
scale, so inverse∘forward is the identity up to rounding.
Sizes are powers of two, `1 <= N <= TOY_FFT_MAX_N = 2048`
(§3 explains the cap). Invalid sizes and NULL buffers return
`-1` with buffers untouched; `N = 1` is the identity. The code
writes no globals and allocates nothing, so it satisfies the
`step()` rules in CARTRIDGES.md as-is.

## 2. Twiddles: correctly rounded, computed per stage

At stage `len`, butterfly offset `j` needs
`W = exp(-2*pi*i*j/len)`. The library's trig is argument-in-pi,
so the argument is `theta = 2j/len`, formed in code as
`f64_div(from_u32(2j), from_u32(len))`, then

```
wr = cosPi(theta),   wi = -sinPi(theta)     (forward)
```

Three facts make this the accurate choice:

- **The angle is exact.** `2j` and `len` are small integers and
  their quotient is a dyadic rational with a short binary
  expansion, exactly representable in binary64. The division
  therefore rounds *not at all*: `theta` is the true angle, not
  an approximation of it. There is no `2*pi` constant anywhere —
  working in cycles removes the usual first source of twiddle
  error entirely.
- **The twiddle is correctly rounded.** muse-softfloat's
  `sinPi`/`cosPi` return the nearest binary64 to the true
  sine/cosine of that exact angle. Each component is within
  1/2 ulp of the mathematically exact twiddle, independently
  of every other twiddle.
- **Nothing accumulates.** The faster textbook alternative —
  `W *= W_step` along a stage — rounds at every step, so error
  grows along the stage and the factors spiral off the unit
  circle. Here twiddle error is bounded by construction at
  1/2 ulp per component, forever, at every size.

The price is trig calls: `N - 1 - log2(N)` twiddle pairs per
transform (the `j = 0` factor at each stage is exactly 1 and is
special-cased, which also keeps DC and impulse responses
bitwise exact). The stage loop is ordered `j`-outer so each
twiddle is computed once per stage and reused across blocks —
no table storage, and bit-identical results to block-major
order because butterflies within a stage are disjoint.

If a future cart transforms the same size repeatedly, the
obvious optimisation is a twiddle table computed once in
`reset()` (one `W_N^k` table of `N/2` pairs serves every stage
by stride; +8·N bytes of state) trading state for the trig
cost. Deliberately not built now: it changes no results —
table entries would be the same correctly rounded values —
so it is a pure speed parcel when timing demands it (§5).

## 3. API rationale and state fit

```c
int toy_fft_valid_size(uint32_t n);
int toy_fft_forward(f64 *re, f64 *im, uint32_t n);  /* unscaled */
int toy_fft_inverse(f64 *re, f64 *im, uint32_t n);  /* includes 1/N */
```

Split (not interleaved) arrays, because a cartridge's signal
history is naturally a real array: a real-input cart fills `re`
from its history ring, zeroes `im`, transforms in place, and
reads magnitudes out. In-place operation means the transform
needs no scratch beyond the two arrays it is handed.

A cartridge holds those arrays inside its POD state, so the
budget that matters is `TOY_MAX_STATE_SIZE = 65536` bytes.
The two arrays cost `16·N` bytes; a magnitude (or power) array
kept alongside costs another `8·N`:

| N    | re+im (16·N) | + magnitude (24·N) | state left* | verdict |
|------|-------------:|-------------------:|------------:|---------|
| 64   | 1,024 B      | 1,536 B            | ~64.0 KiB   | trivial |
| 128  | 2,048 B      | 3,072 B            | ~62.5 KiB   | trivial |
| 256  | 4,096 B      | 6,144 B            | ~59.5 KiB   | comfortable |
| 512  | 8,192 B      | 12,288 B           | ~53.2 KiB   | comfortable |
| 1024 | 16,384 B     | 24,576 B           | ~40.0 KiB   | comfortable |
| 2048 | 32,768 B     | 49,152 B           | ~16.0 KiB   | max supported |
| 4096 | 65,536 B     | —                  | 0           | not supported |

\* leftover after the 24·N column, i.e. room for the signal
history ring, integrator state, trails and counters. For
scale: today's largest state is fig8 at 13,960 bytes.

`TOY_FFT_MAX_N = 2048` is set where the buffers alone reach
half the state budget. N = 1024 is the recommended ceiling for
a cart that also keeps a sample history and a magnitude array;
N = 256 or 512 will cover most spectrum displays (a 240-px
round glass cannot show more than ~120 useful bars anyway,
and binning 512 points into displayed bars is a render-side
job).

Real-input note: a real signal's spectrum is conjugate
symmetric, so a cart only needs bins `0..N/2`. The utility
does not exploit this (no packed real-FFT trick): simplicity
and one well-tested code path first. A real-input wrapper that
halves the work is a possible T2 optimisation, at the cost of
a second code path to verify.

Board pickup is deliberately *not* done in this parcel:
`board/CMakeLists.txt` lists sources explicitly and stays
untouched, per CARTRIDGES.md's "reviewed change, not an
accident of a glob" rule. The host build already compiles
`core/toy_fft.c` (it globs `core/*.c`), warning-clean under
`-O2 -std=c11 -Wall -Wextra`. Adding the one board source line
belongs to the first cart that calls the FFT.

## 4. Verification (measured, host)

`tools/fft_test.c` is a standalone program with its own `main`
(built separately; `tools/` is not globbed by `run.sh` and this
parcel keeps it that way). Observed on the development host
(MSYS2 gcc 16.1, 32-bit MinGW target, `-O2`), 2026-11-03:

**(a) Against an independent direct DFT.** The test implements
the O(N²) defining sum with the same softfloat ops and the
same correctly rounded twiddles, but sums sequentially over n
for each bin. Bitwise equality with the FFT is **not** expected
and would in fact be suspicious: the butterfly tree and the
sequential sum round in different orders, so each side carries
its own rounding path to the same mathematical value. The
agreement metric is ulp distance. Two honest readings:

| N   | max raw ulp (any component) | full-scale ulp err | exact components |
|-----|----------------------------:|-------------------:|-----------------:|
| 16  | 3                           | 0.8                | 14 / 32          |
| 64  | 384                         | 1.6                | 28 / 128         |
| 128 | 1,664                       | 4.7                | 35 / 256         |
| 256 | 7,040                       | 4.4                | 42 / 512         |

"Full-scale ulp" is the largest absolute component error
expressed in ulps *of the largest bin*: at N = 256 the two
methods never disagree by more than ~4.4 ulp of full scale
(≈ 1e-15 relative) — exactly the few-roundings-per-stage
behaviour the algorithm predicts. The raw-ulp column looks
alarming only because near-cancellation bins are tiny, so their
ulps are tiny too: an absolute error of 1e-15 on a bin whose
value is 1e-13 is thousands of *that bin's* ulps while being
1e-15 relative to the spectrum. Both numbers are printed by
the test; the pass gate is on full-scale ulp (≤ 256, observed
≤ 4.7).

**(b) Known signals.** Impulse → flat `1 + 0i` spectrum,
bitwise exact at N = 64 and 256. DC → `X[0] = N` exactly and
exact zeros elsewhere (zero *signs* may vary through
`twiddle × 0` products; magnitudes are exactly zero). A cosine
generated exactly on bin k gives peaks at k and N−k that are
**0 ulp** from N/2 at both N = 64 (k = 5) and N = 256 (k = 17),
with worst off-bin leakage 9.1e-15 against a peak of 128 —
the residual there is the correctly rounded *samples* being
1/2-ulp approximations of a true cosine, not transform error.
Forward+inverse round trip on the pseudo-random vector:
max 640 raw ulp (same near-zero-bin effect as above).

**(c) Cross-target hash.** FNV-1a 64 over the full output of a
fixed vector (N = 256, LCG seed `0x5eed1234`, samples are exact
dyadic rationals), hashed in an explicitly little-endian byte
order so the value is a property of the numbers, not of memory
layout:

```
bd852da604b03e3a
```

This is the number a future board or WASM build of the same
test must reproduce. (It survived an internal loop
restructuring during development unchanged, as it should:
reordering disjoint butterflies changes no rounding.)

**(d) Existing carts untouched.** `bash tools/run.sh check`
passes (run under WSL; Git Bash on Windows has the known
pre-existing CRLF issue with this script). Both registered
carts reproduce their hash streams with `toy_fft.c` newly
present in the globbed build.

## 5. Cost: measured host, estimated RP2040

Host timing (same machine/build as §4; `clock()` over repeated
transforms of a fixed vector). These are **host-only** numbers:

| N    | butterflies (N/2)·log2N | twiddle pairs | host µs/transform |
|------|------------------------:|--------------:|------------------:|
| 64   | 192                     | 57            | 533               |
| 128  | 448                     | 120           | 1,203             |
| 256  | 1,024                   | 247           | 2,673             |
| 512  | 2,304                   | 502           | 5,607             |
| 1024 | 5,120                   | 1,013         | 12,000            |
| 2048 | 11,264                  | 2,036         | (not timed)       |

Per butterfly the cost is 10 softfloat ops (4 mul, 2 add/sub
for the complex multiply, 4 add/sub for the butterfly itself);
per twiddle pair, 2 trig calls + 1 exact division. Cost grows
as N·log N with the trig term a large constant factor.

**RP2040 posture — estimates, not measurements.** No board run
was performed for this parcel (by instruction). The RP2040 is
a Cortex-M0+ at 133 MHz with no FPU; the library is integer C,
so the same code runs, but at some large multiple of host time
driven by clock ratio, 32-bit datapath and 64-bit emulation.
Using a deliberately wide planning band of 50–200× the host
figures above — a band to be *replaced by measurement* in the
first board parcel, not quoted as a result:

- N = 256: roughly 0.13–0.55 s per transform — a spectrum
  updating a few times per second is plausible.
- N = 1024: roughly 0.6–2.4 s per transform — fine for a proof
  run or a slowly refreshing instrument, not for
  animation-rate spectra.

Consequences for T2 cart design, whichever end of the band is
true: (i) transform at instrument rate, not per `step()` —
e.g. accumulate samples in `step()`, run the FFT every K steps
into a magnitude array that `render()` only reads; (ii) start
at N = 256; (iii) if board timing hurts, the reset-time
twiddle table of §2 removes the trig term without changing a
single result bit, and is the first optimisation to reach for.

## 6. The binary32 question

Does Wave T2 need an `f32` FFT — half the state, presumably
faster butterflies? The library's standing convention is
format parity: a capability added for one of binary16/32/64 is
added for all alike, never as a quiet single-format extra.
The toy's sim core, however, is binary64-only by design
(`toy_math.h` wraps nothing else; the one planned exception,
the Mandelbrot cart, uses binary32 *as its subject matter* —
a precision comparison — not as a speed hack).

Recommendation: **defer the f32 FFT until a spectrum cart
demonstrates a measured need, then build it to the same
standard, not as a downgrade.** Reasons:

- **State is not the binding constraint.** §3: N = 1024 in
  binary64 leaves ~40 KiB of state free. f32 would only matter
  at N ≥ 2048, where display resolution argues against the
  size anyway.
- **Speed is unproven as binding.** §5's band is an estimate;
  the twiddle-table optimisation (same bits, binary64) should
  be measured on board before reaching for a second format.
- **Correctness story.** An f32 spectrum of an f64 simulation
  inserts a rounding the proof hash then has to explain. The
  toy's thesis is exactness you can watch; keep the spectrum
  in the same arithmetic as the phenomenon it displays.
- **Parity, when it comes, is cheap to do properly.** The
  library already provides correctly rounded binary32
  `sinPi`/`cosPi` and arithmetic, so a future `toy_fft32`
  sibling — same algorithm, own test vector and hash, wrapped
  via a small `toy_math32.h` — is a well-defined parcel. It
  should land as a *matching-format* addition (with the
  same verification bar), triggered by a board-measured
  requirement, not pre-emptively.

## 7. How spectrum carts should use it

Suggested pattern for T2 carts (orbital, synth, seismometer):

- **State**: signal history ring (the phenomenon's own f64
  samples), `re[N]`, `im[N]`, `mag[N/2+1]`, plus cart state.
  Fill `re` from the ring oldest-first, zero `im` with
  `toy_memset` (bit pattern 0 is +0.0, so memset *is* the
  zeroing), transform, then `mag[k] = sqrt(re² + im²)` via
  `f64_fma`/`f64_sqrt` at instrument rate. Magnitudes live in
  state (they are simulation output, hashed like everything
  else); dB scaling and bar heights are render-side.
- **Windowing belongs cart-side, before the transform**, in
  softfloat: e.g. Hann weights `0.5 − 0.5·cosPi(2n/(N−1))`
  computed in `reset()` into the state (or applied on the fly
  per fill). Whether to window at all is a per-phenomenon
  choice — an orbital spectrum of a periodic signal may want
  none; a seismometer will. The utility stays window-agnostic;
  a shared `toy_fft_window_hann` helper can be added when the
  second cart wants one (rule of two).
- **Proof coverage**: because buffers and magnitudes are
  state, the existing whole-state FNV hash already covers the
  spectrum — no new proof mechanism is needed. A cart's
  `proof_steps` should span at least a few transform epochs so
  the hashed states include post-FFT magnitudes. The test
  vector hash `bd852da604b03e3a` covers the utility itself,
  independent of any cart.
- **Determinism hygiene**: sample the phenomenon at a fixed
  step cadence (never wall clock), trigger transforms on step
  counts, and keep the emulated-signal seam (toy1-signals)
  responsible for feeding test streams deterministically.

## 8. Wave T2 parcel breakdown

1. **fft-board-timing** (small): add the one
   `board/CMakeLists.txt` source line plus a board-side timing
   run of `toy_fft_forward` at 256/512/1024; replaces §5's
   estimated band with measured RP2040 numbers and confirms
   the §4(c) hash on hardware. Blocks cart pacing choices.
2. **orbital-spectrum cart**: fig8-family integrator +
   N = 256 spectrum of a coordinate history, no window;
   validates the §7 pattern end to end (host check, sheets,
   board proof, WASM hashes per CARTRIDGES.md).
3. **twiddle-table optimisation** (only if parcel 1 says so):
   reset-time table variant behind the same API shape; gate is
   identical output hash `bd852da604b03e3a` at lower board time.
4. **synth-spectrum / seismometer carts**: after toy1-signals
   lands; Hann windowing introduced here (second user), sizes
   per parcel 1's measurements.
5. **toy_fft32** (only on measured need, per §6): parity
   sibling with its own wrappers, test vector and study note.

## Risks / open items

- RP2040 cost is the one real unknown; everything in §5 beyond
  the host column is a planning estimate until parcel 1 runs.
- Overflow: the unscaled forward transform can grow values by
  up to N (DC case). Carts feeding large-magnitude signals at
  N = 2048 own their scaling; binary64 range makes this a
  theoretical rather than practical concern for toy signals.
- The utility is untested on board and WASM by design in this
  parcel; §4(c)'s hash exists precisely to close that gap in
  parcel 1.
