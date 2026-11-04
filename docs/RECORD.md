# Record

Dated entries, appended as work lands. Failures stay in.

## 2026-11-02 - TOY-0 and sim core v0

Repository created (private), library pinned as submodule at
`v1.0.0` (957fdae). TOY-0 design of record written. Sim core v0:
240x240 RGB565 framebuffer contract, cartridge interface, two
cartridges (figure-8 choreography, double-pendulum twins) stepped
entirely through muse-softfloat binary64. Host screen emulator
renders frames and prints state hashes. Board core-only firmware
runs the same steps on the genuine Pico; host == board state hash
is the round's gate. Display kit (Waveshare RP2040-Touch-LCD-1.28-B)
ordered, not yet arrived; display/touch/IMU drivers are future work.

Measured this round (host check budget, 10,000 steps): fig8 final
hash `d8fda0444c2da4ef`, pendulum final hash `3a378e9b9e40055b`.
Board budget PASS on the genuine Pico (serial E66440A2CB6F5C2B,
125 MHz): fig8 2,000 steps and pendulum 1,000 steps matched the
host hash at every reported step. A first board run failed only
on a capture race (reader attached after the header line);
firmware now waits for USB attach before printing. An RP2350
1.85C kit (360x360, RTC, microphone, microSD) is under
consideration as the primary target; the core was written
resolution- and chip-independent so TOY-0 now dual-targets.

## 2026-11-03 - parcel-ready contract and live WebAssembly emulator

The cartridge descriptor now carries demo pacing and proof-run
configuration, with `TOY_MAX_STATE_SIZE` enforced by hosts, so a
new cartridge enters the native emulator, board proof and WASM
site by registration alone. `docs/CARTRIDGES.md` records the
state/render/determinism rules and the seams that still need a
study before parcels use them (input/settings, checkpoint
versioning, device sensors/audio).

`site/` is the permanent browser emulator: the registered C
cartridges compiled to WebAssembly and stepped live, rendered
from one state at both 240x240 and 360x360. URL parameters
`?cart=`, `?steps=`, `?pause=1` deep-link an exact deterministic
state. The WASM build verifies every proof hash against values
pinned from the native/board runs before deployment.

Deployed the same day to GitHub Pages
(<https://loganw234.github.io/muse-softfloat-toy/>) from the
private repository; note the Pages site itself is public even
though the source repository is private. The first workflow run
built and verified the WASM successfully but stopped at Pages
configuration until Pages was explicitly enabled for Actions;
the rerun deployed and the live deep-linked pendulum state at
step 3600 rendered with the expected hash on both displays.

## 2026-11-04 - Wave T1 merges begin: carousel

First Wave T1 landing. `carousel` (figure-8, Lagrange
equilateral, Euler collinear; one Yoshida-4 integrator,
auto-advancing every two periods) merged after planted-copy
verification scored 5/5 (`docs/metrics/plants.jsonl` — this
repo's own plant ledger, started today). Lead gates on merged
main: `tools/run.sh check` PASS, WASM checker PASS (carousel
pinned at all 8 proof report points from the lead's native
run), board budget PASS with host == board at every reported
step on the genuine Pico (E66440A2CB6F5C2B). Two Windows-host
portability fixes landed on the way: CR stripping in the
cart-list loops and a CR-insensitive final diff in the board
gate — both found by Wave T1 parcels/verifiers, both now in
the stock scripts.

`stagnation` landed the same day: planted verification 5/5,
then check/WASM/board PASS on merged main with hashes pinned
from the lead run (identical to the parcel report). The
binary16/32/64 lanes freeze at 257/1025/2049 exactly as
predicted analytically; the augmented-addition lane's exact
total at proof end is 2^53 + 1952, on the Pico as on the host.

`microscope` landed next: planted verification 5/5, then
check/WASM/board PASS with lead-pinned hashes identical to
the parcel report. The sqrt-at-2 zoom reaches its 8-ulp-wide
deepest stage (13 distinct x values, 6 y levels) identically
on host, board, and in the browser.

`harmonograph` landed next: planted verification 5/5
(including a pixel-measured screen-safety catch against the
planted copy), then check/WASM/board PASS with lead-pinned
hashes identical to the parcel report. Pattern life on the
Pico is the designed 1507 steps; the six detuned rosettes
render inside the glass at both sizes.

`lorenz` landed next: planted verification 5/5 (the verifier
separated two same-symptom twin plants by reset-bit and dt
evidence), then check/WASM/board PASS with lead-pinned hashes
identical to the parcel report. The one-ulp twins separate
at step ~6.3k on the Pico exactly as on the host.

`intervallens` landed next: planted verification 5/5 (an
independent exact-range probe is now the standing way to check
the enclosure), then check/WASM/board PASS with lead-pinned
hashes identical to the parcel report. On-device tallies match
host: 1852 certified-in / 1852 certified-out / 296 uncertain,
zero containment violations over the proof.

`pendwave` landed next: planted verification 5/5, then
check/WASM/board PASS with lead-pinned hashes identical to
the parcel report; the 3000-step recurrence hash matches
across proof, check, and sheet streams on every target. Note:
its demo pacing (75 steps/frame) is a deliberate time-lapse so
a sheet spans one recurrence; whether an RP2040 sustains that
live at 30 fps is unmeasured — the board gate proves the proof
stream, not live pacing. Revisit at display-layer time.

`orrery` landed next: planted verification 5/5, then
check/WASM/board PASS with lead-pinned hashes identical to
the parcel report (Mercury's 87.9694-day period and radius
range reproduce on-device). The board capture window was
raised 900 s -> 2400 s at this landing: with ten cartridges
registered the cumulative on-device proof simply outgrew it
(orrery was cut off at step 1500/2000 with every hash correct
- a window limit, not a failure).

`cloud` landed next: planted verification 5/5, then
check/WASM/board PASS. Its proof horizon was trimmed by the
lead at merge, 6000 -> 1000 steps (report every 250), after a
timestamped on-device measurement: 434.4 ms/step on the
RP2040 (fig8 35.9 ms, pendulum 109.7 ms for scale), so the
parcel's 6000-step proof is a ~43-minute board run. Per-step
maths is unchanged - the trimmed stream's step-1000 hash and
the 10000-step check hash are the verified values - and the
separation growth at 1000 steps is x244, per the parcel's own
table.

`mandelbrot` landed last of the cartridges: planted
verification 5/5, then check/WASM/board PASS. Its proof was
trimmed by the lead at merge, 896 -> 672 steps (exactly six
stage cycles, ending on the held stage-5 grid where the
binary32 half has collapsed), after a timestamped on-device
measurement of ~1.2-2.0 s per compute step (~181x host)
projected the original proof at ~24 minutes of board time.
The cumulative board gate for all twelve cartridges now runs
inside a 3600 s capture window. All ten Wave T1 cartridges
are landed; the five seam studies follow.
