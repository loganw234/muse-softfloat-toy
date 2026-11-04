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

First study landed: `signals` (planted verification 5/5).
The answer to "could we feed emulated signals to test with"
is yes, prototyped: seeded integer-only synthetic IMU/mic
streams behind one source abstraction, hash-identical across
regeneration and TSIG replay (60 s desk IMU stream
333c0854a00a61df on the lead's rerun), with stream kind
carried in state so emulated data cannot alias measured.
Merged with check/WASM PASS; the board firmware does not
include this utility yet, so no board re-run was needed for
this landing (host and WASM builds glob it in cleanly).

`fft` landed next: planted verification 5/5 (defects
separated by symptom class - scramble/mirror/smear/scale/
bound), lead fft_test rerun PASS with the pinned output hash
bd852da604b03e3a reproduced, check/WASM PASS. The utility is
not yet referenced by any cartridge or the board firmware;
it exists so Wave T2 spectrum cartridges start from a proven
transform.

`input`, `timebank`, and `checkpoint` studies landed last
(each planted-verified 5/5 as documents: citations,
arithmetic, and internal consistency checked against the
code).

## Wave T1 close (2026-11-04)

All fifteen Wave T1 parcels are merged: ten cartridges
(cloud, carousel, orrery, lorenz, harmonograph, pendwave,
mandelbrot, microscope, stagnation, intervallens) and five
studies (input, signals, timebank, checkpoint, fft). Every
parcel went through planted-copy independent verification
before merge — 75 planted defects, 75 caught, each verdict
recorded in `docs/metrics/plants.jsonl` — and every cartridge
passed lead check + WASM + board gates on merged main with
hashes identical across host, RP2040, and browser at every
reported step. Two proof horizons were trimmed at merge on
timestamped on-device measurements (cloud 434 ms/step,
mandelbrot ~181x host); per-step maths nowhere changed.
Final state: `tools/run.sh check` PASS (12 cartridges),
WASM checker PASS, board budget PASS. The gallery now runs
twelve deterministic cartridges on the live site and on the
genuine Pico; Wave T2 (interactive/sensor cartridges on the
input/signals seams, time banking, checkpointing, spectra on
toy_fft) is designed and waiting on the display kits.

## Wave T2 opens (2026-11-04)

Owner decisions in `docs/WAVE-T2.md`: RP2350 is the primary
target, always-plugged power, manual time setting,
touchscreen-only interaction plus IMU motion (including
turn-the-device as a dial). First landing: `orbitspec`
(planted verification 5/5; check/WASM/board PASS with hashes
identical to the parcel report). It spectra the figure-8's
body-0 radius — coherently sampled, dominant bin 17 with the
harmonic ladder, confirmed against an independent pipeline —
and is the first cartridge to run `toy_fft` on the Pico
firmware itself.

The input/settings seam and checkpoint core then landed
(each planted-verified 5/5): optional pre-step `apply()` via
six appended descriptor fields, 32-byte integer events
(incl. the dial event) quantised to step boundaries,
`SET_PARAM` settings, compiled-in proof scripts replayed
identically by emu, board and WASM; and TOYCKPT1
checkpointing (64-byte header, CRC-32 + FNV identity,
`state_version` appended last, emu save/load with proven
resume continuity). One combined board gate covers both
merges: check, WASM, and board budgets all PASS with every
pre-seam proof hash unchanged — the seams cost existing
cartridges nothing. Descriptor tails are now spelled out
explicitly in all thirteen cartridges.

Repository made public today at the owner's election (MIT
unchanged); earlier entries describing it as private are the
historical record of how it started.

## Wave T2b batch 1 lands (2026-11-04)

Three parcels, each planted-copy verified 5/5 before merge
(planted tallies in docs/metrics/plants.jsonl): SIG-1 signal
feed seam (feed + sig_spec appended after state_version,
feed -> apply -> step everywhere, parameter-block stream
ids, in-state labelling, gyro dial derivation; checkpoint
save/load refuse feed carts loudly), time layer v1
(runner-owned clock with placement guard, TOYBANK1 sparse
keyframe banking with hash-verified re-stepping replay,
morning report; warp defaults live 1x, playback 16x), and
julia, the first interactive cartridge (touch-chosen c,
proof final 012134f7). Lead merge fixes, each delta-checked:
bank parse now validates the header schedule and refuses
feed carts, and julia gained its SIG-1 zero tail. One
combined board gate over all 14 cartridges: host == board
at every reported step on the genuine Pico (first attempt
captured zero lines in 37 min - an attach race, firmware
proven alive by reboot probe; the comparison was completed
against the preserved capture). check and WASM PASS, with
julia pinned from the lead rerun on the live site.

## Wave T2b batch 2 lands (2026-11-04)

Three more cartridges, again 5/5 planted-verified each:
gravity (KDK leapfrog N-body sandbox, drag/fling spawns
from step counters, tilt field; proof 0011689c), gpend
(first registered feed cart: pendulum driven by the fed
IMU gravity direction off the synthetic desk stream, proof
651bc7de; also adds the true-underrun case to sigcore_test),
and seismo (tremor trace + Hann FFT spectrum; the synthetic
desk tap fires its detector exactly once at step 301;
proof cc3111b9). sigcore_test registry assertion was
generalized to a feed-cart allowlist after going stale
twice on registry growth. All lead gates PASS on merged
main (sigcore/ckpt tests, check, WASM with all three new
carts pinned from lead reruns); the 17-cartridge board
gate is the remaining on-device proof for this batch.
