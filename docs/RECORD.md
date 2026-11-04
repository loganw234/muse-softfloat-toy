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
