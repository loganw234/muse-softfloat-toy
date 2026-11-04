# Study: emulated signal feeding (toy1-signals)

Question (owner): "could we feed emulated signals to test with" —
can sensor-dependent cartridges (desk seismometer/tremor display,
attitude ball, mic spectrum) be developed and *proven* before and
without the physical sensors, by feeding deterministic synthetic
or recorded-replayed sensor streams through the same abstraction
the real drivers will use?

**Answer: yes, with one small cartridge-contract extension, and
with a hard boundary on what "proven" means.** A sensor cartridge
fed a synthetic or replayed stream through the interface below
can be proven bit-identical on host, board and WASM exactly like
today's carts — same stream in, same state hash out, everywhere.
That proves the cartridge's mathematics, rendering, state
handling, cost per sample and failure handling. It cannot prove
anything about the physical sensor: its noise, its clock, its
bus, its mounting, or whether the desk actually trembles the way
the synthetic scenario says it does (§9). The live driver, when
the kits arrive, drops in as a third source kind behind the same
interface; a live run can additionally be *recorded* into the
replay format, turning today's hardware observation into
tomorrow's permanent proof vector.

This study ships working code, not only prose:
`core/toy_signal.h`, `core/toy_signal.c`,
`core/toy_signal_sin.inc`, `tools/signal_proto.c`,
`tools/gen_sin_table.py` (§8 for what they demonstrate, with
hashes). No existing file is modified. The one contract change
the design eventually needs — an optional `feed` step on the
cartridge descriptor — is specified in §4 but deliberately
*not* implemented in this parcel; it is T2 parcel SIG-1 and must
be coordinated with the toy1-input study, which may land a
general input seam that subsumes it.

## 1. Where this hooks in today

The current contract is sensor-free by design, and this study
must not pretend otherwise:

- `core/toy_cart.h:32-34` — a cartridge is exactly
  `reset`/`step`/`render`, all taking only the state (plus the
  framebuffer for render). There is no input channel at all.
- `docs/CARTRIDGES.md:34-35` — "`step()` advances one fixed
  internal dt and does bounded work. It does not allocate,
  print, **read input**, or branch on platform."
- `docs/CARTRIDGES.md:19-29` — state is one POD of bit-pattern
  integers ≤ `TOY_MAX_STATE_SIZE` (`core/toy_cart.h:19`); no
  pointers, no native float/double, no `rand()`, no wall clock
  in `reset()`/`step()`; all FP through `core/toy_math.h`.
- `docs/CARTRIDGES.md:83-84` — "No audio output or sensor input
  abstraction; FFT/microphone/IMU cartridges need a device-input
  study after the kits arrive." This study is the device-input
  study, pulled earlier: the seam can be designed and proven
  with emulated sources before the kits arrive.
- Proof today is a hash stream: `emu/main_emu.c:54-68` (`proof`
  mode), `board/main_board.c:21-37` (`run_cart` over CDC), and
  `site/wasm_glue.c:75-81` (`toy_wasm_step`) each call
  `step()` `proof_steps` times and print `toy_hash_state`
  (`core/toy_carts.c:18-27`, FNV-1a 64) at intervals from the
  descriptor (`core/toy_cart.h:29-31`). Any signal design that
  preserves "the host loop decides, sample by sample, what the
  cartridge consumed" slots into all three loops unchanged in
  shape (§4).
- `tools/run.sh:14-21` globs `core/*.c` into the host build;
  `board/CMakeLists.txt:29-36` enumerates sources explicitly.
  New core code therefore reaches host automatically but the
  board only deliberately — the right asymmetry, kept (§10).
- Hardware facts assumed: QMI8658C 6-axis IMU on the RP2040 kit,
  "accelerometer noise ~150-200 ug/sqrt(Hz)" per
  `docs/TOY-0.md:40`; 6-axis IMU + audio codec/microphone +
  microSD on the RP2350 kit per `docs/TOY-0.md:30-34`.

## 2. The sample-source abstraction

One interface, three kinds. The kind is a first-class value,
never inferred (`core/toy_signal.h:34-41`):

```c
#define TOY_SIG_KIND_NONE      0u  /* no source attached      */
#define TOY_SIG_KIND_SYNTHETIC 1u  /* generated from a seed   */
#define TOY_SIG_KIND_REPLAY    2u  /* recorded bytes, bit-exact */
#define TOY_SIG_KIND_LIVE      3u  /* real driver (T2)        */

#define TOY_SIG_DEVICE_IMU6 1u  /* ax,ay,az,gx,gy,gz int16 counts */
#define TOY_SIG_DEVICE_MIC1 2u  /* one int16 PCM channel          */
```

Every stream is described by one plain-integer info block
(`core/toy_signal.h:55-68`): kind, `orig_kind` (for a replay:
were the original samples synthetic or measured?), device,
channel count, sample rate, scale constants (LSB/g, LSB/(°/s)),
scenario id, seed, stream id, sample count. `stream_id` is the
FNV-1a 64 hash of the payload in the *defined* byte order
(§5); two runs are comparable only if their `stream_id`s match,
and hosts print it in the proof header line (§7).

The source itself mirrors the cartridge pattern — small POD
state, no heap, functions that fill caller buffers. The
prototype form (`core/toy_signal.h:211-237`) is a tagged union
with one poll function:

```c
typedef struct {
    toy_sig_info_t info;
    toy_sig_synth_imu_t synth_imu;
    toy_sig_synth_mic_t synth_mic;
    toy_sig_replay_t replay;      /* cursor offsets only, no ptr */
} toy_sig_source_t;

uint32_t toy_sig_poll(toy_sig_source_t *src,
                      const uint8_t *replay_bytes, size_t replay_len,
                      int16_t *out, uint32_t max_samples,
                      uint32_t *flags_out);  /* END / UNDERRUN */
```

Two deliberate properties:

- **The source is not cartridge state and is never hashed.**
  The host owns it. Only the sample *values*, the kind, the
  stream id and counters cross into hashed state (§4). This
  keeps `docs/CARTRIDGES.md:19-22` intact even for LIVE, whose
  driver state (I2C buffers, DMA) is not POD.
- **The replay cursor holds offsets, not a pointer**
  (`core/toy_signal.h:195-200`); the host passes the byte
  buffer on every poll. The buffer can then live in host RAM,
  board flash or WASM linear memory without the source struct
  changing shape.

For T2, when LIVE arrives, the same semantics should graduate
to the descriptor shape the project already uses for cartridges
(`core/toy_cart.h:21-35`): a `const` vtable
(`info`, `reset`, `poll`) + opaque host-owned state. The poll
signature and flag semantics above carry over verbatim; the
tagged union is the prototype, not the endpoint.

## 3. Rates, quantisation, and what a "sample" is

A sample is what the real device would hand us: sensor-native
**int16 counts**, quantised at generation time. The generator
never produces a float and converts it — there is no native
floating point anywhere in the sample path (§6), which is the
only way the same seed yields the same counts on RP2040 later.

Assumed device configurations (to confirm against the QMI8658C
datasheet and the RP2350 kit's codec when they land; changing
them changes `scale_num`/`scale2_num` in the info block, not
any code shape):

| | IMU (both kits) | Mic (RP2350 kit) |
|---|---|---|
| layout | ax,ay,az,gx,gy,gz interleaved | mono |
| counts | int16 per axis | int16 PCM |
| accel range | ±2 g at 16384 LSB/g | — |
| gyro range | ±256 °/s at 128 LSB/(°/s) | — |
| default rate | 100 Hz | 8000 Hz |
| bytes per sample | 12 | 2 |

Rate choices: a desk seismometer's band of interest is roughly
0.1–20 Hz (building sway, footfalls, bumps), so 100 Hz gives
comfortable Nyquist headroom at trivial cost (1200 B/s). The
attitude ball wants the same 100 Hz. 8000 Hz mono is the
cheapest rate that makes a mic spectrum honest to 4 kHz
(speech, tones, room events); 16 kHz is a parameter change, not
a design change.

Quantisation realities the models must respect, and do in the
prototype:

- counts clamp at int16 range (`toy_sig_clamp_i16`,
  `core/toy_signal.c:63`) — a synthetic tap that should saturate
  a ±2 g range saturates at 32767 in the stream too, so carts
  see clipping before hardware exists to clip;
- amplitudes are chosen in LSB and physical units both (1 g =
  16384 LSB; the prototype tap peaks at 8000 LSB ≈ 0.49 g);
- sub-LSB phenomena simply do not exist in the stream — if a
  cart needs averaging to see them, it will average synthetic
  noise exactly as it would real noise.

Conversion from counts to binary64 for cartridge mathematics is
the *cartridge's* job, through the existing softfloat wrappers
(`core/toy_math.h`, e.g. int→f64 then divide by an
`f64_dec("16384")` scale at reset, per
`docs/CARTRIDGES.md:30-33`). The stream stays integer so the
proof covers the conversion too.

## 4. How samples enter cartridge state (the one new seam)

Samples enter **only at step boundaries**, in an order fixed by
the host loop — the sample analogue of "stepped states are
exact; interpolation is drawing" (`docs/TOY-0.md:68-72`,
`docs/CARTRIDGES.md:58-59`). Proposed contract extension, the
single deliberate change to `toy_cart_t` in this design:

```c
/* Optional; NULL for every current cartridge. */
void (*feed)(void *st, const int16_t *samples, uint32_t n_samples,
             uint32_t channels, const toy_sig_info_t *info);
uint32_t sig_device;         /* TOY_SIG_DEVICE_* or 0          */
uint32_t sig_sample_rate_hz; /* rate the cart's dt was built for */
uint32_t sig_samples_per_step;
```

Host proof loop, all three hosts identically (compare
`emu/main_emu.c:60-65`, `board/main_board.c:27-34`,
`site/wasm_glue.c:79-80`):

```c
for (s = 1; s <= proof_steps; s++) {
    if (cart->feed) {
        got = toy_sig_poll(src, ..., frame, cart->sig_samples_per_step, &flags);
        if (got != cart->sig_samples_per_step) abort_run(); /* proof: fail loud */
        cart->feed(st, frame, got, chans, &src->info);
    }
    cart->step(st);
    ... hash+print as today ...
}
```

Rules that make this provable rather than merely plausible:

1. **Index-derived, never clock-derived.** The sample sequence
   number of the i-th sample in a proof run is
   `(s-1)*samples_per_step + i`. Wall-clock time chooses *when*
   a live demo renders; it never chooses *which* samples a step
   consumes. (Live demo pacing is the timebank study's problem;
   proof has no pacing.)
2. **`feed` copies values into POD state** — a small ring or
   block buffer, plus `sig_kind`, `sig_orig_kind`,
   `sig_stream_id`, next-expected sequence and a gap counter.
   `step()` then consumes from state only, so the existing rule
   "step does not read input" survives literally: input crosses
   the boundary in `feed`, between steps, and everything it
   wrote is inside the hashed bytes.
3. **Rates bind statically.** The host refuses to start a proof
   run unless `src.info.sample_rate_hz == cart->sig_sample_rate_hz`
   and the device matches. No resampling in v1; a deterministic
   integer decimator (e.g. 200 Hz recording → 100 Hz cart) is a
   possible later parcel, and it would live in core/ so it is
   itself proven, never in host glue.
4. **Underrun/end never silently becomes data.** In a proof
   run, `TOY_SIG_FLAG_UNDERRUN`/`END` before `proof_steps` is a
   failed run (no zero-fill, no sample-repeat). In a live demo,
   a gap is fed as zero *samples with the gap counter
   incremented in state*, so the display can show the gap
   rather than invent a measurement.
5. **Typical bindings:** seismometer/attitude at 100 Hz with
   `dt = 10 ms` → 1 sample/step. Mic spectrum → one FFT block
   per step (e.g. 256 or 512 samples/step at 8 kHz; the block
   size belongs to the toy1-fft study's cost measurements).
   `demo_steps_per_frame` still governs how much sim time a
   rendered frame advances (`core/toy_cart.h:29`,
   `docs/CARTRIDGES.md:42-44`).

Until SIG-1 lands this seam, a synthetic-only proof can already
be run *today* by embedding the generator state inside a
cartridge's POD state and generating inside `step()` — the
generator below is POD-shaped for exactly that fallback. Replay
cannot work that way (bytes must come from the host), which is
why the seam is needed rather than optional polish.

## 5. The three source kinds in detail

### 5.1 SYNTHETIC

Fully specified integer generation (`core/toy_signal.c`):

- **PRNG:** SplitMix64 as a sequential generator
  (`core/toy_signal.c:10-26`): one uint64 state, period 2^64,
  only defined unsigned arithmetic. Not cryptographic; its job
  is identical noise everywhere, and any seed (including 0) is
  valid. Bipolar draws take the top 16 bits recentred to
  [-32767, 32767] (`core/toy_signal.c:27-34`).
- **Sine:** phase accumulator (2^32 = one cycle) +
  1025-entry Q15 table with linear interpolation
  (`core/toy_signal.c:45-54`, table in
  `core/toy_signal_sin.inc`, generated once offline by
  `tools/gen_sin_table.py`). Frequency = `round(f·2^32/Fs)` in
  uint64 from millihertz parameters
  (`core/toy_signal.c:55-61`) — no per-frequency offline
  constants, and a chirp is just a per-sample increment delta.
  The table integers are structural constants in the sense of
  `docs/CARTRIDGES.md:30-33`: the script that made them is
  committed, and regenerating them is a format version bump.
- **Shifts/division:** all fixed-point scaling goes through
  `toy_sig_asr64` (`core/toy_signal.c:36-43`), a floor
  arithmetic shift built from unsigned ops, so nothing depends
  on C11's implementation-defined signed `>>`; signed overflow
  is excluded by amplitude bounds (products are int64, values
  are ≤ ~2^15 × gains ≤ 2^8).

Signal models (prototype implements the first three IMU models
and the first two mic models; the rest are specified here as
scenario additions):

- *Gravity under slow rotation.* A 1 g vector of fixed tilt
  (0.25 g circle in ax/ay, az = 15864 LSB =
  round(√(16384²−4096²))) describing one circle per 20 s
  (`core/toy_signal.c:140-205`). |g| stays 1 g to quantisation
  (§8 measures 1000 mg). General form for the attitude ball:
  sin/cos phasor pairs per axis from the same table.
- *Tremor as band-limited noise.* White noise through two
  one-pole integer lowpasses, output = fast − slow — a crude
  bandpass with two int32 of state
  (`core/toy_signal.c:70-79`), independently per axis, scaled
  to ~80 LSB sd (≈5 mg) on accel. Honest limit: this is *a*
  coloured noise, not measured desk spectra (§9).
- *Tap impulse.* At a scheduled sample, a 20 Hz ringing burst
  with Q15 exponential decay (×29491/32768 per sample),
  8000 LSB peak on az, quarter on ax
  (`core/toy_signal.c:179-191`).
- *Footsteps (specified, not yet coded):* a train of thumps at
  ~1.8 Hz, each a 6–10 Hz half-cycle burst of 300–1500 LSB,
  alternating sign, amplitude jittered by the PRNG.
- *Car pass (specified):* a 3–6 s raised envelope
  (integer Hann via the sine table) over 2–5 Hz bandnoise at
  100–400 LSB — the "something rolled past the building"
  transient a seismometer cart must not mistake for a tap.
- *Mic tones/chirps/noise:* sine from the same phasor
  (440 Hz at 12000 LSB peak in the prototype), linear chirp
  440→880 Hz, bandnoise hiss, all summable, all clamped
  (`core/toy_signal.c:208-253`).
- *Fault scenarios (specified, high value):* saturation
  (over-range held at ±32767), stuck axis (one channel
  frozen), dropouts (poll returns UNDERRUN on schedule),
  swapped axes, DC offset drift. Emulated faults are the one
  thing real hardware is *worse* at providing on demand; carts
  can be proven to flag rather than hide them.

Scenarios are parameterised by `scenario_id` + `seed` in the
info block; a proof run names both, so "the seismometer proof
stream" is a pair of small integers, not a file.

### 5.2 REPLAY

TSIG v1 byte format (`core/toy_signal.h:170-184`,
writer/parser at `core/toy_signal.c:285-354`): a 64-byte header
— magic "TSIG", version, header size, orig_kind, device,
channels, rate, both scales, seed, sample count, payload hash,
scenario id, reserved-zero — followed by interleaved int16
little-endian samples. All integers are packed by explicit
byte stores, never struct `fwrite`, and the stream hash is
FNV-1a 64 over the payload *in that byte order*
(`core/toy_signal.c:93-106`, same constants as
`core/toy_carts.c:21-25`). Note the one deliberate upgrade over
`toy_hash_state`: the stream hash is over a defined byte order,
not raw struct memory, so it is endian- and layout-proof.
The parser bounds-checks the claimed payload against the
actual buffer length before any read
(`core/toy_signal.c:306-345`) and rejects truncation and bad
magic (§8 tests both).

Sizes are small: 60 s of 100 Hz IMU is 72,064 B; 60 s of
8 kHz mic is 960,064 B. Proof clips should be seconds, not
minutes; long recordings are for demo, not gates.

How each target obtains the bytes:

- **Host emu:** a file argument; the emu reads it whole (its
  existing file I/O lives in `emu/main_emu.c:22-45`).
- **Board:** embedded as a generated `const uint8_t[]` in the
  board layer, living in the 16 MB flash both kits have
  (`docs/TOY-0.md:26,38`); `board/CMakeLists.txt` gains the
  line deliberately, as with cartridges
  (`docs/CARTRIDGES.md:85-88`). The microSD on the RP2350 kit
  (`docs/TOY-0.md:33-34`) later allows field recordings
  without reflashing, but proof vectors should stay embedded
  so a board gate cannot depend on card contents.
- **WASM:** the glue already exposes buffers to JS
  (`site/wasm_glue.c:93-95` framebuffer pattern); add
  `toy_wasm_sig_buffer()`/`toy_wasm_sig_capacity()` and let
  `app.js`/the Node checker copy TSIG bytes in before select.
  The Node proof checker reads the same file the emu used, so
  `site/expected-hashes.json` gains stream ids next to state
  hashes.

### 5.3 LIVE (specified now, implemented with the kits)

A driver is a source whose `poll` drains the device FIFO into
the same int16 interleaved layout. Two rules keep live honest
and future proofs possible:

- **Record raw counts.** Anything the driver can be configured
  to do invisibly — on-chip filtering, offset registers, AGC —
  is either disabled, or recorded in the TSIG header's scale /
  scenario fields when a recording is made, so replay
  reproduces what the cartridge saw. Calibration belongs in
  core/ (deterministic, proven), fed by raw counts.
- **Every live source gets a tee recorder**: the same frames
  can be written to a TSIG file (microSD, or up the CDC link
  the board proof already uses, `board/main_board.c:3-5`).
  This is how real proof vectors are born: measure once on
  hardware, replay forever everywhere.

A live run is *never* a proof run — its samples are not
reproducible — but its recording is, the moment it exists.

## 6. Determinism: why the same stream hashes the same everywhere

The generator's sample path uses only: uint64 wraparound
arithmetic (defined), int64 products of bounded int32 values
(no overflow), floor shifts built from unsigned ops, table
lookups, and truncating integer division (defined in C11).
No `float`/`double`, no `<math.h>`, no `rand()`, no clock, no
heap, no platform headers — the file compiles `-ffreestanding`
with only `<stddef.h>`/`<stdint.h>` pulled in, like the rest of
core/ under wasm32 `-nostdlib` (`core/toy_util.h:1-2`).
The one native-FP exception in the whole design is offline:
`tools/gen_sin_table.py` used host `math.sin` once to produce
integer constants that are now source, exactly the
structural-constants exception of `docs/CARTRIDGES.md:30-33`
and the same posture as the hand-typed `1/pi` bits at
`core/toy_math.h:46`. Justification: the FP ran once, on the
author's machine, and its outputs are frozen; no target
recomputes them, so no target can disagree about them.

Consequence: "(seed, scenario, rate, n)" and "these TSIG
bytes" are two names for the same stream, with the same
`stream_id`, on x86 host, RP2040, RP2350 and wasm32. The
prototype verifies the equality in-process (§8); the board
and WASM legs are T2 parcels SIG-3/SIG-4, gated the same way
existing carts are gated (`docs/CARTRIDGES.md:61-70`).

## 7. The labelling rule and the proof story

**Emulated vs measured must be explicit in state, on screen,
and in every proof line — and must never be silently
conflated.** Concretely:

1. `feed` copies `kind` + `orig_kind` + `stream_id` into
   hashed cartridge state. A synthetic run and a replay of a
   live recording *cannot* produce the same state hash even on
   identical samples, because the kind words differ inside the
   hashed bytes. `KIND_NONE` (no source attached) is a visible
   state of its own — a sensor cart with no source shows "NO
   SIGNAL", never a flat zero trace pretending to be a quiet
   desk.
2. `render` shows the provenance with the existing micro-font
   (`toy_fb_text`, `core/toy_fb.h:33`): `SYNTH`, `REPLAY-SYN`,
   `REPLAY-LIVE`, `LIVE`, `NO SIG`. This is the signal analogue
   of the banked-playback rule — a drawn frame must never
   claim to be a stepped state (`docs/TOY-0.md:68-72`); here,
   an emulated frame must never claim to be a measurement.
3. Proof headers gain the stream identity. The current line
   (`emu/main_emu.c:59`, `board/main_board.c:25-26`)
   `TOY-RUN cart=X state_size=N hash=fnv1a64` becomes, for
   signal carts only:
   `TOY-RUN cart=seismo ... sig=synthetic scenario=1
   seed=0x5eed rate=100 stream=333c0854a00a61df`
   (or `sig=replay orig=live stream=...`). The scrolling short
   state hash the toy already plans as its visible proof hook
   (`docs/TOY-0.md:71-72`) gains real meaning here: anyone can
   regenerate the stream from the printed seed and check it.

What a proof run for a sensor cart looks like (seismometer
example, assuming SIG-1's seam):

1. Descriptor names the proof signal alongside `proof_steps`
   (`core/toy_cart.h:30-31`): synthetic scenario 1, seed
   `0x5eed`, 100 Hz, 1 sample/step, `proof_steps = 6000`
   (60 s of desk: tilt + tremor + the tap at t = 30 s).
2. `tools/run.sh check`: emu runs it twice; both state-hash
   streams and the printed `stream=` id are byte-identical.
3. Site gate: the WASM checker runs the same (scenario, seed)
   and must print the same stream id and state hashes that
   are pinned in `site/expected-hashes.json`.
4. Board gate: firmware generates the same synthetic stream
   on-device (no sensor fitted or used) and must emit the
   same two hash streams over CDC. This is the strongest
   leg: it proves the RP2040 build of the integer generator
   matches the host's, sample for sample.
5. Once a kit exists, the same cart additionally gets a
   REPLAY proof: a short recorded clip (orig=live) embedded
   per §5.2. Synthetic proves breadth (faults, extremes);
   recorded proves the cart handles *real* bit patterns,
   including whatever the sensor did that the models didn't.

## 8. Prototype results (this parcel)

Files (all new; nothing existing modified):

- `core/toy_signal.h` / `core/toy_signal.c` /
  `core/toy_signal_sin.inc` — the source abstraction of §2/§5.
  `core/toy_signal.c` is picked up by the `core/*.c` glob in
  `tools/run.sh:19`; the replicated check below passes with
  zero warnings under `-O2 -std=c11 -Wall -Wextra`, and the
  file also compiles `-ffreestanding`.
- `tools/signal_proto.c` — standalone host program (own
  `main`, no edits to `tools/run.sh`; `tools/` is outside the
  emu/core globs). Build:
  `gcc -O2 -std=c11 -Wall -Wextra -Icore tools/signal_proto.c
  core/toy_signal.c -o build/signal_proto`
- `tools/gen_sin_table.py` — the offline table generator.

Output of `signal_proto build/signal_proto_replay.tsig`
(native MSYS2 gcc 16.1.0 on Windows host; run twice, outputs
and the written `.tsig` byte-identical):

```
== synthetic IMU: 60 s at 100 Hz, seed 0x5eed, scenario 1 (desk)
  run A hash 333c0854a00a61df
  run B hash 333c0854a00a61df          [PASS] identical twice
  seed 0x5eee hash 0a9259465c7da6ed    [PASS] seed changes stream
IMU statistics (integer: int64 sums):
  ax min -4288 max 4252 mean 1  mean|d| 52
  ay min -4285 max 4284 mean 2  mean|d| 49
  az min 10812 max 22723 mean 15867  mean|d| 58
  gx/gy/gz within +/-89, means 0
  |gravity| (tap window excluded): 16387 LSB = 1000 mg   [PASS]
  largest az deviation: 6859 LSB at sample 3001          [PASS]
  ax peak-to-peak 8540 vs 8192 tilt-only                 [PASS]
== replay equivalence: buffer and file both re-hash to
   333c0854a00a61df; truncated/bad-magic rejected        [PASS]
== microphone 1 s at 8000 Hz:
  440 Hz tone: hash 6f95cf9c0c81b4d9, 880 zero crossings [PASS]
  chirp 440->880: hash 28e936c7756b4684, 1320 crossings  [PASS]
signal_proto: PASS (0 failures)
```

Statistics are **integer-computed** (int64 sums, integer
square root; `tools/signal_proto.c` states this and uses no
native FP at all). Cross-checked independently by a Python
reader over the emitted file: payload FNV-1a recomputes to
`333c0854a00a61df`, |gravity| = 1000.19 mg, max az deviation
at sample 3001 — agreeing with the C program by a second
route.

Existing gates: `tools/run.sh check` was replicated command
for command (Git Bash on this host hits the known pre-existing
CRLF issue in `run.sh`; replication used MSYS2 gcc directly):
build of `core/*.c emu/*.c libs/muse-softfloat/src/*.c` with
the script's exact flags, then `run <cart> 10000 1000` twice
per cart with byte comparison — **PASS** for fig8 and
pendulum, no new warnings. `board` budget not run (per
parcel fence; SIG-4's job anyway).

Known prototype simplifications, stated plainly:

- Gyro channels carry tremor only; they are **not yet the
  derivative of the accel attitude**, so an attitude-ball
  fusion algorithm would see an inconsistent IMU. Making the
  gyro coherent (differentiate the tilt phasor in fixed
  point) is part of SIG-6.
- One IMU scenario (desk) and tone/chirp mic are coded;
  footsteps, car-pass and fault scenarios are specified
  (§5.1) but not implemented.
- The poll interface is the tagged-union form; the
  descriptor/vtable graduation is SIG-1.

## 9. Limits: what emulated signals cannot prove

- **The sensor itself.** Nothing here validates the
  QMI8658C's real noise density (the 150–200 µg/√Hz quoted in
  `docs/TOY-0.md:40`), its spectral colour, cross-axis
  sensitivity, zero-g offsets, temperature drift, or how its
  internal anti-alias filter phases a tap. Synthetic tremor
  amplitude (≈5 mg sd in the prototype) is a chosen test
  level, not a measurement of anyone's desk.
- **The clock.** Streams assume exactly 100.000 Hz. A real
  ODR has tolerance; a cart that integrates gyro or measures
  frequency will be off by that tolerance on hardware no
  matter how green its synthetic proofs are. Proof streams
  deliberately contain no timestamps, so they cannot even
  express jitter — jitter handling is a live-path concern
  (sequence gaps are the only timing signal the design
  admits, §4 rule 4).
- **The bus and the driver.** I2C/SPI throughput, FIFO
  watermark behaviour, missed samples under display load,
  codec gain staging and mic clipping character are all
  invisible to emulation. The fault scenarios (§5.1) can
  prove the *cart* reacts sanely; they cannot prove the
  driver produces the faults the same way.
- **The mounting and the room.** Case resonance, desk
  coupling, acoustic environment: only recorded replay
  (§5.2) brings these into the proof world, and only after
  the hardware exists to record them.
- **Scale assumptions.** 16384 LSB/g, 128 LSB/(°/s), 100 Hz,
  8 kHz are placeholders to confirm against datasheets and
  kit schematics. If they change, stream ids change — by
  design, loudly.

What emulation *does* prove, completely, before any hardware:
cartridge mathematics and rendering against known inputs
(including closed-form checks like |g| = 1000 mg and an
880-crossing 440 Hz tone), bit-exact reproducibility on every
target, per-sample cost, state-size discipline, labelling,
and behaviour under faults that hardware cannot be ordered
to produce.

## 10. T2 parcel breakdown

| parcel | deliverable | depends on |
|---|---|---|
| SIG-1 core seam | `feed` + sig fields on `toy_cart_t`; promote `toy_signal` poll to descriptor form; policy text into CARTRIDGES.md | toy1-input study (avoid two input seams) |
| SIG-2 scenarios | footsteps, car-pass, fault scenarios, gyro-coherent attitude model; scenario table pinned as proof vectors | SIG-1 (can start on prototype code) |
| SIG-3 host+WASM plumbing | emu `--sig`, proof-line stream ids, `run.sh check` extension; WASM sig buffer + Node gate | SIG-1 |
| SIG-4 board proof | `toy_signal.c` into `board/CMakeLists.txt`; on-device synthetic proof over CDC; one embedded TSIG replay proof | SIG-1, SIG-3 line formats |
| SIG-5 seismometer cart | tremor trace + event spikes on synthetic desk stream (time-domain first; spectrum when FFT lands) | SIG-1..4, toy1-fft for spectrum |
| SIG-6 attitude ball cart | gravity ball on gyro-coherent synthetic stream | SIG-2 gyro model |
| SIG-7 live IMU | QMI8658C driver as LIVE source + TSIG tee recorder; first orig=live replay vectors replace synthetic-only proofs | kit arrival, SIG-1 |
| SIG-8 live mic | codec driver as LIVE source + recorder; mic spectrum cart proofs gain a recorded leg | RP2350 kit, toy1-fft |

Unblocks, per `docs/WAVE-T1.md:37-40`: desk seismometer
(signals + FFT), attitude ball and real-gravity pendulum
(sensor input), mic/synth spectrum (FFT + this).

## 11. Owner questions

1. Defaults: is 100 Hz IMU / 8 kHz mic the right baseline,
   and ±2 g (finer resolution, taps may saturate) vs ±4 g
   (coarser, taps survive) the right accel trade for a
   *desk* object? The prototype assumes 100 Hz / 8 kHz / ±2 g.
2. May replay proof clips live in the repo (a few 10–100 KB
   `.tsig` files), or should proofs stay synthetic-only in
   git with recordings kept elsewhere? (Affects SIG-4/SIG-7.)
3. Mic recordings capture the room: should orig=live mic
   vectors ever leave the device, or is the tee recorder for
   mic limited to on-device/microSD by policy?
4. Proof scenario seed `0x5eed` is arbitrary — any preferred
   convention for pinned seeds before SIG-2 freezes several?

## Verdict

Feeding emulated signals works, is cheap, and is the only way
sensor carts can join the host == board == WASM proof culture
before the kits arrive — provided the labelling rule (§7) is
treated as non-negotiable and the limits (§9) are quoted
wherever a synthetic proof is quoted. Recommend: take SIG-1
jointly with the input seam, then SIG-2..4 as one proving
wave, and build the seismometer against the synthetic desk
stream immediately after.

## Appendix (Wave T3): scenario 2 "chest"

A second IMU6 synthetic scenario, added additively for the
`heartbeat` cartridge (a seismocardiography *toy* - not a
medical device, and labelled as such in its render). The
scenario-1 desk stream bytes are unchanged by its presence
(signal_proto scenario-1 hash is still pinned); scenario 2 is
a new proof vector in the sense of section 5, served by
`toy_sig_synth_imu_chest_init` / `toy_sig_source_synth_imu_scenario`
in `core/toy_signal.{h,c}` and selected through the unchanged
descriptor seam (`toy_cart_check_signal` now allows scenario
{1, 2}; `toy_sig_proof_source_init` dispatches, and refuses
device/scenario combos with no generator such as MIC
scenario 2).

Model (integer only, desk-generator style; device flat on the
sternum, screen up, so in this frame gravity rests in +az at
16384 LSB and the SCG out-of-plane direction is az):

- breathing ~0.25 Hz at ~12 mg (197 LSB) on az, small
  fractions spilling into ax/ay;
- a heartbeat pulse train at exactly 75 BPM (period
  4*rate/5 samples = 80 at 100 Hz): an SCG-like doublet from
  integer per-mille shape tables - systolic complex ~12
  samples wide, peak 82 LSB (~5.0 mg), diastolic bump ~40%
  (33 LSB) at +30 samples - plus pulse/4 and -pulse/6
  spillover on ax/ay;
- uniform +-2 LSB noise on all six axes (a quantisation-class
  floor, deliberately *below* real sensor noise - see below);
- the seed varies breathing/beat phase and noise only; the
  first systolic lands one period in plus seed phase.
  `toy_sig_info` reports kind/orig SYNTHETIC, scenario_id 2.

The magnitudes are modelled on published seismocardiography
(chest accelerations order g*1e-3, typical features ~5 mg,
placement-dependent roughly 1-20 mg) and the QMI8658C
datasheet noise density (150 ug/sqrt(Hz), ~0.56 mg RMS over a
1-15 Hz band; 61 ug/LSB at +-2 g/16-bit): the expected
hardware SNR is workable on paper, but this vector proves the
*detector chain*, not the physiology or the sensor. What
remains for real hardware to confirm: placement pressure and
clothing coupling, actual noise at the chosen ODR/range, the
real module's axis orientation, and gross-motion behaviour
(none is modelled here). Per section 7, every run is labelled
SYNTHETIC CHEST from hashed state, and NO SIG before the
first feed.
