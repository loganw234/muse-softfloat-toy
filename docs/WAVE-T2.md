# Wave T2 — seams live: input, signals, time, checkpoints, spectra

Opened 2026-11-04 on the owner's Wave T1 close-out decisions.
Base: main at 9b74fa8 (Wave T1 closed, 12 cartridges live).

## Owner decisions (2026-11-04)

- **Primary device: RP2350-Touch-LCD-1.85C (360x360)** — added
  sensors and microSD. The RP2040 boards remain the proof
  target until the kit is in hand; the sim core stays
  soft-float-identical across both.
- **Power: always plugged in.** No battery/sleep design in T2;
  checkpointing still protects against unplug/power loss.
- **Time: manually set** (time + zone offset set by the user
  on the touchscreen; no network time). Clock-unset is a
  first-class state until set.
- **Warp: no preference yet** — implement parameterised, pick a
  sane default, expose as a setting.
- **Interaction: touchscreen only** (self-contained unit - no
  encoder/buttons), **plus IMU motion where applicable,
  including turning the whole device as a rotary dial**
  (a derived dial gesture from the gyro).

## Interface decisions (lead, from the T1 studies)

- Events follow study-input: appended descriptor fields
  (study section 5.3 shape), optional pre-step `apply()`,
  32-byte integer `toy_event_t`, step-boundary quantisation,
  `SET_PARAM` settings, compiled-in `proof_inputs[]` scripts.
  Existing carts are `NULL`/zero-filled: zero hash churn is a
  merge gate.
- The event vocabulary includes a **dial event** (integer
  milliturn delta) for the turn-the-device gesture; its
  producer (IMU layer) lands with signals/board work, the
  vocabulary lands now.
- Raw sensor streams are **not** events: signals-core adds a
  separate optional feed mechanism in T2b, designed against
  the landed input seam (SIG-1 in study-signals).
- Checkpoint descriptor tail: input fields first, then
  `state_version`; signals fields append after those in T2b.
- Genuine study errata found by T1 verifiers are T2 inputs,
  not blockers (field-count wording, helper ownership,
  drain-fate detail, timebank units/DST passage, checkpoint
  wrapper wording). T2 briefs state the corrected reading.

## Tranche A (dispatched on this plan)

| branch | parcel | one-line |
|---|---|---|
| toy2-inputcore | input seam implementation | apply()/events/settings/proof-scripts in core + all 3 runners; zero-churn gate |
| toy2-ckpt | checkpoint core | TOYCKPT1 format + state_version + emu save/load + resume-continuity proofs |
| toy2-orbitspec | `orbitspec` cartridge | fig8 coordinate spectrum via toy_fft (first FFT consumer; adds toy_fft.c to board firmware) |

## Tranche B (after A lands)

signals-core (feed seam + synthetic sources in runners),
clock-core (toy_clock + keyframe banking/playback + morning
report v1), cartridges `julia` (touch-c), `gravity`
(sandbox), `gpend` (IMU-gravity pendulum), `attitude`
(IMU ball/horizon), `seismo` (synthetic tremor + FFT),
`synspec` (synth spectrum), and an RP2350 build-readiness
parcel (compile-proven; on-device proof when the kit lands).
Display/touch/IMU drivers wait for the physical kit.

## Method

Unchanged from T1: parcels in worktrees, planted-copy
independent verification (`docs/metrics/plants.jsonl`)
before merge, lead check/WASM/board gates per landing,
proof horizons sized by measured board step times.
