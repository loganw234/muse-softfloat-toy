# Wave T1 — first cartridge wave + seam studies

Opened 2026-11-03 by owner election ("dispatch parcels for each
which can be made now, along with the small studies, as well as
one to see if we could feed emulated signals to test with").
Base: main at 597173f. Every parcel follows docs/CARTRIDGES.md;
merges are lead-sequenced with planted Spark verification and the
board proof, as in the library project.

## Implementation parcels (fit the current contract as-is)

| branch | cartridge | one-line |
|---|---|---|
| toy1-cloud | `cloud` | N figures-8 perturbed by k ulps: the divergence cloud |
| toy1-carousel | `carousel` | choreography carousel, auto-advancing each period |
| toy1-orrery | `orrery` | Keplerian solar-system orrery clock |
| toy1-lorenz | `lorenz` | Lorenz attractor, slow projection rotation |
| toy1-harmonograph | `harmonograph` | damped coupled-pendulum Lissajous drawing |
| toy1-pendwave | `pendwave` | pendulum-wave ring, phase interference |
| toy1-mandelbrot | `mandelbrot` | binary32 vs binary64 split deep-zoom |
| toy1-microscope | `microscope` | function microscope: scripted zoom to the ulp staircase |
| toy1-stagnation | `stagnation` | f16/f32/f64 accumulation freeze + augmented resurrection |
| toy1-intervallens | `intervallens` | scripted interval-geometry lens (certified vs naive) |

## Studies (design + recommendation; code only where named)

| branch | study | unblocks |
|---|---|---|
| toy1-input | Input & settings seam (touch/encoder/IMU events, deterministic injection) | gravity sandbox, Julia-by-touch, all interactive carts |
| toy1-signals | Emulated signal feeding: synthetic/replayed sensor streams as test inputs (prototyped) | seismometer, spectrum carts, sensor testing before hardware |
| toy1-timebank | Time layer: wall-clock abstraction, pacing/banking buffers, morning-report data | overnight banking, active hours, morning report |
| toy1-checkpoint | Checkpoint format & versioning (state bytes + header, flash layout) | power-loss resume on device |
| toy1-fft | FFT utility: own radix-2 in binary64 via sinPi/cosPi twiddles, sizes/cost measured | orbital spectrum, synth spectrum (Wave T2) |

## Deferred until their seam lands

Gravity sandbox and Julia-by-touch (input), real-gravity pendulum
and attitude ball (sensor input), desk seismometer (signals + FFT),
orbital & synth spectra (FFT), morning report (timebank). These
are Wave T2 candidates, not omissions.
