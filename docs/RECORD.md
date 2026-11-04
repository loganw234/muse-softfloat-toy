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
