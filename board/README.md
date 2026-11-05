# Board firmware

Core-only proof firmware (`main_board.c` + `clock_board.c` + the
registered cartridges + `libs/muse-softfloat`): runs every
cartridge's proof stream headless and prints state hashes over
USB CDC. No display/touch/IMU drivers yet; those land with the
RP2350-Touch-LCD-1.85C kit (`docs/WAVE-T2.md`).

## Platform selection

One knob, default = the historical RP2040 build:

```sh
# RP2040 (default), SDK platform rp2040, board pico
cmake -S board -B build/board-rp2040 -G Ninja \
  -DPICO_SDK_PATH=<pico-sdk> -DCMAKE_BUILD_TYPE=Release
cmake --build build/board-rp2040

# RP2350 (Cortex-M33), SDK platform rp2350 -> rp2350-arm-s, board pico2
cmake -S board -B build/board-rp2350 -G Ninja \
  -DPICO_SDK_PATH=<pico-sdk> -DCMAKE_BUILD_TYPE=Release \
  -DTOY_PICO_PLATFORM=rp2350
cmake --build build/board-rp2350
```

Rules (`board/CMakeLists.txt` is the authority):

- Use a fresh build directory per platform; switching platform
  inside one directory is refused.
- Passthrough: `-DPICO_PLATFORM=rp2040|rp2350` (or the env var)
  without `TOY_PICO_PLATFORM` selects identically, and a lone
  `-DPICO_BOARD=<board>` lets the SDK derive the platform from
  the board header. `TOY_PICO_PLATFORM` in the cache always ends
  up naming the platform actually built (see the
  `toy_board: TOY_PICO_PLATFORM=...` configure line). Combined
  knobs must agree.
- `rp2350-riscv` and `host` are rejected: TOY-0 targets the
  Cortex-M33 pair; the RISC-V cores/toolchain are unaudited.
- `PICO_HARD_FLOAT_ABI=1` is rejected unless RP2040 is selected
  (where the SDK ignores it): on RP2350 it would route native
  float/double through the hardware FPU (see below).

The RP2040 configuration is unchanged by this selection layer:
same source list, same SDK defaults, and (verified 2026-11-04)
a bit-for-bit identical UF2/BIN to a build of the pre-knob
`CMakeLists.txt` at the same commit.

`board/run_toy.sh` is deliberately untouched: it is the RP2040
board gate (flashes the genuine Pico by serial under the shared
board lock) and relies on the default above. An RP2350 on-device
gate wants its own serial/lock/capture flow when the kit is in
hand, not a flag on the RP2040 script.

## Float/double discipline (why RP2350 is pinned)

The sim is muse-softfloat integer C on both chips; the RP2350's
SP FPU and double coprocessor (DCP) must never run cartridge
maths (TOY-0 section 2). Left alone, the SDK would substitute
hardware paths for any native float/double in the binary:

| SDK switch (default) | RP2040 | RP2350 default | RP2350 here |
|---|---|---|---|
| compiler ABI | `-mfloat-abi=soft` | `-mfloat-abi=softfp` | `softfp` overridden by `-msoft-float` (via float=none) |
| `pico_float` (`pico`) | ROM routines | `pico_float_pico_vfp` (SP FPU) | `none` |
| `pico_double` (`pico`) | ROM routines | `pico_double_pico_dcp` (DCP) | `compiler` (libgcc) |
| `pico_printf` (`pico`) | `_ftoa`/`_etoa` on `double` | same, via DCP | same, via libgcc |

The pins are per-target (`pico_set_float_implementation` /
`pico_set_double_implementation` in `CMakeLists.txt`), so they
override `PICO_DEFAULT_FLOAT_IMPL`/`_DOUBLE_IMPL` for
`toy_board`. The RP2040 side needs no pin (its ROM routines are
already software) and is not touched.

Audit of what actually uses native float/double in this binary
(object undefined-symbol scan + disassembly call-site scan,
2026-11-04, SDK 2.3.1):

- Cartridge `reset`/`step`/`apply`/`feed` and all of
  `libs/muse-softfloat`: no `__aeabi_f*`/`__aeabi_d*` calls,
  only `msf_*`. Nothing in `core/` or `board/` uses `float`.
- Cartridge `render()` functions: native `double` display
  scaling only (state -> pixels; never feeds back into state).
  These are the only cartridge-side `__aeabi_d*` callers.
- SDK `printf.c` `_ftoa`/`_etoa`: `double` formatting. No format
  string in this firmware uses `%f`/`%e`/`%g`, so it is dead at
  runtime but linked; pinned to libgcc software like the rest.

Consequences of the chosen pins:

- `float=none`: no SP consumer exists, so SP calls fail closed
  (the SDK wraps them to a panic stub) and its `-msoft-float`
  stops the compiler emitting VFP instructions anywhere in this
  target. With the SDK default the compiler even used `vldr d7`
  as a 64-bit move inside integer code; that is gone.
- `double=compiler`: `render()` and `printf %f` keep working
  through libgcc software routines
  (`thumb/v8-m.main+fp/softfp/libgcc.a(_arm_*df3.o)`), instead
  of the DCP wrappers that the unpinned build links
  (`__wrap___aeabi_dadd` disassembles to `dcp_add0` etc.).
- Verified in the pinned RP2350 ELF: zero VFP instructions and
  zero DCP arithmetic instructions in the disassembly. Two
  honest residuals: (1) the ELF still advertises
  `Tag_FP_arch: FPv5` because the SDK compiles for
  `-march=armv8-m.main+fp+dsp` (the ABI stays soft-float);
  (2) SDK boot code (`runtime_init_per_core_enable_coprocessors`)
  still enables CP10/CP4/CP0 at startup and issues one
  `dcp_rcmp` to clear the DCP engaged flag. That initialisation
  is left alone on purpose: `hardware_gpio` on RP2350 uses the
  GPIO coprocessor (CP0, e.g. `mcrr 0,4,...` in `main`), so
  skipping the enable would fault. Enabling a coprocessor is
  not doing maths with it; nothing calls it. If a future
  hardening pass wants the enable gone too, pair
  `PICO_RUNTIME_SKIP_INIT_PER_CORE_ENABLE_COPROCESSORS=1` with
  `PICO_USE_GPIO_COPROCESSOR=0` and re-prove on hardware.

Sizes at the audited commit (Release, `-O2` target option):
RP2350 flash image 130,900 B (`.text` 108,244 + `.rodata`
16,532 + `.data` load image 6,060 + metadata) of the 4 MiB the
`pico2` header assumes (the kit has 16 MiB); RAM 88,988 B
(`.data` 6,060 + `.bss` 82,656 + vector table 272) of 520 KiB,
plus 2 KiB heap and a 2 KiB stack in scratch. RP2040 image
135,036 B of 2 MiB, RAM 86,564 B of 256 KiB. Pinning cost vs the
unpinned RP2350 build: +1,176 B of text (libgcc double in
flash/RAM instead of DCP stubs).

## API notes for RP2350 (compile-proven only)

- `PICO_DEFAULT_LED_PIN` is 25 on both `pico` and `pico2`, so
  `main_board.c` needs no change for the default RP2350 build.
  But SDK 2.3.1 has no `waveshare_rp2350_touch_lcd_1.85` board
  header; the closest in-tree header
  (`waveshare_rp2350_touch_lcd_1.28`) defines no LED pin at all
  and uses GPIO25 as the LCD backlight. Building against it
  fails today on `PICO_DEFAULT_LED_PIN` undeclared - the first
  kit-day job is a real 1.85C board header (LED or none, 16 MiB
  flash, display/touch/IMU/RTC pins) and checking what GPIO25
  is wired to on the 1.85C before flashing a `pico2` UF2 that
  toggles it.
- `clock_get_hz(clk_sys)`: same API; the RP2350 default is
  150 MHz (RP2040: 125 MHz). The `TOY-BOARD clk_hz` line will
  show it; nothing in the proof depends on the rate.
- `pico_get_unique_board_id_string` / `PICO_UNIQUE_BOARD_ID_SIZE_BYTES`:
  same API and 8-byte ID, but the source differs - flash unique
  ID on RP2040, bootrom `SYS_INFO_CHIP_INFO` on RP2350. Serial
  strings are therefore not comparable across chips; record the
  kit's serial from its first banner (or `picotool info`).
- `get_absolute_time()`/`to_ms_since_boot()` (`clock_board.c`),
  `gpio_xor_mask`, USB stdio: unchanged APIs on RP2350.

## Internal-flash checkpoint slots (compile-proven, not yet wired)

`ckpt_flash.{h,c}` + `core/toy_ckpt_slot.h` is the resume-last
checkpoint backend (`core/toy_ckpt.h` images, at most 65,600 B).
It is linked into `toy_board` (symbols rooted against
`--gc-sections`) but `main_board.c` never calls it: no boot
behaviour changed, and the proof hash stream is untouched.
Nothing here has run on a board; the first on-device proof is
the checklist below. Pure slot logic (geometry, newest-valid,
rotation) is host-tested against a RAM fake NOR in
`tools/flashslot_test.c`.

Slot design (`docs/study-checkpoint.md` 5.1, A/B form): each
slot is sized for the format cap, not today's largest cart -
`ceil(65600 / 4096) * 4096 = 69,632 B` (17 sectors) - and holds
one TOYCKPT1 image verbatim at offset 0, padded with 0xFF to a
256 B program-page multiple, no wrapper bytes. Torn writes are
caught by the image's own header/payload CRCs + state hash in
`toy_ckpt_validate()`. Selection: among fully valid slots for
the expected cartridge, the highest header `step_count` wins
(ties: lowest slot); blank (0xFF) and torn slots are skipped.
The next write always targets the slot *after* the newest
valid one, so a power cut mid-write leaves the previous
checkpoint intact. One honest limitation: `step_count` restarts
on cartridge switch, so scans must be cartridge-scoped (the
backend's `write_next` scopes rotation to the image's own
cart); whether a cart switch erases the slots is wiring policy,
not backend policy. Writes go through the SDK's
`flash_safe_execute` (interrupts/core safety is the SDK's, with
`PICO_FLASH_ASSUME_CORE1_SAFE=1` because this firmware is
single-core), erase the whole slot, program, then read back
via XIP and re-validate byte-for-byte. The image buffer must
be in SRAM. The backend imposes no save-rate policy - triggers
(events + 15 min backstop, study 5.1) are the caller's duty,
called between frames: a cap-size write is 17 erases + 257
page programs with XIP stalled (fig8: 17 + 55).

Layout derives from `PICO_FLASH_SIZE_BYTES` (board header),
slots at the END of flash. Concrete maps (offsets are flash
offsets; add `0x10000000` for XIP addresses):

| flash | region base | slot 0 | slot 1 | region end |
|---|---|---|---|---|
| 2 MiB (Pico/RP2040 dev) | `0x1DE000` | `0x1DE000..0x1EF000` | `0x1EF000..0x200000` | `0x200000` |
| 4 MiB (`pico2` default) | `0x3DE000` | `0x3DE000..0x3EF000` | `0x3EF000..0x400000` | `0x400000` |
| 16 MiB (RP2350 module) | `0xFDE000` | `0xFDE000..0xFEF000` | `0xFEF000..0x1000000` | `0x1000000` |

The pair reserves 139,264 B: 6.6% of 2 MiB, 0.83% of 16 MiB.
Firmware fits far below it: the default RP2040 BIN is 152,668 B
against a 1,957,888 B base - always re-check `BIN size <
layout->base_offset` when the firmware grows. Catch for the
module: today's RP2350 build uses the `pico2` header, which
*assumes* 4 MiB, so its slots land at the 4 MiB end (correct for
that header, mid-device on the 16 MiB part). The 16 MiB row
above becomes real only with a board header defining
`PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)` - part of the 1.85C
board-header job below.

Wear (assumes the W25Q-class 100k erase cycles/sector - verify
against the fitted parts; every save to a slot erases all 17 of
its sectors once, and rotation halves the rate per slot):

| saves/day | shape | per-slot erases/day | time to 100k cycles |
|---|---|---|---|
| 4 | event-only, light use | 2 | ~137 years |
| 20 | event-heavy (cart switches) | 10 | ~27 years |
| 96 | 15 min backstop, always stepping | 48 | ~5.7 years |
| 288 | 5 min backstop | 144 | ~1.9 years |
| 1,440 | every minute | 720 | ~139 days |

So per-frame or per-minute checkpointing kills the part; the
study's events-plus-15-minute policy gives years of margin.
Widening to the 4-slot ring `toy_ckpt_slot.h` already supports
(278,528 B, base `0xFBC000` on 16 MiB) halves the rates again
if a tighter resume window ever matters on the module.

Internal slots vs SD: internal flash holds exactly one thing -
resume-last for the running cartridge, available with no card
present. Per-cartridge banks, a checkpoint gallery and history
belong on the RP2350 kit's microSD as generation-named
`.ckpt` files (study 5.2); the same TOYCKPT1 bytes serve both,
and SD must never gate internal-flash resume.

First on-device proof (Friday; deliberately not done by this
compile-only parcel):

1. Confirm the fitted flash: sector/page geometry and rated
   erase cycles vs the assumptions above (also why
   `ckpt_flash.c` static-asserts the SDK sizes).
2. One write/read proof: throwaway build with
   `-DTOY_CKPT_FLASH_SELFTEST=ON` plus a temporary boot call to
   `toy_ckpt_flash_selftest_roundtrip()` around a real
   `toy_ckpt_serialize()` image; print the result, the written
   slot, `toy_ckpt_flash_last_safe_rc()`, and the read-back
   `state_hash` next to the host hash for the same step count
   (they must match). Remove the call and rebuild default-OFF
   afterwards - a boot write per power-up is wear for nothing.
3. Power-cut test: loop `toy_ckpt_flash_write_next()` with
   increasing step counts, yank power mid-loop several times,
   and confirm boot always resumes a fully valid earlier image,
   never the torn one.
4. Confirm a fresh `picotool load` of a new UF2 leaves a saved
   slot readable (expected - UF2 writes addressed blocks only -
   but unproven here), and re-check BIN size vs the slot base.
5. Then the real wiring: resume-at-boot + event/backstop save
   triggers in the live firmware's frame loop.

Build evidence for this backend (Release, fresh dirs,
2026-11-04): RP2040 UF2 305,664 B (BIN 152,668 B), RP2350 UF2
299,008 B (BIN 149,248 B). The RP2040 delta against the same
commit built without the backend (UF2 297,984 B, BIN 148,756 B)
is +7,680 B UF2 / +3,912 B BIN, +56 B BSS - the backend objects
only; the UF2 is expected to differ from current `main` for
exactly that reason.

## When the kit arrives (not done here - compile proof only)

Nothing was flashed for this work. Expected first steps:

```sh
picotool info -a build/board-rp2350/toy_board.uf2   # inspect only
picotool info                                      # find the kit's serial in BOOTSEL
picotool load --ser <RP2350-SERIAL> -f -x build/board-rp2350/toy_board.uf2
```

`picotool load` has the same shape as the RP2040 gate (`--ser`,
`-f`, `-x`); RP2350 UF2s carry family `rp2350-arm-s` plus
picobin metadata, so a wrong-family file is refused rather than
misflashed. Then: read the hash stream over CDC exactly as
`read_toy.py` does and diff against the host - host == RP2040 ==
RP2350 at every reported step is the cross-chip gate TOY-0
promises. After that, in kit order: board header (above), RTC
backing for `clock_board.c` (replacing the unset stub),
ST77916 display, CST816 touch, QMI8658-class IMU, microSD.
