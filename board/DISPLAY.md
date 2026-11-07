# Display bring-up: CO5300 on RP2350-Touch-AMOLED-1.75 (D0)

Scope: lite-the-panel only. `toy_display_test` is a standalone
bring-up firmware, separate from the `toy_board` proof. It does
not wire the panel into cartridges/`toy_fb` yet; the banded
presenter is a later parcel. The driver API in
`display_co5300.h` (`set_window` + `write_pixels` + `fill`)
already supports band-shaped updates.

Status: compile-proven only. Nobody has seen this panel light
with this code yet; first flash is the lead's, with the owner
watching the glass.

## Hardware facts and where each came from

Board: Waveshare RP2350-Touch-AMOLED-1.75 (RP2350A), 466x466
CO5300 AMOLED on QSPI, CST9217 touch on I2C.

Sources: [Waveshare product docs](https://docs.waveshare.com/RP2350-Touch-AMOLED-1.75)
and its [resources page](https://docs.waveshare.com/RP2350-Touch-AMOLED-1.75/Resources-And-Documents)
(schematic PDF, CO5300 datasheet, AXP2101 datasheet), plus the
official example repo
[waveshareteam/RP2350-Touch-AMOLED-1.75](https://github.com/waveshareteam/RP2350-Touch-AMOLED-1.75)
consulted only for board-specific hardware facts (pin defines,
init register values, column offset). All code here is original.

Pin map (vendor pinout, schematic nets `LCD_*` at panel
connector J3 `FH26-31S-0.3SHW`, example `lib/QSPI_PIO/qspi_pio.h`):

| Signal | GPIO |
|---|---|
| LCD_CS | 10 |
| LCD_SCL | 11 |
| LCD_SIO0 | 12 |
| LCD_SI1 | 13 |
| LCD_SI2 | 14 |
| LCD_SI3 | 15 |
| LCD_RESET | 16 |
| LCD_TE | 17 |

Shared I2C1 is SDA GPIO6 / SCL GPIO7 (schematic; example
`lib/Config/DEV_Config.h`: `I2C_PORT i2c1`, pins 6/7, 400 kHz).
GPIO25 is UART1 RX on this board — it is never an LED; board
builds for this kit pass `-DTOY_BOARD_LED_PIN=-1`.

### Power: no AXP2101 setup needed, and none is done

Evidence, in triage order:

1. Schematic: panel connector J3 carries `VCC3V3` (two pins) and
   GND as its only supply nets — no AXP2101 LDO output
   (`ALDO*`/`BLDO*`/`VL*` nets) reaches J3. The schematic rail
   table assigns `DCDC1 -> VCC3V3`. `VCC3V3` also powers the
   RP2350's IOVDD, so if firmware executes at all, the panel
   supply is already up.
2. CO5300 datasheet (s4.1, p11): VDDI 1.65–3.3 V and
   VCI 2.7–3.6 V; a single 3.3 V feed on the panel FPC satisfies
   both.
3. The official C example (`examples/C/01_GUI/main.c`) runs
   `AMOLED_1IN75_Init()` + a full-screen clear with zero AXP2101
   writes before or during display init; its only AXP2101 write
   comes after, setting battery charge current. Its
   `PIN_PWR_EN` is `-1` (no panel power-enable GPIO).

So this parcel writes nothing to the AXP2101 — no rail setup, no
charge/shutdown policy, nothing. Do not add AXP writes to fix a
black screen before exhausting the diagnostic order below.

### Wire protocol (CO5300 datasheet s5.2, p20–21)

- Register write: instruction `02h`, address `{00h, REG, 00h}`,
  parameter bytes — all 1-wire on SIO0, MSB first, one CS.
- Pixel write: instruction `32h`, address `{00h, 2Ch, 00h}`
  1-wire, then pixel bytes 4-wire in the same CS assertion.
- RGB565 byte order: high byte (`RRRRRGGG`) first, then low
  byte (`GGGBBBBB`); each byte high nibble first, nibble bit j on
  pin SIj. Callers pass host-order `uint16_t` (as in
  `core/toy_fb.h`); the driver orders the bytes.
- Window: glass coords 0..465 map to RAM columns +6 (6..471),
  rows +0 — the vendor example's `AMOLED_1IN75_SetWindows`
  offsets for this 466-wide glass on the wider CO5300 source
  RAM. The border pattern exists to convict this if wrong.
- SCL cap: datasheet s6.4.1 (p107) Note 3 limits 4-wire QSPI
  pixel transfers to 50 MHz. The vendor example exceeds this
  (200 MHz system overclock, undivided PIO — see Report
  discrepancies). D0 runs 25 MHz at the stock 150 MHz system
  clock.

### Init sequence

`display_co5300.c` carries the table with a citation per row.
Order and values are the vendor example's `AMOLED_1IN75_InitReg`
(validated on this panel by the vendor), each value decoded
against the datasheet: `11h` SLPOUT (+120 ms), `C4h=80h` SPI
SRAM-write enable (load-bearing: default 00h disables pixel
writes, s7.5.57 p216), `44h=01D7h` TE scanline 471 (s7.5.35),
`35h=00h` TEON, `53h=20h` brightness-control block on (s7.5.40),
`29h` DISPON, `51h` brightness DBV (value is ours: init takes it
as an argument instead of the vendor's A0h-then-FFh),
`20h` INVOFF, `36h=00h` MADCTL (no mirror, RGB order),
`3Ah=05h` COLMOD RGB565 (s7.5.32, p176).

Two deliberate divergences from the vendor example, both
electrically inert: CS is released before the 120 ms post-SLPOUT
wait (vendor sleeps holding CS), and `set_window` does not issue
the vendor's empty 1-wire RAMWR after CASET/RASET (the 32h
pixel-write header re-issues RAMWR with the pixels).

## Build

Fresh RP2350 build dir (the new target exists only there):

```sh
cmake -S board -B build/board-rp2350-disp -G Ninja \
  -DPICO_SDK_PATH=<pico-sdk> -DCMAKE_BUILD_TYPE=Release \
  -DTOY_PICO_PLATFORM=rp2350 -DTOY_BOARD_LED_PIN=-1
cmake --build build/board-rp2350-disp
# -> build/board-rp2350-disp/toy_display_test.uf2 (and toy_board.uf2)
```

Host framing test (from repo root):

```sh
gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard \
    tools/display_test.c board/display_co5300.c -o build/display_test
./build/display_test   # prints: display_test: PASS
```

Flashing (lead only, by serial, as with `run_toy.sh`):
`picotool load --ser <RP2350-SERIAL> -f -x toy_display_test.uf2`,
then watch USB CDC for the `TOY-DISP-*` stream.

## What the owner should see

Serial phases: `TOY-DISP-BOOT` → per-device `TOY-DISP-I2C
addr=0x..` lines + `TOY-DISP-I2C-SCAN-DONE` (read-only scan;
expect the roster of the shared bus — touch, IMU, RTC, PMIC,
codec) → `TOY-DISP-QSPI scl_hz=25000000 ...` → `TOY-DISP-INIT
rc=0` → black fill with a `TOY-DISP-TIME ... full_frame_us=`
line (expect roughly 35–45 ms at 25 MHz quad; a full frame is
466x466x2 bytes) → `TOY-DISP-TE edges=...` (a scanning panel
pulses TE about once per frame, so a 300 ms window should count
a few dozen edges).

Then at modest brightness (~1.2 s each): solid red, green, blue,
grey. Then quadrants: top-left red, top-right green, bottom-left
blue, bottom-right yellow. Then a white border on the glass
rectangle's edges with a cyan border 8 px inside it (corners
fall off the round glass; judge the edge midpoints). Then a
white centred cross with a magenta disc at the centre. Finally
border + cross held dim for 20 s, then `TOY-DISP-OFF` and the
panel goes dark. Total lit time ≈ 35 s, never a sustained full
white.

## If the screen is black: diagnostic order

1. Is there a serial stream at all? `TOY-DISP-BOOT` proves the
   firmware runs (which, per the power evidence above, also
   proves the panel's VCC3V3 rail is up). No stream: USB/flash
   problem, not a panel problem.
2. I2C roster sane? `TOY-DISP-I2C-SCAN-DONE count=` should be
   several. Zero devices would say the board's shared bus (and
   likely its power tree) misbehaves — stop, that's bigger than
   the display.
3. `TOY-DISP-INIT rc=0` present but glass black, and
   `TOY-DISP-TE edges=0`: the panel likely never accepted the
   command stream. Suspects in order: CS/SCL/data pin order,
   the 1-wire expansion (host-tested, so likelier a PIO config
   slip), reset level. TE stays inactive in sleep-in, so also
   confirm SLPOUT landed.
4. `TOY-DISP-TE edges=` healthy but glass black: the panel
   scans; the stream landed but pixels aren't showing. Suspects:
   `C4h=80h` (without it pixel writes are ignored by design),
   brightness actually written, COLMOD.
5. Colours wrong: red/blue swapped in quadrants → MADCTL RGB
   bit; mirrored quadrants → MADCTL MY/MX. One-byte fixes in
   `INIT_STEPS`.
6. Image shifted or wrapped: border midpoints uneven → the +6
   column (or +0 row) offset is wrong; adjust
   `CO5300_COL_OFFSET`/`CO5300_ROW_OFFSET` and re-run the host
   test (it pins the expected RAM coordinates).
7. Sparkle/garbage only at speed: drop `DISPLAY_QSPI_SCL_HZ`
   (25 MHz → 12.5 MHz) before suspecting anything else.

## Not in this parcel

No touch, no TE-synchronised updates (TE is only observed), no
DMA, no banded `toy_fb` presenter, no shell wiring, no AXP2101
anything, no read path (`03h` reads of panel ID/status are the
natural next diagnostic if D0 stays black).

## Addendum (toy5-presenter): product presenter + attract

Status update: the panel is glass-proven with the D0 code above
(466x466, full-frame flush 34.8 ms at 25 MHz QSPI, column
offset +6, zero AXP writes). The presenter forecast as "banded"
above did NOT land banded - see the framebuffer decision below.

`toy_product` (`board/main_product.c` + `board/presenter.*`,
RP2350-only CMake target appended after `toy_display_test`) is
the first on-glass product firmware. It boots the panel at a
constant DBV `0x50`, then cycles all 23 registered cartridges
in registry order, 20 s of wall clock each
(`TOY_PROD_CART_SECONDS`), at each descriptor's demo steps per
frame, forever. Serial markers: `TOY-PROD-BOOT`,
`TOY-PROD-PRESENTER`, `TOY-PROD-MEM` (linker-symbol memory
evidence), `TOY-PROD-CART` per switch, and once per second
`TOY-PROD-PACE cart= sim_us= render_us= flush_us= frames=
seconds=` (window totals, integer prints only). Runner posture
is live, not proof: feed carts use the synthetic on-device
source in `feed -> apply -> step` order exactly as
`main_board.c`; the apply slot drains an always-empty live
queue (no touch/IMU/RTC/shell yet); proof scripts are not
replayed except step-0 parameter setup, mirrored from emu
frames mode. The framebuffer persists across frames within a
cart (the emu runner contract) and is cleared, with a black
glass fill, only on cart switches.

Framebuffer decision, by measurement (full table in the parcel
report): a full 466x466 RGB565 framebuffer (434,312 B) plus the
65,536 B cart state buffer does not fit RP2350 main RAM - the
same source built with `-DTOY_PRESENTER_MODE_FULL466=1` fails
to link ("region RAM overflowed by 2080 bytes"; .bss
518,496 B against the 512 KiB region, core-0 stack reserved
separately as 4,096 B in SCRATCH_Y via `PICO_STACK_SIZE=4096`).
Shipped mode is therefore a 360x360 framebuffer (259,200 B -
the size the emulator renders) upscaled to the glass during
flush: centre-aligned nearest neighbour,
`src(d) = ((2d+1) * 360) / (2*466)` in integer math on both
axes, pinned pixel-for-pixel by `tools/presenter_test.c`. The
shipped link leaves 173,032 B of main RAM free after .data,
.bss and the heap reserve. Banded rendering was rejected: with
all bands stored it is the full framebuffer again, and
re-rendering per band would need a clip/offset window `toy_fb`
does not have. (Observed while measuring: current trail carts
redraw from state ring buffers each render, so persistence is
preserved as a forward contract, not because a shipped cart
reads the framebuffer back.)

Host checks: `tools/presenter_test.c` (mapping, crafted
pixels, fade persistence, harmonograph/fig8 sequences
pixel-identical to a plain-fb reference after upscale) and
`tools/display_test.c` both pass; `tools/run.sh check` passes.
Flashing stays lead-only by serial as above; expect the
`TOY-PROD-*` stream over USB CDC and a new cartridge on glass
every 20 s, starting with fig8.
