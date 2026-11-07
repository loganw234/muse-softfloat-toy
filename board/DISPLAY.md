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
  discrepancies). D0 ran 25 MHz at the stock 150 MHz system
  clock; toy5-pace raises the target to the 50 MHz cap, which
  the integer PIO divider lands at an actual 37.5 MHz (see
  the toy5-pace addendum below).

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
codec) → `TOY-DISP-QSPI scl_hz=37500000 ...` → `TOY-DISP-INIT
rc=0` → black fill with a `TOY-DISP-TIME ... full_frame_us=`
line (wire floor 23.2 ms at 37.5 MHz quad since toy5-pace,
was 34.8 ms at 25 MHz; a full frame is
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
   (50 MHz target / 37.5 MHz actual → 25 MHz target / actual)
   before suspecting anything else.

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
to link ("region RAM overflowed by 2088 bytes"; .bss
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

### Black-glass fix (toy5-presenter follow-up)

First on-glass result for the parcel above: serial perfect
(`INIT rc=0`, per-second `PACE`, no `FLUSH-ERR`), glass black.
Isolation was immediate: the merged `toy_display_test`
reflashed and cycled correctly, so panel + init + `co5300_fill`
were glass-proven and the fault sat in the presenter's flush
shape or its transport conditions. Root cause, in merged D0
code plainly: `wait_idle` in `display_qspi_pio.c` tested the
sticky PIO TXSTALL flag without clearing it first. A mid-stream
FIFO underrun (the CPU refilling in 128-byte conversion chunks
makes underruns routine; product flush measured ~71 ms/frame
against ~38 ms of wire time, i.e. the machine stalls about
half the flush) arms that flag long before the stream ends, so
CS could rise while the final byte was still shifting out of
the OSR - truncating the last parameter byte of short CASET/
RASET command streams, after which the panel can silently drop
the address window and the following pixel stream lands
nowhere. `toy_display_test` never armed it: its conversion-free
fill loop stays ahead of the FIFO (34.8 ms full frame =
wire-bound). Two fixes land together: (1) `wait_idle` now waits
for FIFO empty, clears TXSTALL, then waits for a fresh stall
event (with a short already-stalled backstop); (2) the driver
gained a streaming API (`co5300_stream_begin/pixels/end`) and
the presenter flush is now ONE full-glass window with expanded
rows streamed inside a single CS - the fill's exact wire shape -
instead of 466 per-row windows per frame. `tools/display_test.c`
pins streamed output byte-identical to `co5300_write_pixels`
plus the sequencing/count errors; `tools/presenter_test.c`
pins exactly one RAMWR stream per flush. A diagnostic boot
self-test (`-DTOY_PROD_SELFTEST=ON`, default OFF) adds three
~1.5 s stages to `toy_product` - fill-red, presenter-green,
legacy-perrow-blue - whose `TOY-PROD-SELFTEST` outcomes separate
product bring-up, the production flush, and the old per-row
framing (now with the fixed drain) in a single flash.

## Addendum (toy5-pace): flush maps, QSPI clock, sim budget

Owner decisions behind this parcel: rendering stays 360x360
with the integer upscale to the 466 glass, and pacing is
frame-rate first (the sim runs slower than wall clock on
heavy carts rather than the frame rate collapsing). Measured
baseline (real RP2350, attract, 25 MHz QSPI): flush was a
constant 62.7 ms/frame against a 34.8 ms wire floor;
sim-bound carts at fixed demo steps ran 1-4 fps (pendulum
sim 1,659 ms/frame, pendwave 2,529, mandelbrot 14,690,
cloud 795, synspec 216); render-bound carts spent solclock
370 ms, pond 347 ms and attitude 155 ms in `render()` per
frame (that cost is inside the carts writing the 360
framebuffer - `render_us` in PACE brackets only the cart
call, and carts never call the presenter, so there is no
presenter-side component to it).

Three changes: (1) the upscale no longer divides per pixel -
`toy_presenter_init` precomputes the 466-entry column/row
source maps from the pinned `toy_presenter_src_coord`
formula (1,864 B), expansion is a table lookup, and the 106
glass rows that repeat a source row reuse the last expanded
row (cache reset every flush; `tools/presenter_test.c` pins
all of it pixel-identical to the formula). (2) QSPI SCL
target raised to the datasheet 50 MHz cap (V0.00 s6.4 p107:
TSCYC write min 20 ns, Note 3 caps 4-wire QSPI at 50 MHz);
with the integer PIO divider at the stock 150 MHz sysclk
that is divider 2 = an actual 37.5 MHz, the nearest below
the cap (divider 1 = 75 MHz violates TSCYC; a fractional
1.5 would emit sub-20 ns cycles). Wire floor for a full
frame: (466x466x2 pixel bytes x 2 nibbles + 20 command bytes
x 8 cycles) / 37.5 MHz = 23.2 ms, down from 34.8 ms at
25 MHz. `toy_display_test` shares this transport and simply
runs at the faster rate (`TOY-DISP-QSPI scl_hz=37500000`).
(3) The attract loop time-boxes sim work per presented
frame (`TOY_PROD_SIM_BUDGET_US`, default 30,000): at least
one step always runs, stepping stops at
`demo_steps_per_frame` or at the budget, checked once per
step. A single over-budget step still completes and the
frame presents after it. `TOY-PROD-PACE` gains `steps=` (the
window's total steps), so the sim-speed ratio vs fixed
pacing is steps / (frames x steps_per_frame); `steps ==
frames` marks the one-step-per-frame regime. Determinism is
untouched by construction: state after N steps is the same
as ever (feed/apply/step order and step-0 setup unchanged);
only how many steps happen per presented frame - runner
pacing, not cart behaviour - varies with wall clock.
