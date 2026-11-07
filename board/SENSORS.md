# Sensors: I2C bring-up (toy5-sensors)

Drivers + one standalone test firmware for the three sensors on
the Waveshare **RP2350-Touch-AMOLED-1.75**. No product
integration yet: nothing here is wired into `toy_board`, the
cartridges, or `clock_board.c`.

| File | Contents |
|---|---|
| `imu_qmi8658.{h,c}` | QMI8658C IMU: ID check, soft reset, fixed config, burst read, integer milli conversion |
| `touch_cst9217.{h,c}` | CST9217 touch: GPIO reset, identity attributes, polled report decode, INT level read |
| `rtc_pcf85063.{h,c}` | PCF85063A RTC: BCD time read/set, oscillator-stop flag handling |
| `main_sensor_test.c` | `toy_sensor_test` firmware (RP2350 only): phased CDC test described below |
| `../tools/sensor_drivers_test.c` | Host test for every pure parser/conversion in the headers |

Bus: **I2C1, SDA GPIO6, SCL GPIO7, 400 kHz**, shared by all
five devices below. Side pins: touch INT **GPIO22**, RST
**GPIO23**; IMU INT1 **GPIO28**, INT2 **GPIO29**; RTC INT
**GPIO27**. Only the touch RST/INT pins are driven/read in this
parcel (poll-based; no IRQ infrastructure anywhere). GPIO25 is
never used (UART1 RX on this board; build with
`-DTOY_BOARD_LED_PIN=-1`).

## The five scanned addresses, explained

The lead's read-only scan on the real board found exactly
`0x18, 0x34, 0x51, 0x5A, 0x6B`. Each is one fitted part; there
are no strays and nothing is missing:

| Addr | Part | Why this address | In this parcel |
|---|---|---|---|
| `0x18` | ES8311 audio codec (schematic U6) | CE pin low selects `0x18` (high would be `0x19`), per the ES8311 datasheet's address table | Untouched. Presence by scan only; no register is read or written |
| `0x34` | AXP2101 PMIC (schematic UP1) | Fixed address `0x34` per the X-Powers datasheet | One read-only ID read (reg `0x03`, expect `0x4A`). No writes, ever; battery/charge registers out of scope |
| `0x51` | PCF85063A RTC (schematic U4) | Fixed slave address `1010001b = 0x51`, no address pins (NXP datasheet Rev 7 §9.5.1) | Full driver (`rtc_pcf85063.*`) |
| `0x5A` | CST9217 touch (display FPC J3) | Fixed CST92xx-family address `0x5A` | Full driver (`touch_cst9217.*`) |
| `0x6B` | QMI8658C IMU (schematic U3) | `0x6A` with SA0 low/floating, `0x6B` with SA0 pulled up (QST datasheet Rev B, I2C section). The schematic annotates U3 `0X6B`, and the scan agrees: SA0 is pulled up on this board | Full driver (`imu_qmi8658.*`) |

Sources: Waveshare schematic
`RP2350-Touch-AMOLED-1.75.pdf` and the linked datasheets
(QMI8658C Rev B via QST, PCF85063A Rev 7 via Waveshare/NXP)
from the product resources page. The CST9217 has no datasheet
linked there and no public PDF was reachable (re-checked
2026-11-07 for the touchfix); its register facts come from
the family reference flow preserved in SensorLib
`TouchDrvCST92xx` (as shipped in Waveshare BSPs), the
independent Rust `cst92xx` crate (cites the CST9217 datasheet
§10.5 for reset timing), and Waveshare's own CST9217 driver
for this exact board (GitHub `waveshareteam/RP2350-Touch-
AMOLED-1.75`, `examples/C/01_GUI/lib/Touch/CST9217.c` +
`main.c`). All were consulted for hardware facts only; all
code here is original, and the test firmware prints raw
report bytes so a wrong assumption shows itself on the first
run instead of hiding.

## QMI8658C IMU (`imu_qmi8658.*`)

### Why this configuration

The toy's signal seam was pinned before hardware existed:
`core/toy_signal.h` synthetic IMU defaults are 100 Hz,
16384 LSB/g (±2 g), 128 LSB/(°/s) (±256 °/s), and the four IMU
carts bind statically to that (`gpend`/`attitude`: IMU6 desk,
100 Hz, 1 sample/step; `seismo`: 10/step; `heartbeat`: chest,
5/step; all divide counts by `f64_dec("16384")`). Configuring
the real part to the same two scales makes live raw counts
drop-in compatible with the synthetic streams — the single
most valuable property for later integration.

| Choice | Value | Citation |
|---|---|---|
| Accel range | ±2 g, **16384 LSB/g** | Datasheet sensitivity table; CTRL2 `aFS=000` |
| Gyro range | ±256 °/s, **128 LSB/(°/s)** | Datasheet sensitivity table; CTRL3 `gFS=100` |
| Accel 1 LSB | 1000/16384 mg ≈ **0.0610 mg** | arithmetic |
| Gyro 1 LSB | 1000/128 m°/s = **7.8125 m°/s** | arithmetic |
| Temp 1 LSB | **1/256 °C** (1000/256 ≈ 3.906 m°C) | Datasheet temperature table |
| ODR | **125 Hz Normal** (`0110`), accel + gyro | CTRL2/CTRL3 ODR tables. There is **no 100 Hz Normal rate** (…250, 125, 62.5…); a 128 Hz entry exists only as accel-only Low Power, unusable with the gyro on. 125 Hz is the nearest documented rate ≥ 100 Hz |
| On-chip LPFs | **Disabled** (CTRL5 = 0x00) | Bring-up honesty: carts already filter in integer code (EMAs/detrends), and `seismo` wants the raw spectrum. Enabling e.g. mode `11` (14% of ODR ≈ 17.5 Hz) is a one-line change if live noise demands it |

The 125-vs-100 Hz gap is real and deliberately **not** hidden:
the driver exports `QMI8658_ODR_HZ = 125`, and a future live
runner must decimate/resample explicitly (the seam refuses
silent rate mismatch by design). That decision belongs to the
integration parcel.

**Noise vs heartbeat's model (read this before trusting live
SCG dreams).** Datasheet noise density is 200 µg/√Hz (accel)
and 15 m°/s/√Hz (gyro). At 125 Hz ODR with filters off
(≈62.5 Hz bandwidth) that is ≈1.6 mg RMS (≈26 LSB) /
≈119 m°/s RMS per sample. The brief's ~0.56 mg modelled floor
is ≈9.2 LSB — comfortably above the 0.061 mg quantisation
step, so resolution is not the problem; **noise is**. The
synthetic chest stream models only ±2 LSB uniform noise
(≈0.07 mg RMS) under its ~5 mg systolic peak, and the cart's
default detect threshold is 2.0 mg. Live single-sample noise
RMS (~1.6 mg) sits just under that threshold before the cart's
own detrend band-limiting helps. Expect integration to need
averaging/decimation (125→100 Hz conversion can provide some)
or the on-chip LPF. None of that is faked here.

### Init sequence (`qmi8658_init`)

| Step | Register | Value | Meaning |
|---|---|---|---|
| Soft reset | `0x60` RESET | `0xB0` | Datasheet Rev B §5.9. Then poll `0x4D == 0x80` (reset-done) briefly + 100 ms settle (covers accel 3 ms + 3/ODR and gyro 60 ms + 3/ODR turn-on) |
| Interface | `0x02` CTRL1 | `0x40` | `ADDR_AI=1` auto-increment for bursts; `BE=0` little-endian (power-on default is big-endian — must be cleared); oscillator on |
| Accel | `0x03` CTRL2 | `0x06` | `aFS=000` ±2 g, `aODR=0110` 125 Hz, self-test off |
| Gyro | `0x04` CTRL3 | `0x46` | `gFS=100` ±256 °/s, `gODR=0110` 125 Hz, self-test off |
| Filters | `0x06` CTRL5 | `0x00` | both LPFs disabled (above) |
| Enable | `0x08` CTRL7 | `0x03` | `aEN|gEN`; AttitudeEngine off, `syncSmpl` off |
| Verify | readback | — | All five registers re-read; any mismatch fails init |

Identity: `WHO_AM_I (0x00) == 0x05` is the hard check.
`REVISION_ID (0x01)` reads `0x7C` per Rev B and is reported
but never fatal (older Rev 0.6 text disagrees with itself:
0x68 in the overview table, 0x79 in the detail table).

Reads: one 14-byte burst from `TEMP_L (0x33)` —
TEMP, AX, AY, AZ, GX, GY, GZ as little-endian int16 pairs
(`0x33..0x40`). `STATUS0 (0x2E)` bit0/bit1 are the accel/gyro
new-data flags (`qmi8658_data_ready`). Conversions to
mg/m°/s/m°C are integer-only, round-to-nearest with ties away
from zero (same convention as `toy_px_to_milli`).

**Axis frame:** values are in the sensor's own frame. How that
frame sits on the PCB (which axis reads ±1000 mg lying flat,
screen up, and with what sign) is measured on the first board
run, not assumed here. Remapping to the seam's convention
(chest/desk rest at `az = +16384`) is integration work.

## CST9217 touch (`touch_cst9217.*`)

Registers are 16-bit, sent big-endian; payloads little-endian.

| Register | Use |
|---|---|
| `0xD000` | Touch report: read 15 bytes; ack real (non-empty, non-all-FF) reports by writing `{0xD0, 0x00, 0xAB}` |
| `0xD101` | Enter debug/info (command) mode before attribute reads |
| `0xD106` | "Disable low-power scan" — defined in the SensorLib/Rust headers but written by **neither** reference and not by Waveshare's board driver. This driver does not write it (see touchfix note below) |
| `0xD109` | Normal scanning mode — only via the verified handshake below |
| `0xD11E` | Mode-switch handshake register (SensorLib `setMode`) |
| `0x0002` | Mode status echo: byte 1 repeats the low byte of the last accepted mode command |
| `0xD1F8` | Resolution: u16le X, u16le Y (expect 466×466) |
| `0xD1FC` | Firmware checkcode: u32le, high half `0xCACA` (live board read `0xcaca4e20`, matching the reference validation rule) |
| `0xD204` | u16le project ID, u16le chip type (expect `0x9217`; `0x9220` accepted as same-family) |
| `0xD208` | u32le firmware version, u32le checksum |

Reset: RST (GPIO23) low 10 ms, then 100 ms settle (datasheet
TRST ≈0.1 ms / TRON ≈100 ms via the `cst92xx` citation;
Waveshare's driver uses 10 ms + 50 ms + 30 ms, covered).
INT (GPIO22) is input **with pull-up**, active-low, idle
high: a falling edge means a report is ready (Waveshare's
board example registers a `GPIO_IRQ_EDGE_FALL` IRQ after
`DEV_KEY_Config`'s `gpio_pull_up`; SensorLib's default
`irqTriggerLevel` is also LOW). No IRQ is registered here;
the test firmware level-polls INT every loop pass and
gates report reads on assertion.

**Mode entry is the verified `setMode` handshake**
(`cst9217_enter_normal()`), re-derived in the touchfix
after the first live run (identity OK, `normal=OK`, then
60 s of all-`0xFF` report reads and 1509 anonymous
errors):

| Step | Transfer | Check |
|---|---|---|
| Handshake (≤3 attempts, 200 ms back-off on transfer failure) | write `D1 1E`, write `D1 1E` again (two STOP-terminated writes), read 4 B at `0x0002` | byte 1 == `0x1E` |
| Mode write | write `D1 09` (`0xD109`), read 2 B at `0x0002` | byte 1 == `0x09` |
| Settle | 10 ms | — |

If an echo never matches, entry **fails**
(`CST9217_ERR_MODE = -4`; a trailing transfer failure
surfaces as `CST9217_ERR_NACK = -2` instead) and phase 3
prints `normal=FAIL rc=<n>` — transfer success alone no
longer prints OK. (SensorLib's original falls through
and writes the mode even when no attempt echoed; the
Rust flow's fail-closed behaviour is followed here.)
Facts: SensorLib `TouchDrvCST92xx::setMode` and Rust
`cst92xx` `set_mode`/`REG_MODE_HANDSHAKE`/`REG_MODE_STATUS`.
Waveshare's own board driver writes no normal-mode
command at all and still receives reports after the
`0xD101` attribute read, so the attribute window alone
does not block reports — the handshake is used because
it is the only mode entry any source proves with a
readback, and its failure is visible.

Register reads everywhere: write the 2 address bytes
(big-endian) with repeated start (no STOP), then read.
Read outcomes are named, never anonymous:
`CST9217_OK` (frame decoded, possibly all-zero empty),
`CST9217_NO_REPORT` (window read all-`0xFF`: nothing
latched — counted as `touch_no_report_ff`, never acked,
never treated as a release), `CST9217_ERR_NACK` (any
transfer failed — counted as `touch_nack`).

### Report byte layout (15-byte read, ≤2 points)

| Byte | Meaning |
|---|---|
| 0 | Point 0: `track_id<<4 \| event`; `0x00`/`0xAB` here = no data |
| 1 | Point 0 `x[11:4]` |
| 2 | Point 0 `y[11:4]` |
| 3 | Point 0 `x[3:0]<<4 \| y[3:0]` |
| 4 | Gesture/cover byte (parser guard: high nibble set with bit7 = cover-gesture frame, not points) |
| 5 | Point count, low 7 bits (1..2 accepted) |
| 6 | Validity echo, must be `0xAB` |
| 7–10 | Point 1, same 4-byte packing |
| 11–14 | Unused tail of the fixed-size read |

Decode rules (`cst9217_parse_report`, host-tested): all-zero =
no new report (never acked); all-`0xFF` = nothing latched
(`all_ff`, never acked, never a release frame); byte 6 ≠ `0xAB`
= torn/stale, dropped; only event nibble `0x06` counts as an
active contact; if slot 0 is not an accepted contact the whole
frame is discarded (reference-driver rule). This layout is
confirmed byte-for-byte by all three sources (SensorLib
`getTouchPoints`, Rust `decode_touch_report`, Waveshare
`CST9217_Read_Data`); the first live failure was **not** a
layout error. Coordinates are raw 12-bit panel values,
**0..465** on this 466×466 glass; the parser does not clamp,
and `cst9217_report_in_panel()` flags strays.

There is no accepted "up" event code: a release appears as the
track vanishing (or a no-contact frame). **DOWN/MOVE/UP are
derived by the caller** from successive reports — see phase 5.
For display space, Waveshare's driver for this board mirrors
both axes (`466 - x`, `466 - y`, i.e. 465-based `465 - x` up
to their off-by-one convention); the driver here still
reports controller space unchanged, and integration applies
the mirror. The first live run produced no contact frames,
so the mirror is vendor-stated, not yet owner-observed.

## PCF85063A RTC (`rtc_pcf85063.*`)

Time registers `0x04..0x0A` (BCD, datasheet Table 5):

| Reg | Field | Mask | Notes |
|---|---|---|---|
| `0x04` | Seconds | `0x7F` | Bit7 = **OS** oscillator-stop flag |
| `0x05` | Minutes | `0x7F` | |
| `0x06` | Hours | `0x3F` | 24 h mode (init forces `12_24=0`) |
| `0x07` | Days | `0x3F` | 1..31 |
| `0x08` | Weekdays | `0x07` | Binary 0..6, free-running; convention is the owner's |
| `0x09` | Months | `0x1F` | 1..12 |
| `0x0A` | Years | `0xFF` | BCD 00..99, reported as 2000+yy |

Bad BCD nibbles or out-of-range fields decode to
`valid=false`, never to a clamped guess.

**OS flag = "do not trust this time."** It is set from
power-up until cleared and after any oscillator stop
(datasheet §8.3.1.1). The driver surfaces it untouched: init
never sets the clock and never clears OS. This matches the
product rule that unset time is a first-class state
(`core/toy_clock.h`); the owner sets time manually (no NTP),
via `pcf85063_set_time()` — STOP, burst-write all 7 registers
(seconds written with OS=0), restart. `pcf85063_clear_os()`
exists separately; if the oscillator is truly stopped the flag
will not stay cleared, which is the intended diagnostic.

Init (`pcf85063_init`): read Control_1, rewrite with
EXT_TEST=0, STOP=0 (run), 24 h mode, and **CAP_SEL preserved**
(7 pF vs 12.5 pF internal load depends on the crystal's CL,
which the schematic does not state — it shows 22 pF external
caps — so the driver reports the bit instead of guessing).
Init never issues the `0x58` software reset; that would
destroy a valid battery-backed time. Alarm/timer/CLKOUT
features are untouched.

## Test firmware: `toy_sensor_test`

RP2350 only (the appended CMake block is skipped entirely on
RP2040). Same no-hardware-float pins as `toy_board`
(float=none, double=compiler); `-Wall -Wextra -Werror` on its
own sources. Build:

```sh
cmake -S board -B build/sensor-rp2350 -G Ninja \
  -DPICO_SDK_PATH=<pico-sdk> -DCMAKE_BUILD_TYPE=Release \
  -DTOY_PICO_PLATFORM=rp2350 -DTOY_BOARD_LED_PIN=-1
cmake --build build/sensor-rp2350 --target toy_sensor_test
# -> build/sensor-rp2350/toy_sensor_test.uf2
```

### Expected output + what the owner does

Open the CDC terminal before/at reset (firmware waits ≤15 s
for USB attach). Values in `<>` vary; everything else is
literal.

```
SENSOR-TEST git <head> serial <id> clk_hz 150000000
SENSOR-TEST PHASE 0 bus-scan i2c1 sda=6 scl=7 hz=400000
SCAN found 0x18 0x34 0x51 0x5a 0x6b (5 devices, expect 0x18 0x34 0x51 0x5a 0x6b)
SENSOR-TEST PHASE 1 imu QMI8658C addr=0x6b
IMU who=0x05 rev=0x7c probe=OK (expect who=0x05 rev=0x7c)
IMU init=OK odr_hz=125 accel_lsb_per_g=16384 gyro_lsb_per_dps=128
SENSOR-TEST PHASE 2 rtc PCF85063A addr=0x51
RTC ctrl1_before=0x<..> read=OK
RTC init=OK ctrl1_after=0x<..> cap_sel=<0|1> (preserved)
RTC <date> <time> wday=<n> os=1 valid=1 (os=1: time not trusted until set)
SENSOR-TEST PHASE 3 touch CST9217 addr=0x5a rst=23 int=22
TOUCH reset=OK
TOUCH chip=0x9217 project=0x<....> res=466x466 fw=0x<........> csum=0x<........> check=0x<........> info=OK (expect chip=0x9217 res=466x466)
TOUCH normal=OK rc=0 int_level=1 (rc -4=echo-mismatch -2=nack; int idle=1, asserted=0)
SENSOR-TEST PHASE 4 pmic+codec (read-only; nothing written)
AXP2101 id_reg03=0x4a (expect 0x4a) read-only
ES8311 present at 0x18 by scan only; codec registers untouched
SENSOR-TEST PHASE 5 live 60s: tilt/rotate the board, tap and drag on the glass
SENSOR-TEST tip: send 'SET 2026-11-07 12:34:56 5' + Enter to set the RTC (weekday 0-6 last)
```

On a board whose RTC was never set, `os=1` with a reset-default
date (`2000-01-01`-ish per the register reset values) is the
correct, honest reading — not a failure.

Phase 5 (60 s, then it stops on its own). Owner script:

1. **Lay the board flat, screen up, still** for ~5 s. Expect
   ~10 `IMU` lines/s at 10 Hz with one accel axis near
   ±1000 mg and the others near 0, gyro near 0, temperature
   plausible. Note **which** axis carries gravity and its sign
   (this fixes the sensor-frame mapping for integration):
   `IMU t=<ms> a_raw=(…) a_mg=(…) g_mdps=(…) temp=<d>.<mmm>C`
2. **Tilt and slowly rotate** the board through all faces.
   Gravity should walk across axes, always |a| ≈ 1 g.
3. **Tap the desk / flick a rotation**: gyro channels should
   spike and settle; a sharp tap shows as an accel transient.
4. **Touch** (reads are INT-gated: the firmware watches the
   active-low INT line and reads on assertion, with a 1 Hz
   ungated probe as fallback): tap once (expect
   `TOUCH RAW first-contact: …` once, then
   `TOUCH DOWN id=0 x=<0..465> y=<0..465> int=<…>`), hold and
   **drag slowly** corner to corner (`TOUCH MOVE …`, printed
   when the point moves ≥2 px; check the extremes approach 0
   and 465 and note any axis mirroring), lift
   (`TOUCH RAW first-no-contact: …` once, then `TOUCH UP …`).
   Try a second finger (`id=1`). If touch is still broken,
   phase 6 now separates the evidence instead of one error
   count: `touch_int_falls` (does INT ever assert?),
   `touch_no_report_ff` (reads succeed but the window is
   all-`0xFF`; first one dumped as
   `TOUCH RAW first-no-report-ff: …`), `touch_nack`
   (transfers failing).
5. **RTC**: watch the 1 Hz `RTC …` lines count seconds
   steadily. Optionally send `SET YYYY-MM-DD HH:MM:SS W`
   (real current time; W = weekday 0–6 in any consistent
   convention). Expect `RTC SET … -> OK` and subsequent lines
   with `os=0`. A malformed line is rejected with a hint and
   changes nothing.

```
SENSOR-TEST live 10s/60s (keep tapping/tilting)   (every 10 s)
SENSOR-TEST PHASE 6 done imu_ok=<~600> imu_err=0 rtc_err=0 touch_down=<n> touch_up=<n> touch_nack=0 touch_no_report_ff=<n> touch_int_falls=<n>
SENSOR-TEST DONE (idle; reset to run again)
```

Failures degrade per-device: a sensor that fails its phase
prints `FAIL` there and is skipped in phase 5; the others
still run. Error counters in phase 6 separate transport
failures from silence.

## Verification in this parcel

- Host: `tools/sensor_drivers_test.c` —
  `gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard` — QMI
  conversions (negatives, exact .5 ties at ±8 gyro counts,
  full-scale edges), burst parse, CST reports (no-touch,
  single/multi-touch, 0/465 extremes, bad echo, cover guard,
  first-slot rule, truncation), PCF BCD (bad nibbles,
  masks, range gate, round-trips). Touchfix adds:
  mode-handshake framing/echo bytes, named FF/NACK outcomes,
  INT predicates + gating, all-FF parser classification
  (225 checks total, PASS).
- RP2350 fresh-dir build (SDK platform `rp2350-arm-s`, board
  `pico2`, `-DTOY_BOARD_LED_PIN=-1`): zero warning lines;
  no `__aeabi_f*`/`__aeabi_d*` in the four sensor objects
  (only integer `__aeabi_uldivmod`); zero VFP/DCP mnemonics in
  the ELF disassembly. UF2 74,752 B / BIN 36,988 B
  (text 41,080 B). Touchfix rebuild (`build/touchfix-rp2350`):
  zero warnings, UF2 76,288 B / BIN 37,644 B,
  SHA-256(uf2) `92035d2a…3454` (full hash in the parcel
  report's fix section).
- RP2040 fresh configure: succeeds with **no**
  `toy_sensor_test` target; `toy_board` unchanged.
- Nothing was flashed by this parcel. On-board proof is the
  phase script above, run by the lead + owner.
