# Product firmware: the shell on glass (toy5-shell)

`toy_product` is now the real product: the runner shell
(`core/toy_shell.*`) driving every cartridge on the CO5300
glass, with live CST9217 touch, QMI8658 tilt/dial, and the
PCF85063 RTC as the wall clock. It replaces the
toy5-presenter attract rotation (not retained; see
`main_product.c` header). The four IMU feed carts run on
the real QMI8658 when it is up (toy5-livefeed; see *Live
IMU feed* below) and fall back loudly to their synthetic
scenarios when it is not. A running cart checkpoints to
internal flash every 30 s and on clean exit, and boot
auto-resumes the newest checkpoint (toy5-flashprod; see
*Flash checkpoints + boot resume* below — read the wear
numbers there before leaving a cart running 24/7).

| File | Contents |
|---|---|
| `main_product.c` | Boot + shell runner loop (event funnel, time-boxed stepper, presenter flush) |
| `product_input.h` | Pure seams: touch mapping, gesture machine, tilt gate, dial accumulator, still calibration |
| `live_feed.h` | Pure live-feed seam: sensor->synthetic axis transform, latest-sample hold + counters, source selection, LIVE info |
| `product_ckpt.h` | Pure checkpoint-policy seam: `TOY_PROD_CKPT_SECONDS` cadence timer, skip-live decision, boot-resume decision over slot scan × registry |
| `ckpt_flash.{c,h}` | Internal-flash A/B slot backend over `flash_safe_execute` (shared with `toy_board`, which links but never calls it) |
| `clock_rtc.{c,h}` | `toy_clock.h` over the PCF85063 (toy_product only; `toy_board` keeps `clock_board.c`) |
| `clock_civil.h` | Pure civil <-> epoch conversions, clock-commit combiner, tz RAM-byte encoding |
| `../tools/product_input_test.c` | Host test for both pure headers (163 checks) |
| `../tools/live_feed_test.c` | Host test for `live_feed.h` + the core LIVE labelling path (45 checks; link line in its header) |
| `../tools/prod_ckpt_test.c` | Host test for `product_ckpt.h` (29 checks; slot selection itself stays pinned by `../tools/flashslot_test.c`) |

## Boot sequence

1. USB CDC wait (≤15 s), boot/presenter/memory banner.
2. Panel init (PIO QSPI + CO5300), black fill. Optional
   `-DTOY_PROD_SELFTEST=ON` colour stages (diagnostic only).
3. I2C1 up (GPIO6/7, 400 kHz), then per-device, each failure
   loud on serial and non-fatal:
   - RTC bind + `pcf85063_init` (never sets the time, never
     clears OS). tz comes from the chip's RAM byte if a
     previous set stored one (`tz_src=rtc-ram`), else the
     compiled default UTC-7 (`tz_src=default`).
   - Touch reset + identity + verified normal-mode handshake.
   - IMU probe + init, then a ~1 s still-desk calibration
     offer: 50 samples; accepted only if every sample sits
     at |a| 950..1050 mg with a tight gz spread. Prints
     `TOY-PROD-CAL ok bias_mdps=<n>` or `skipped` (bias 0).
     Leave the board flat and untouched through boot if you
     want the dial zeroed.
4. Shell init over the full cart registry, **GALLERY
   first** — then the boot-resume scan: if internal flash
   holds a valid checkpoint for a registered cart, that
   cart auto-starts from where power left it
   (`TOY-PROD-RESUME …`); otherwise the gallery stays up
   (see *Flash checkpoints + boot resume*).

## Controls map

The device has three input channels. Everything below is
synthesised in `product_input.h` and fed to the shell first;
a running cart sees only what the shell forwards.

- **Touch** — tap and drag are ordinary pointer input.
  Controller coordinates are mirrored on both axes
  (`465 - v`, the vendor display mapping) before conversion
  to milli-radius coords. Up to two contacts, tracked per
  finger. The touch INT line is latched by a falling-edge
  IRQ, so a short tap that starts and ends during one
  frame's render+flush still triggers a report read on the
  next frame instead of being missed between level polls.
- **Gallery tap-to-launch** — on glass, **one tap on a cart
  row launches it** (the runner confirms the selecting tap
  when the finger lifts; taps longer than ~0.5 s or drags
  only select). This is glass/runner behaviour only: the
  keyboard/host flow stays select + Enter, the `RUN >`
  footer still launches the selected cart, and the
  SET CLOCK / CLOCK FACE rows and all other screens are
  unchanged (settings keeps its select-then-adjust taps).
- **Tilt** — tipping the board sends milli-g tilt (≥20 mg
  change, at most every 33 ms) in display space.
- **Dial** — rotating the whole device in its plane sends
  signed milliturns (gyro about the screen normal, still
  bias subtracted, events at ≥25 milliturn = 9°). Clockwise
  (as you look at the glass) is positive: scroll down /
  increase.
- **Long-press** — one finger held **STILL** ~1 s (≥1000 ms
  within 12 px of where it landed) = **back/exit**
  (KEY_ESC). **Dragging never exits**: once a touch moves
  more than 12 px from its start point it is a drag for
  good — holding it there, or dragging back to the start
  and holding, will not exit; lift and press again to
  long-press. The initial tap still lands first (a running
  cart sees the touch-down; the exit key itself never
  reaches it, and the rest of that press is swallowed).
- **Two-finger tap** — second finger down within 300 ms of
  the first, **while a cart is running** = **settings**
  (KEY_X). On shell screens the second finger is just a
  second pointer.

Per screen:

| Screen | Tap | Dial (rotate) | Long-press | Notes |
|---|---|---|---|---|
| GALLERY | **single tap on a cart row launches it**; `RUN >` footer also launches the selected cart; `SET >` footer opens settings; last two rows are SET CLOCK / CLOCK FACE (tap once to select, again to open — tap-to-launch covers cart rows only) | move selection | to CLOCK FACE | idles into CLOCK FACE after 1800 quiet frames (frame-counted, ~2 min at glass frame rates; handling the board counts as activity) |
| RUNNING (cart) | the cart's own pointer input | the cart's DIAL input (pond size, julia, synspec, …) | exit to GALLERY | tilt feeds TILT carts (gravity, pond, shepherd); two-finger tap opens SETTINGS and pauses the sim |
| SETTINGS | tap a row to select; tap the selected row's left/right third to −/+; tap footer to run/resume | adjust selected value | back (to the cart if it was running, else GALLERY) | `*` = staged for next (re)start; live values apply at the next step when resumed |
| CLOCK SET | tap the HH / MM / UTC-offset zone to pick the field; tap the SET zone to commit; tap the bottom zone to cancel | adjust the picked field (tz in 15-min steps) | cancel | only hh:mm + offset are edited; the date stays whatever the RTC already holds (see clock below) |
| CLOCK FACE | any tap wakes to GALLERY (or straight to CLOCK SET if the clock is unset) | wakes to GALLERY | — | shows runner wall time; an unset clock shows as unset, never a fake time |

## Live IMU feed (toy5-livefeed)

The four signal-feed carts — **gpend, seismo, attitude,
heartbeat** — take the real QMI8658 as their sample source
in this firmware. The proof firmware (`toy_board`), the
emulator and the web build keep the pinned synthetic
streams; only the product runner goes live, through the
seam core always reserved for it (`toy_feed_step` stays
proof-strict; the live runner fills frames itself in
`live_feed.h` and calls `feed()` directly — no core change).

- **Which source, when.** Decided once at cart start: IMU
  up → **LIVE** (`TOY-PROD-FEED … src=live … stream=…`);
  IMU down at boot/recovery → the cart's synthetic scenario
  with `TOY-PROD-FEED … src=synthetic-fallback reason=imu-down`
  on serial. The on-glass provenance label always tells
  the truth either way (`LIVE` / `LIVE IMU` / `LIVE CHEST`
  vs `SYNTHETIC` / `SYNTH CHEST` — the kind sits in the
  cart's hashed state). An IMU that dies mid-run degrades
  to holding the last good reading (errors counted, bus
  recovery runs as usual); an IMU that recovers mid-run
  does **not** swap a running synthetic cart to live —
  exit and relaunch.
- **How samples flow.** One IMU read per presented frame —
  the same read tilt/dial always took, same error
  back-off, same bus-health accounting, zero extra I2C.
  Each sim step needs a fixed number of samples (gpend 1,
  attitude 1, heartbeat 5, seismo 10) at the cart's 100 Hz
  assumption, so the step's frame repeats the freshest
  real reading (**latest-sample hold**). Repeats are
  counted, never hidden, and motion is never invented: a
  failed read feeds the last good values and flags the
  gap into the cart's state; before the first good
  reading the cart is not stepped and shows `NO SIG`.
  The sensor itself runs at 125 Hz ODR, but the runner
  consumes one reading per frame, so the **fresh** rate
  is the frame rate (single-digit to ~20 Hz depending on
  the cart's render cost) — the rest is hold repeats.
- **Telemetry.** While a live cart runs, 1 Hz next to
  PACE: `TOY-PROD-FEED cart=… src=live fresh=… rep=… err=…
  max_age_ms=… fresh_hz_x10=… cons_hz_x10=… spec_hz=100
  warp_x100=…`. `fresh`/`rep` = samples fed that second
  that were a new reading / a hold repeat; `err` = IMU
  read failures; `max_age_ms` = oldest held reading
  actually fed; `fresh_hz_x10` = achieved new-reading
  rate; `cons_hz_x10` = total samples fed per wall second
  (steps × samples-per-step); `warp_x100` = cons/spec×100
  — how fast cart time runs vs wall time. Warp ≠ 100
  scales anything the cart derives per assumed sample:
  seismo's displayed Hz and heartbeat's BPM both read
  true÷warp (e.g. warp 75 → BPM reads ~1.33× high).
  Expect roughly: gpend/attitude slow (warp well under
  100 — attitude's heavy render makes it slowest),
  heartbeat near 100 at ~20 fps, seismo fast (~200).
  The FEED line on your flash replaces these estimates
  with measurements.
- **Axes.** Sensor counts are mapped into the synthetic
  scenarios' frame (flat screen-up rests at +1 g on az):
  az is negated (the sensor's +Z points through the back
  of the board), ax/ay pass through under the same
  starting alignment as the tilt map. The in-plane signs
  are **not yet owner-observed for feed carts**: if
  gpend's bob or attitude's ball falls the wrong way when
  you tip the board, the fix is `TOY_LIVE_FEED_AX_SIGN` /
  `TOY_LIVE_FEED_AY_SIGN` in `live_feed.h` (one define
  each, same ritual as the tilt signs).
- **No checkpoints of live runs.** A live stream cannot
  be rebuilt from a step count, and core refuses it; the
  product skips checkpointing LIVE runs entirely (one
  `TOY-PROD-CKPT skip-live` line per run — so a power cut
  during a live feed cart boots to whatever older
  checkpoint exists, usually the gallery). Synthetic
  feed runs checkpoint and resume normally (next
  section). Proof hashes are synthetic-only everywhere,
  unchanged.

### Heartbeat chest test (owner)

The cart says it on glass and it stays true live: **toy,
not medical** — no reading from it means anything about
your health. To try it: launch **heartbeat** (label must
read `LIVE CHEST`; `SYNTH CHEST` means the IMU was down —
relaunch after checking `TOY-PROD-IMU ok`), lie back,
rest the board screen-up flat on your sternum, hold it
still with as little hand pressure as you can, breathe
slowly, give it ~10 s. Honest expectations: breathing
(~12 mg modelled) should visibly ride the trace; the
heartbeat features are only ~5 mg against this sensor's
noise and the ~frame-rate fresh sampling (see above), so
beats may register intermittently or not at all, and the
BPM number is warp-scaled (divide by warp_x100/100 from
the FEED line). A desk test works first: heartbeat live
on a table, tap the table — the detector should tick.
Serial `fresh_hz_x10` tells you the real sampling you
got; anything under ~15 Hz fresh will mostly show
breathing and motion, not a clean pulse train.

## Flash checkpoints + boot resume (toy5-flashprod)

The desk-toy behaviour: cut power mid-cart, restore it,
and the toy comes back running that cart from (at most
one cadence before) where it was. All of it is on serial
as `TOY-PROD-CKPT` / `TOY-PROD-RESUME` lines; nothing is
silent.

- **When it saves.** While a cart is RUNNING, every
  `TOY_PROD_CKPT_SECONDS` (**30 s**, the named constant in
  `board/product_ckpt.h`) of wall time, plus once on
  clean exit-to-gallery. Nothing saves from the gallery,
  settings (sim paused, state frozen), or clock face.
  LIVE feed carts never save (previous section);
  synthetic feed carts and all non-feed carts do.
- **Where, and why a power cut is safe.** Two slots of
  69,632 B (17 × 4 KiB sectors) at the end of the 4 MiB
  `PICO_FLASH_SIZE_BYTES` region (`base=0x3de000` on this
  build; the boot `TOY-PROD-CKPT layout` line prints the
  resolved numbers). Each save erases + programs the slot
  that is **not** the newest valid one and read-back
  verifies it — cut power at any instant and the previous
  checkpoint is still intact and still wins the next
  boot's scan. A save stalls the glass once: the erase +
  program runs with XIP down, placed after a frame's
  flush and before the next step, and the measured stall
  is printed (`… saved … ms=<n>`; expect a visible pause
  about that long every 30 s — typ. hundreds of ms,
  dominated by 17 sector erases; the line is the
  measurement, trust it over this sentence). Touch input
  during the stall is not lost: interrupts are masked by
  the SDK's flash-safe path, so an edge just lands late.
- **Boot resume.** After shell init, the newest slot that
  fully validates (both CRCs + state hash) and names a
  still-registered cart auto-starts that cart from the
  saved bytes — parameters included, they ride inside the
  state. A resumed synthetic feed cart stays **synthetic
  for that whole run** (its source is respawned and
  seeked to the saved step count; `src=synthetic-resume`)
  even if the IMU is up; exit and relaunch to go live.
  Anything wrong — torn/corrupt image, cart renamed or
  removed by newer firmware — boots to the gallery with
  a `TOY-PROD-RESUME-ERR` line naming the stage.
- **Launching a cart retires the old checkpoint.** Any
  fresh gallery launch (or staged-parameters restart)
  erases both slots first (`TOY-PROD-CKPT cleared …
  why=fresh-start`). That is what keeps boot's "newest =
  highest step count" honest: step counts restart at 0
  per launch and mean nothing across runs, so the slots
  only ever hold the current run's images. Deliberate
  consequence: launch a cart for two seconds and cut
  power before its first save, and the toy boots to the
  gallery — the previous cart's resume was retired when
  you launched.
- **Single-core bound.** The flash writes assume core 1
  never runs (`PICO_FLASH_ASSUME_CORE1_SAFE`, same as the
  proof firmware). A future multi-core product must
  revisit this before shipping (drop the define, add the
  core-1 lockout victim; `ckpt_flash.h` states it).

### Flash wear: the honest arithmetic

NOR flash wears per sector erase. Every save erases one
whole slot (17 sectors, one cycle each) and saves
alternate slots, so at the 30 s default a cart running
**continuously** produces 86,400 / 30 = **2,880 saves a
day across the pair = 1,440 erase cycles per sector per
day**. Against the 100,000-cycle minimum endurance
typical of W25Q-class parts (the module vendor's exact
figure is not recorded in this tree — if it is a
10,000-cycle part, divide everything by ten):

| Cart actually running | Cycles/sector/day | Time to 100k cycles |
|---|---|---|
| 24 h/day | 1,440 | **≈ 69 days** |
| 8 h/day | 480 | ≈ 208 days |
| 2 h/day | 120 | ≈ 2.3 years |
| 1 h/day | 60 | ≈ 4.6 years |
| 15 min/day | 15 | ≈ 18 years |

That is the real trade behind the 30 s default: it buys
lose-at-most-30 s-of-play after a power cut, and it is
fine for a toy that runs in sittings — it is **not** free
for a leave-it-running-forever desk piece. The lever is
one constant: `TOY_PROD_CKPT_SECONDS 300` stretches every
row ×10 (24/7 → ≈1.9 years) at the cost of resuming up
to 5 minutes back. Clean exits save once and launches
erase once; both are human-cadence and negligible
against the cadence term. Nothing
saves while the toy sits in the gallery or on the clock
face, so idle-on-desk time costs no cycles at all.

## The clock

- The RTC registers hold **UTC**; the tz offset lives in
  the PCF85063's free RAM byte (battery-backed) and is
  applied once, at display. A power loss keeps both, and
  a tz misread can only mislabel the displayed local time
  — UTC itself never moves. (This convention replaced
  registers-hold-local after two shell boots reported UTC
  going backwards; the UTC switch plus the tz sentinel
  below are the busfix-round fixes, story in
  `clock_civil.h`.) Any offset the CLOCK SET screen can
  produce (15-min steps, UTC−12:00..+14:00) persists. The
  stored byte carries a validity sentinel (magic bit), so
  a reset/erased RAM byte boots as the UTC−7 default
  (`tz_src=default`), never as an invented offset; the
  first boot after the busfix flash also boots
  `tz_src=default` until the next CLOCK SET commit (old
  bare-index bytes are rejected by design).
  `toy_sensor_test`'s serial `SET` writes the registers
  verbatim, i.e. it takes **UTC**.
- `toy_clock_now()` is valid only when the chip's
  oscillator-stop flag is clear and the fields form a real
  date. Otherwise the clock is *unset* — a first-class
  state: the clock face says so and nothing blocks.
- Committing CLOCK SET keeps the RTC's existing calendar
  date and seconds, replacing hh:mm and the offset. On a
  never-set chip whose fields do not even parse, the date
  falls back to 2000-01-01 and the serial line names it
  (`date_src=fallback-2000-01-01`) — set the real date with
  `toy_sensor_test`'s serial `SET` command if that ever
  matters; the glass UI edits time-of-day only (shell v1).
- Monotonic pacing time is the RP2350 boot clock; setting
  the wall clock never injects or swallows sim steps.

## I2C bus recovery (busfix)

Touch, IMU and RTC share I2C1, and a transfer that dies
mid-byte can leave a slave holding SDA low — then every
device NACKs forever while the frame loop runs on (the
observed wedge: tens of minutes of `-TOUCH-ERR` /
`-IMU-ERR`, gallery still idling). The runner now heals
this itself, no owner action:

- Transport errors from touch and IMU count *consecutive*
  failures (`CST9217_NO_REPORT` — an all-FF window — is a
  successful transfer and clears the count; it is never
  an error).
- At 5 consecutive errors (≈a few frames), the main loop
  runs one bus reset: `i2c_deinit`, up to 9 SCL clocks
  with SDA released, a forced STOP, re-init at 400 kHz,
  then re-probes of every device that was healthy at boot
  (RTC init + read, touch normal-mode handshake —
  escalating once to a reset pulse — IMU probe + config;
  the boot gyro bias is kept). A recovery costs one
  ~150 ms frame and completes well inside 2 s of the
  wedge. Attempts are at least 1 s apart; a device that
  stays down is retried every 5 s. The error counter
  resets only when a probe read actually succeeds.
- Serial evidence: `TOY-PROD-BUS-RECOVER begin/done`
  (see the marker table). `sda=0` with `clocks=k` in the
  done line means a slave was holding the bus and let go
  after *k* clocks — the aborted-transfer signature.

## Serial markers (USB CDC, output only)

| Marker | Meaning |
|---|---|
| `TOY-PROD-BOOT / -PRESENTER / -MEM` | banner, fb mode, linked-layout memory evidence (`-MEM` now also prints `ckpt_buf_bytes=65600`, the static SRAM checkpoint image buffer) |
| `TOY-PROD-INIT rc=` | panel init result |
| `TOY-PROD-RTC ok valid=… tz=… tz_src=…` | wall clock state at boot (`-RTC-ERR` if the chip is unreachable) |
| `TOY-PROD-TOUCH ok chip=… mirror=1 … irq=edge-fall-latched` | touch up, INT edges latched by IRQ (`-TOUCH-ERR rc=` otherwise; runtime read errors reprint at counts 1, 100, …) |
| `TOY-PROD-TOUCHDBG src=real\|synth-tap id=… raw=(x,y) glass=(x,y) milli=(x,y) screen=… act=0x… mirror=… irq_edges=…` | one line per touch DOWN acted on (never MOVE/UP): `raw` = controller coords, `glass` = after the mirror define, `milli` = what the shell received, `act` = shell action mask (`0x11` = start cart). `src=synth-tap` is the runner's tap-to-launch confirm. Touch a known corner: if `milli` names the opposite corner from your finger, the mirror define is wrong; `irq_edges` growing with no matching DOWN line means reports still die before the drain read |
| `TOY-PROD-IMU ok …` / `TOY-PROD-CAL ok bias_mdps=…` or `skipped` | IMU up; dial zero accepted or declined |
| `TOY-PROD-SHELL carts=… touch=… imu=…` | shell live, which senses survived |
| `TOY-PROD-EVENT screen=<name> cart=<name>` | every screen change |
| `TOY-PROD-EVENT start cart=… params=… hash=…` | cart reset + step-0 params applied; hash is the post-setup state (a boot-resumed start reads `resumed=1` instead of `params=…`; hash is the restored state) |
| `TOY-PROD-EVENT params-queued n=…` | live setting edits queued for the next step boundary |
| `TOY-PROD-EVENT exit cart=… why=…` | cart stopped (`shell-exit`, `sig-feed-error`) |
| `TOY-PROD-EVENT clock-commit local=hh:mm tz=… utc=… date_src=… ok=…` | a CLOCK SET commit and its fate |
| `TOY-PROD-PACE cart=… sim_us=… steps=…` | 1 Hz while RUNNING (unchanged toy5-pace line) |
| `TOY-PROD-FEED cart=… src=live odr_hz=125 spec_hz=… sps=… stream=…` | at a feed cart's start: it is on the real IMU; `stream` is the LIVE parameter hash stamped into cart state |
| `TOY-PROD-FEED cart=… src=synthetic-fallback reason=imu-down\|no-live-device …` | at a feed cart's start: no live source, the synthetic scenario runs instead (on-glass label agrees) |
| `TOY-PROD-FEED cart=… src=live fresh=… rep=… err=… max_age_ms=… fresh_hz_x10=… cons_hz_x10=… spec_hz=… warp_x100=…` | 1 Hz while a LIVE cart runs: new vs hold-repeated samples fed, read errors, oldest held sample fed, achieved fresh rate and consumption rate vs spec, cart-time warp (100 = wall rate) |
| `TOY-PROD-FEED cart=… src=synthetic-resume step=… seek_samples=…` | boot resumed a synthetic feed cart: source respawned and seeked to `step × samples_per_step`; the run stays synthetic until relaunched |
| `TOY-PROD-CKPT layout slots=2 slot_bytes=69632 base=0x… flash_bytes=… cadence_s=30` | boot: resolved A/B slot placement at the end of `PICO_FLASH_SIZE_BYTES` + the compiled save cadence |
| `TOY-PROD-CKPT saved cart=… step=… slot=0\|1 bytes=… ms=… why=cadence\|exit` | a checkpoint landed (slots alternate); `ms` is the measured between-frames flash stall; `bytes` is the TOYCKPT1 image size |
| `TOY-PROD-CKPT cleared cart=… ms=… why=fresh-start` | a fresh launch retired the previous run's checkpoints (both slots erased before step 0) |
| `TOY-PROD-CKPT skip-live cart=… why=…` | once per LIVE feed run: not checkpointing (live position not reconstructible); cadence/exit saves are skipped for the whole run |
| `TOY-PROD-CKPT-ERR cart=… stage=serialize\|write\|clear …` | a save/clear failed (serialize `err=`, flash `err=` + SDK `safe_rc=`); the run continues and the next attempt is a full cadence out — a surviving older slot is still resumable |
| `TOY-PROD-RESUME cart=… step=… slot=…` | boot auto-started the newest valid checkpoint's cart at its saved step |
| `TOY-PROD-RESUME none (gallery)` | boot: both slots blank (first boot, or a fresh launch was the last flash write) |
| `TOY-PROD-RESUME-ERR … (gallery)` | boot resume declined: `cart=<n> not-in-registry`, `no-valid-checkpoint` (torn/corrupt only), or `stage=read\|sig-init\|seek\|load\|launch` with the failing stage named |
| `TOY-PROD-FLUSH-ERR / -SIG-ERR / -BAD-CART` | presenter/feed/descriptor failures; the toy degrades to the gallery instead of wedging |
| `TOY-PROD-BUS-RECOVER begin n=… consec=… touch_err=… imu_err=…` | shared-bus reset starting (n = attempts so far incl. this one, consec = consecutive transport errors that triggered it) |
| `TOY-PROD-BUS-RECOVER done n=… ok=… sda=… scl=… clocks=… touch=… imu=… rtc=… (expected t=… i=… r=…)` | reset outcome: `sda/scl` = line levels found after deinit (0 = held low), `clocks` = SCL pulses until SDA released, per-device live flags after re-probe, `expected` = healthy-at-boot set. `ok=1` = a probe read proved the bus alive |

## First shell flash: owner script

Flash `toy_product.uf2`, open the CDC terminal, reset.
Leave the board flat and still until `TOY-PROD-SHELL`
prints (that is the calibration second).

1. **Gallery opens.** Expect the cart list, `FIG8` selected
   at the top. Rotate the board slowly clockwise in its
   plane: the selection should walk **down** the list;
   counter-clockwise walks it back up. *(Dial direction
   check — if reversed, the fix is `TOY_PROD_DIAL_SIGN` in
   `product_input.h`.)*
2. **Launch a cart with ONE tap.** Tap any cart row that
   is not already selected, once. Expect two
   `TOY-PROD-TOUCHDBG` lines (`src=real … act=0x0` — the
   tap selected the row — then `src=synth-tap … act=0x11`
   — the runner's confirm launched it), then
   `TOY-PROD-EVENT start cart=<name>` and 1 Hz PACE lines.
   (Tapping the already-selected row launches directly on
   the real DOWN: one line, `act=0x11`, no synth.) Before launching, also tap each glass corner
   once and keep the serial log: each corner tap's
   `TOUCHDBG milli=` should name that same corner
   (top-left ≈ (−1000,−1000)). *(Touch mirror check — if
   milli names the opposite corner, or drags in a cart run
   backwards, the fix is `TOY_PROD_TOUCH_MIRROR 0`.)*
3. **Tilt.** Exit (next step), launch **gravity**, and tip
   the right edge down: bodies should slosh toward the
   right and the teal spoke should point right-down-ish.
   Tip the top edge away: slosh toward the bottom.
   *(Tilt axis check — if an axis runs backwards, the fix
   is `TOY_PROD_TILT_X_SIGN` / `TOY_PROD_TILT_Y_SIGN`; if
   the axes are swapped outright, say so — that is a
   mapping change, not a sign.)*
4. **Long-press exit.** In any cart, press one finger and
   hold it **STILL** ~1 s: the toy returns to the GALLERY
   (`EVENT exit … why=shell-exit`, then `screen=gallery`).
   Also check the inverse: relaunch, press, drag well away
   from the start point, and keep holding for 2 s — the
   cart must stay running (dragging never exits).
5. **Two-finger settings.** Relaunch a cart, touch with one
   finger and add a second finger promptly: SETTINGS opens
   and the sim pauses. Tap a parameter row, tap its right
   third a few times (or rotate the device) to change the
   value, tap the footer to resume; live parameters take
   effect as it resumes (`EVENT params-queued`). Long-press
   also backs out of SETTINGS to the cart.
6. **Set a staged parameter.** From the GALLERY, open
   gravity's settings via `SET >`, change **Bodies**, tap
   the footer: the cart restarts with the new count
   (`EVENT start … params=<n>` counts the touched params
   drained at step 0).
7. **Clock face idle.** Back in the GALLERY, put the board
   down and leave it: after ~2 quiet minutes it falls into
   CLOCK FACE showing the real local time (the RTC has been
   counting since the sensors parcel). Any tap wakes it.
8. **Clock-set commit.** GALLERY → last-but-one row
   **SET CLOCK**. Tap the minutes zone, rotate the device
   to move the minutes, tap the SET zone. Expect
   `EVENT clock-commit … date_src=rtc ok=1` and the clock
   face showing the new time. Power-cycle once if
   convenient: time **and** tz should survive
   (`tz_src=rtc-ram` in the boot line after any commit).

Report back the three orientation verdicts (dial, touch
mirror, tilt axes) plus anything that feels laggy or
backwards; each is a one-define flip, deliberately.

9. **Power-cut resume (toy5-flashprod).** Launch
   **fig8**, watch for `TOY-PROD-CKPT saved … slot=0 …
   why=cadence` after ~30 s (note the `ms=` stall and
   the brief glass pause), let a second save land in
   `slot=1`, then **pull power mid-run** (ideally right
   as a `saved` line is due — the torn-write case).
   Restore power: expect `TOY-PROD-RESUME cart=fig8
   step=<n> slot=<s>` and fig8 running again from about
   the last `saved` step — never a half state. Repeat
   with an unclean cut *during* a save if you can time
   it: the older slot must win.
10. **Live carts don't save; synthetic resume.**
    Launch **gpend** with the IMU up: expect exactly one
    `TOY-PROD-CKPT skip-live` line and no `saved` lines;
    power-cut boots back to the gallery (or an older
    cart's checkpoint). For the synthetic path the lead
    can name a fallback condition; a resumed feed cart
    prints `src=synthetic-resume` and its label stays
    `SYNTHETIC`.
11. **Fresh launch retires resume.** Exit any cart
    cleanly (`saved … why=exit`), launch a *different*
    cart (`TOY-PROD-CKPT cleared …`), cut power within
    its first 30 s: boot must show `TOY-PROD-RESUME none`
    and the gallery, not the cart you exited earlier.

## Known v1 limits

- Orientation signs above are vendor/geometry-derived, not
  yet owner-observed (the script resolves them).
- CLOCK SET edits hh:mm + offset only; the calendar date is
  the RTC's existing date (serial `SET` in
  `toy_sensor_test` remains the date entry path).
- IMU samples are read once per presented frame, so dial
  resolution follows frame rate on heavy carts — and so
  does the live feed's fresh rate (see *Live IMU feed*:
  hold repeats fill the gap, warp is printed, never
  hidden). Draining the IMU faster than frame rate (FIFO
  or a mid-frame poll) is a possible later parcel, not
  this one.
- Live-feed in-plane axis signs are geometry-derived, not
  yet owner-observed (the gpend/attitude tip check in
  *Live IMU feed* resolves them; one define each).
- Proof scripts past step 0 never replay on device
  (deliberate; see `main_product.c`).
- Flash wear at the 30 s cadence is the standing cost of
  resume (see *Flash wear* above): continuous running
  reaches a 100k-cycle sector budget in ≈69 days. Raise
  `TOY_PROD_CKPT_SECONDS` for a leave-it-running piece.
- Flash checkpoints are single-core only
  (`PICO_FLASH_ASSUME_CORE1_SAFE`); a core-1 product
  must rework the safe-execute setup first.
- After a boot resume, the SETTINGS editor still seeds
  from parameter *defaults* while the running state
  carries the checkpoint's values; untouched parameters
  are never re-queued, but the editor's initial readouts
  can differ from the live values until first edited.
- A resumed synthetic feed cart cannot become LIVE
  mid-run (exit + relaunch), and a LIVE run leaves no
  checkpoint at all (both deliberate; see above).
