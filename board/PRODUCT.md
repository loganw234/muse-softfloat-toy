# Product firmware: the shell on glass (toy5-shell)

`toy_product` is now the real product: the runner shell
(`core/toy_shell.*`) driving every cartridge on the CO5300
glass, with live CST9217 touch, QMI8658 tilt/dial, and the
PCF85063 RTC as the wall clock. It replaces the
toy5-presenter attract rotation (not retained; see
`main_product.c` header). Feed carts still run on their
synthetic on-device sources; wiring the live IMU into the
signal seam is the next parcel.

| File | Contents |
|---|---|
| `main_product.c` | Boot + shell runner loop (event funnel, time-boxed stepper, presenter flush) |
| `product_input.h` | Pure seams: touch mapping, gesture machine, tilt gate, dial accumulator, still calibration |
| `clock_rtc.{c,h}` | `toy_clock.h` over the PCF85063 (toy_product only; `toy_board` keeps `clock_board.c`) |
| `clock_civil.h` | Pure civil <-> epoch conversions, clock-commit combiner, tz RAM-byte encoding |
| `../tools/product_input_test.c` | Host test for both pure headers (163 checks) |

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
4. Shell init over the full cart registry, **GALLERY first**.

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
| `TOY-PROD-BOOT / -PRESENTER / -MEM` | banner, fb mode, linked-layout memory evidence |
| `TOY-PROD-INIT rc=` | panel init result |
| `TOY-PROD-RTC ok valid=… tz=… tz_src=…` | wall clock state at boot (`-RTC-ERR` if the chip is unreachable) |
| `TOY-PROD-TOUCH ok chip=… mirror=1 … irq=edge-fall-latched` | touch up, INT edges latched by IRQ (`-TOUCH-ERR rc=` otherwise; runtime read errors reprint at counts 1, 100, …) |
| `TOY-PROD-TOUCHDBG src=real\|synth-tap id=… raw=(x,y) glass=(x,y) milli=(x,y) screen=… act=0x… mirror=… irq_edges=…` | one line per touch DOWN acted on (never MOVE/UP): `raw` = controller coords, `glass` = after the mirror define, `milli` = what the shell received, `act` = shell action mask (`0x11` = start cart). `src=synth-tap` is the runner's tap-to-launch confirm. Touch a known corner: if `milli` names the opposite corner from your finger, the mirror define is wrong; `irq_edges` growing with no matching DOWN line means reports still die before the drain read |
| `TOY-PROD-IMU ok …` / `TOY-PROD-CAL ok bias_mdps=…` or `skipped` | IMU up; dial zero accepted or declined |
| `TOY-PROD-SHELL carts=… touch=… imu=…` | shell live, which senses survived |
| `TOY-PROD-EVENT screen=<name> cart=<name>` | every screen change |
| `TOY-PROD-EVENT start cart=… params=… hash=…` | cart reset + step-0 params applied; hash is the post-setup state |
| `TOY-PROD-EVENT params-queued n=…` | live setting edits queued for the next step boundary |
| `TOY-PROD-EVENT exit cart=… why=…` | cart stopped (`shell-exit`, `sig-feed-error`) |
| `TOY-PROD-EVENT clock-commit local=hh:mm tz=… utc=… date_src=… ok=…` | a CLOCK SET commit and its fate |
| `TOY-PROD-PACE cart=… sim_us=… steps=…` | 1 Hz while RUNNING (unchanged toy5-pace line) |
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

## Known v1 limits

- Orientation signs above are vendor/geometry-derived, not
  yet owner-observed (the script resolves them).
- CLOCK SET edits hh:mm + offset only; the calendar date is
  the RTC's existing date (serial `SET` in
  `toy_sensor_test` remains the date entry path).
- IMU samples are read once per presented frame, so dial
  resolution follows frame rate on heavy carts.
- Feed carts (gpend/seismo/attitude/heartbeat/synspec…)
  still consume synthetic sources; live IMU feed is next.
- Proof scripts past step 0 never replay on device
  (deliberate; see `main_product.c`).
