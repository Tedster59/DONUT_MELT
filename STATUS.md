# DONUT_MELT status — 2026-11-20

Snapshot of where the 3lb work stands, for picking it up in a later session. Most recent first
within each section. Details live in the commit messages and the comments next to each setting.

## Branches

| Branch | Head | State |
|---|---|---|
| `traction-control` | `08f36d5` | **7 commits not pushed** |
| `rssi-heading` | traction-control + 7 commits | **7 commits not pushed** (origin has only up to `88d2a6f`) |
| `sinusoidal-translation` | `9f5c88e` | base the traction-control branch came from |
| `main` | `fe30795` | upstream |

`traction-control` commits (newest first):
- `08f36d5` Ignore run logs dumped into the repo root (`/*.csv`; logs stay local)
- `7e633f9` Left stick X rpm trim 0.15 → 0.25 /s
- `e21d26c` Translation lead `TRANSLATION_LEAD_US 1000` (value not yet confirmed on the bot)
- `88d2a6f` Keep the traction control state current while idle (from another session's review of `spinup_traction30.csv`)
- `f4f26c8` **Safety:** stop the motor signal during flash writes and lock out input afterwards
- `90a3c41` S1 slider sets the slip target; RSSI rpm lag correction and steady-speed gate
- `7d156d9` Traction control calibration/launch fixes, per-board accel offsets, `z` command
- `0a05de5` CRSF receive by UART interrupt into a ring buffer
- `97627d6` Skip the accelerometer status poll

`rssi-heading` commits on top of traction-control (newest first):
- this STATUS.md update
- `4d153bf` USB commands `1` / `2`: DShot beacon, the ESC beeps that motor (bench, throttle zero only)
- `02866b5` Log dump feeds the watchdog every line; `dump_log.py` skips lines damaged in transfer
- `3af6949` Heading hold (`RSSI_HEADING_HOLD_US` 2 s, state 3, telemetry Volt 4.0); re-seed speed
  keeping the angle when > `RSSI_HEADING_MAX_RPM_DIFF` (now 10%) off the RSSI rpm
- `fca741f` STATUS.md
- `af1aac4` RSSI phase-locked heading tracker, shadow mode (`RSSI_HEADING` on, `RSSI_HEADING_DRIVE` off),
  RSSI page Volt 3.0 = locked, `tools/rssi_heading_replay.py`
- `343032a` `rssi_rpm_current()`: the lag-corrected RSSI rpm, shared with the tracker

Only uncommitted change: `ESC_TELEMETRY_DIAGNOSTICS` turned on in `donut_config.h` (bench setting).
Board 1 is flashed with exactly this state (still shadow mode: `RSSI_HEADING_DRIVE` off).

## Boards

| Board | Accel offsets | Flashed with |
|---|---|---|
| 1 (in the bot, USB serial `29B1AC51E3DEE97B`, COM4) | measured with `z` | latest `rssi-heading` working tree |
| 2 | measured; ~50% noisier than 1 and 4 (≈1.9 g/axis at rest) | `BOARD_ID 2` build from mid-session (no safety fixes yet) |
| 3 | old values, sensor 2 never measured (build `#warning`) | needs physical repair |
| 4 | measured | `BOARD_ID 4` build from mid-session (no safety fixes yet) |

`BOARD_ID` in `donut_3lb_config.h` selects the offsets: **set it before building for a board.**
Boards 2 and 4 should be reflashed with current code before they go in a bot.

## Safety (incident 2026-11-20)

After a kill the bot ran at full power in the spin direction for several seconds and ignored the
killswitch; no ESC arming beeps (no reset). Most likely cause: the log save erases flash with
interrupts off (45–400 ms per sector); the DShot timer had a negative period, so the SDK fired it
~225 times back to back after each erase, each call restarting the PIO mid-frame; the ESCs got
spliced frames, ~1 in 16 passing DShot's 4-bit CRC as a random throttle, held through the next erase
while the blocked main loop couldn't read the killswitch. Fixed in `f4f26c8`:
- flash writes stop the DShot output (clean signal loss, ESCs stop); positive timer period (no catch-up bursts)
- input lockout after any blocking operation: 2 s, then receiver healthy 1 s, then killswitch seen killed (fast LED blink meanwhile)
- boot sends only zero and never blocks (the old 0-1000-0 init was 1 s of **full reverse** after any non-watchdog reset)
- boot reason (watchdog / reboot after flashing / power-on / brownout / supply glitch / RUN pin) printed and logged as a `B` event

Bench tested wheels up on board 1: arming after boot, motors stopped through the save, ESCs re-arm,
an arm during the save is ignored, boot reason logged correctly. Not yet driven in anger.

## Traction control

- Calibration `TC_MOTOR_RPM_PER_BOT_RPM = 2.50 * 52.3 / TC_WHEEL_DIAMETER_MM` (ratio at throttle cut,
  third_spin.csv; heading_shadow.csv suggests 2.517). Geometry alone gives 2.32 — unexplained.
- Slip vs spin-up (spinup_two/three.csv): 15% target took 6.1 s to 2000 rpm; traction control off
  (60–100% slip) ~2.9 s but out of control. More slip = more acceleration on that floor. The target
  is now on the **S1 slider (CH12)**: bottom 10% → top 100%, shown as Curr on switch C middle
  (30.0 = 30%). User confirmed the slider changes spin-up behaviour. Best target still to be found.
- Wheels are ~23-sided polygons (mold STL tessellation, ±0.25 mm); new molds planned. Sand to true or
  recast, then update `TC_WHEEL_DIAMETER_MM`.

## RSSI heading (rssi-heading branch)

Goal: heading that doesn't drift. Any rpm-only method integrates its error (1% at 2000 rpm = a turn
every 3 s; RSSI rpm alone is ~0.27%). The tracker locks to the phase of the RSSI pattern instead:
speed from `accel_scale × corrected accel rpm`, RSSI phase corrects the angle (KP 0.5) and slowly the
scale (KS 0.005), template fundamental pinned so the frame can't creep.
- Shadow run `heading_shadow.csv` (2000 rpm, translation): firmware then locked 68%, lost lock at
  translation speed dips. Changed to accel-driven speed.
- `heading_shadow2.csv` (rough concrete, bouncing): only 28% locked, rejects in bursts. Added the
  2 s hold and the angle-keeping re-seed (`3af6949`).
- `heading_hold1.csv` (29 s, lots of translation): **96% locked + 4% holding = 100% usable**,
  12.3° spread, quality 0.92, 3% rejected. Ready to try `RSSI_HEADING_DRIVE`.
- Next: enable `RSSI_HEADING_DRIVE`, check forward stays put, then calibrate `RSSI_HEADING_LATENCY_US`
  (does forward need a different trim at 1500 vs 2500 rpm?)
  (`python tools/dump_log.py --plot-only <csv>` prints a heading summary).
- The ELRS Lite has one ceramic chip antenna; pattern harmonics are only ~0.3–1.4 dB against ~5 dB
  noise. To try: a ~6 cm conductive strip (foil) 2–3 cm from the chip, parallel to its long side,
  to deepen the pattern; compare harmonic strength, lock %, LQ minimum.

## Translation

Travel was 30–45° off the LED while translating (earlier tests), 10–15° at 2000 rpm in the last run,
gone on release → wheel response lag. `TRANSLATION_LEAD_US 1000` (≈12° at 2000 rpm) is flashed but
untested; if travel gets worse, try −1000.

## Motors

Motor 1 (GP2, "left" side; confirmed with the `1` beacon) ran hot with more cogging and ~5% slower
than motor 2 in the last two logs. Opened up: grooves on 2 magnets, 2 adjacent stator teeth polished
near the bottom (the next 2 less) — the rotor rubbed the stator. Phases all 0.3 Ω. Bell opening faces
the bot centre (centrifugal load is axial), outer end on a large bearing, h5 Misumi shafts (slight
play), so the side load is floor/bounce impacts. Cleaned, reassembled with Loctite 638 and clamped to
align the bell (full cure ~24 h; removal needs ~250 °C on the joint, keep heat off the magnets).
Motor 2 is fine and already has compound.

After cure: runout, the 2 grooved magnets flush, hand-spin/coast vs motor 2, then wheels up with
`ESC_TELEMETRY_DIAGNOSTICS` (USB "ESC ..." lines): both motors' rpm should match within ~1%; short
run, feel temperatures.

## Accelerometer

`tests/accel_test.c` read accelerometer 1 twice, so every board's accel 2 offsets were wrong (board 1
read ~310 rpm standing still). Fixed; offsets re-measured with the `z` USB command (bot killed, board
still: averages 5 s, prints the `#define` lines). `ACCEL_REST_DIAGNOSTICS` prints offset vs noise
once a second while not live. The user says the accel correction curve isn't fully dialed in yet;
after the offset change the heading needs a re-trim (left stick X, save with CH10).

## Open items

- Motor 1 repair checks (above), then `RSSI_HEADING_DRIVE` build; then latency calibration.
- Antenna reflector strip test.
- Confirm/tune `TRANSLATION_LEAD_US`, then commit it to traction-control.
- Sweep the slip target with the slider (time to 1500 rpm vs controllability).
- Push `traction-control` and `rssi-heading` when ready.
- Optional: only allow going live once both ESCs answer telemetry (instead of a fixed 3 s arm hold).
- Optional: per-sensor noise in `z` to find board 2's noisy accelerometer.
- Next board rev moves the accelerometers to SPI (pin plan in Claude's memory, own branch).

## Workflow notes

- Build: Ninja in `build/`, toolchain under `%USERPROFILE%\.pico-sdk` (ninja v1.12.1, cmake v3.31.5,
  arm 14_2_Rel1). Flash: `picotool load -f -x build/donut_melt.uf2` (picotool 2.2.0-a4) reboots a
  running board; booting sends only zero throttle, so it's safe with ESCs powered (wheels up).
- Logs: kill, wait for the LED to go solid and back to fast blink, then `python tools/dump_log.py COM4 -o <name>.csv`.
  Saving only happens on the killswitch; arming and killing again overwrites the saved run.
  The 384 KB buffer holds ~28 s while traction control logs.
- Transmitter (user's TX12 MkII): switch C = telemetry page (CH8, code `SWITCH_F`), F = spin direction
  (CH6, `SWITCH_B`), B = tank/melty (CH7, `SWITCH_C`), CH5 killswitch, CH9 button toggles traction
  control, CH10 button saves an RPM curve point, S1 slider = CH12 slip target.
