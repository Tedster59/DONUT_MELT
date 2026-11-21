# DONUT_MELT status — 2026-11-20

Snapshot of where the 3lb work stands, for picking it up in a later session. Most recent first
within each section. Details live in the commit messages and the comments next to each setting.

## Branches

| Branch | Head | State |
|---|---|---|
| `traction-control` | `08f36d5` | **7 commits not pushed** |
| `rssi-heading` | traction-control + 3 commits | **6 commits not pushed** (incl. this file) |
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
- this file
- `af1aac4` RSSI phase-locked heading tracker, shadow mode (`RSSI_HEADING` on, `RSSI_HEADING_DRIVE` off),
  RSSI page Volt 3.0 = locked, `tools/rssi_heading_replay.py`
- `343032a` `rssi_rpm_current()`: the lag-corrected RSSI rpm, shared with the tracker

Only uncommitted change: `ESC_TELEMETRY_DIAGNOSTICS` turned on in `donut_config.h` (bench setting).
Board 1 is flashed with exactly this state.

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
  translation speed dips. Changed to accel-driven speed; offline replay: 89% locked, 10.8° spread
  (third_spin 93%/10.5°, spinup_two 58%). **The accel-driven version hasn't run on the bot yet.**
- Next: another shadow run with translation, dump, check lock through the translations
  (`python tools/dump_log.py --plot-only <csv>` prints a heading summary). If good, enable
  `RSSI_HEADING_DRIVE`, check forward stays put, then calibrate `RSSI_HEADING_LATENCY_US`
  (does forward need a different trim at 1500 vs 2500 rpm?).

## Translation

Travel was 30–45° off the LED while translating (earlier tests), 10–15° at 2000 rpm in the last run,
gone on release → wheel response lag. `TRANSLATION_LEAD_US 1000` (≈12° at 2000 rpm) is flashed but
untested; if travel gets worse, try −1000.

## Accelerometer

`tests/accel_test.c` read accelerometer 1 twice, so every board's accel 2 offsets were wrong (board 1
read ~310 rpm standing still). Fixed; offsets re-measured with the `z` USB command (bot killed, board
still: averages 5 s, prints the `#define` lines). `ACCEL_REST_DIAGNOSTICS` prints offset vs noise
once a second while not live. The user says the accel correction curve isn't fully dialed in yet;
after the offset change the heading needs a re-trim (left stick X, save with CH10).

## Open items

- Shadow run of the accel-driven tracker; then `RSSI_HEADING_DRIVE`; then latency calibration.
- Confirm/tune `TRANSLATION_LEAD_US`, then commit it to traction-control.
- Sweep the slip target with the slider (time to 1500 rpm vs controllability).
- Push `traction-control` (4 commits) when ready.
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
