# RSSI RPM tracking (3lb)

The 3lb bot can measure its RPM from the radio link instead of the accelerometers. As the bot spins, the receiver antenna's pattern sweeps past the transmitter, so the uplink RSSI repeats once per rotation. The estimator in `src/donut_rssi_rpm.c` finds that frequency.

The repeating pattern isn't a sine wave. In the first spin log (`first_spin.csv`), the 3rd harmonic of the rotation carried about twice the power of the fundamental, and the fundamental held under 10% of the RSSI signal's energy. So the estimator scores each candidate RPM by the **summed power of its first 4 harmonics**.

The accelerometer stays in the loop for three jobs:
- It gives the RSSI search its starting point.
- It is the fallback whenever RSSI has no confident estimate.
- With the RPM correction curve applied, it is what you drive on in shadow mode.

## Requirements

- **Receiver firmware:** the custom ExpressLRS 3.3.0 receiver fork (branch `melty-rssi-to-fc`). Stock ELRS sends link stats to the flight controller at only ~10 Hz, with filtered RSSI. The fork sends one link stats frame per received packet (~1 kHz) with raw RSSI.
- **Transmitter:** must stay on ELRS 3.x. ELRS 4.x does not work with 3.x receiver firmware.
- **Packet rate:** 1 kHz. CRSF runs at 921600 baud.

Check on the bench with `RSSI_LINK_DIAGNOSTICS` enabled. The USB `LINK` line should show `link_stats` close to 1000 Hz.

## How it works

1. Each link stats frame is timestamped on arrival and interpolated onto a 1 kHz grid.
2. Every 50 ms, a new estimate starts from the last 512 ms of RSSI (Hann-windowed). It searches 48 candidate frequencies across ±30% (`RSSI_RPM_SEARCH_FRACTION`) around the **curve-corrected** accelerometer RPM, scoring each by the Goertzel power at f, 2f, 3f and 4f (`RSSI_RPM_NUM_HARMONICS`). It evaluates one candidate per main loop, so the loop never stalls long enough to drop CRSF bytes. A full scan takes about 50 loops.
3. The best candidate's **confidence** is the fraction of the RSSI signal's energy in its harmonics: about 1.0 when the rotation explains all of it, near 0 for noise. An estimate counts as **locked** when all four hold:
   - confidence ≥ `RSSI_RPM_MIN_CONFIDENCE` (0.20)
   - the peak is not on the edge of the search band
   - the peak isn't itself a harmonic. If half or a third of its frequency scores at least `RSSI_RPM_MAX_SUBHARMONIC_RATIO` (0.8) of its power, the real rotation is slower, and the estimate is rejected.
   - the estimate is younger than `RSSI_RPM_MAX_AGE_US` (200 ms)

These settings come from replaying `first_spin.csv`. While powered, it was locked 99.6% of the time and stayed within 5% of the accelerometer. During a coast-down with a stale search centre, it produced no false locks.

The search centre must be the *corrected* accel RPM. If the curve factor is outside about 0.7 to 1.3, the real RPM falls outside the search band of the raw accel RPM, and RSSI can never lock.

## Modes

| `USE_RSSI_RPM` (donut_config.h) | Drive uses | RSSI estimator |
|---|---|---|
| off (default): **shadow mode** | accelerometer + curve | runs, logs and shows on telemetry, and can calibrate the curve |
| on | RSSI while locked, accelerometer + curve otherwise | drives |

Start in shadow mode. Turn `USE_RSSI_RPM` on only after the logs show RSSI tracking reliably.

### RSSI state

`rssi_rpm_state()` reports one value. The transmitter, the log and the USB output all use it.

| Value | Name | Meaning |
|---|---|---|
| 0 | unlocked | No locked RSSI estimate. The drive is on the accelerometer. With `USE_RSSI_RPM` on, this is a **fallback**. |
| 1 | shadow | RSSI is locked, but the drive is using the accelerometer (`USE_RSSI_RPM` off). |
| 2 | driving | RSSI is locked and driving. |

## Watching it at runtime

### Transmitter (SWITCH_F high, telemetry page 1)

The values arrive as the battery sensor fields:

| Field | Shows |
|---|---|
| Volt | RSSI state: `0.0` / `1.0` / `2.0` |
| Curr | Confidence in %, e.g. `45.0` = 0.45. The lock threshold is `20.0`. |
| Capa | RSSI RPM, or 0 with no estimate |
| Fuel | `1` (page number) |

Telemetry page 3 (SWITCH_F low) still shows the number of saved curve points (Curr) and the current correction factor × 10000 (Capa).

### Heading LED (only with `USE_RSSI_RPM` on)

- **Steady heading arc:** RSSI is driving.
- **Strobing arc** (on/off, `RSSI_RPM_FALLBACK_LED_STROBE_US` = 80 ms period): on the accelerometer, either a fallback or before the first lock after spin-up.

Comment out `RSSI_RPM_FALLBACK_LED_STROBE_US` to disable the strobe.

### USB serial (bench only)

The once-per-second `LINK` line (`RSSI_LINK_DIAGNOSTICS`) ends with:

```
| rpm accel 2100 (corr 2205) rssi 2198 conf 0.61 shadow
```

These are the raw accel RPM, the curve-corrected accel RPM, the RSSI RPM, the confidence, and the state.

## Flash run log

`USE_FLASH_LOGGER` records each live run in RAM and saves it to flash once the killswitch has been on for 2 s. The LED goes solid while it saves. The buffer holds about 37 s. To download and plot a run, with the killswitch on and USB connected:

```
python tools/dump_log.py --plot
```

Record kinds in the CSV:

| kind | Rate | Columns |
|---|---|---|
| R | per link stats frame (~1 kHz) | `rssi` (-dBm), `lq` |
| A | 100 Hz | `throttle_pct`, `accel_rpm` (raw) |
| P | 20 Hz | `rssi_rpm`, `confidence` |
| D | 100 Hz | `rssi_state`, `drive_rpm` (the RPM the drive actually used) |
| C | 100 Hz | `correction` (curve × trim), `accel_rpm_corrected` |
| F | event | Failsafe started: `fs_reasons`, `fs_ms` (time since the last RC frame), `rssi`/`lq` (last link stats) |
| f | event | Failsafe cleared: `fs_ms` = how long it lasted |
| W | event | This run followed a **watchdog reboot**. The run that was in progress was lost from RAM. |

The plot shows RSSI, then raw accel, corrected accel, RSSI and the RPM used by the drive, then throttle, confidence and RSSI state. Stretches where RSSI was **not locked** are shaded red. Logs from before the D and C records were added still plot, without those traces.

### Failsafe events

The bot only records while live, so a failsafe shows up as a gap in the records. Because the ring buffer keeps only the last ~37 s, an early failsafe used to get overwritten. Failsafe events now go in a separate list of up to 64 per run (the first ones are kept), saved with the run.

`dump_log.py` prints them after downloading. The plot marks them as dashed lines: red for start, green for end, black for a watchdog reboot.

`fs_reasons` is a bitmask:

| Bit | Reason | Meaning |
|---|---|---|
| 1 | link LQ/RSSI | The last link stats frame had LQ < 70 or RSSI weaker than -105 dBm (the CRSF library's check) |
| 2 | RC timeout | No RC frame for `RECEIVER_RC_TIMEOUT_US` (100 ms). The receiver stopped sending, or the serial link dropped. |

Any failsafe sets the "require zero throttle" latch, so after a blip the bot stays killed until you pull the throttle down.

The log format is version 2. The firmware still dumps a version 1 log saved by older firmware, without events.

## RPM correction curve

The accelerometer RPM is corrected by a curve of up to 8 (raw RPM, factor) points stored in flash (`src/donut_rpm_curve.c`):

```
corrected = raw_accel_rpm * clamp(curve(raw_accel_rpm) * trim, 0.5, 1.5)
```

RSSI doesn't use the curve, because it measures RPM directly. The curve still matters for two reasons:
- It is the fallback whenever RSSI loses lock.
- It sets the centre of the RSSI search.

### Calibrating the curve from RSSI (recommended)

With `RPM_CURVE_SAVE_FROM_RSSI` defined (the default), pressing **CH10** while RSSI is locked and the speed is steady saves the **measured** factor at that speed:

```
factor = rssi_rpm / mean raw accel rpm over the same ~0.5 s window
```

"Steady" means both of these stay within `RSSI_CAL_MAX_SPREAD` (5%) over the RSSI window:
- the spread of raw accel RPM
- the change between consecutive RSSI estimates

This works in shadow mode, so you can calibrate the fallback curve before you let RSSI drive.

1. Spin up and hold a constant throttle for at least a second. Telemetry page 1 should show Volt `1.0` (or `2.0`) and a healthy confidence.
2. Press CH10. Page 3's curve point count goes up, and the correction factor changes to the measured value. With USB attached, serial prints `CURVE from RSSI: CURVE saved point ...`.
3. Repeat at 3 or 4 speeds across the range you drive at.
4. Kill the bot and leave the killswitch on for 2 s to write the curve to flash.

If the speed isn't steady enough, the press behaves differently depending on the mode:
- **RSSI driving:** CH10 does nothing.
- **Shadow mode:** CH10 falls back to saving the hand trim, described below.

The transmitter doesn't show which kind of save happened, only USB serial does. If saves never come from RSSI, the accel RPM is probably wobbling more than 5%: raise `RSSI_CAL_MAX_SPREAD`.

### Hand trimming (no RSSI lock)

1. Spin at a steady speed.
2. Trim with **left stick X** until the heading holds.
3. Press **CH10**. This stores curve × trim at the current raw RPM and resets the trim to 1.0.

**Left stick X is ignored while RSSI is driving.** The trim would have no visible effect, and it would silently apply on the next fallback.

Hold CH10 for 3 s while killed to clear the curve.

## Suggested test sequence

1. **Bench:** check that `LINK` shows ~1000 Hz link stats.
2. **Shadow spins:** spin at several speeds. On the transmitter, check that Volt reaches `1.0` and confidence stays above `20.0`; the first spin log suggests about 35–45. Dump the log and check that the RSSI dots follow the corrected accel line.
3. **Calibrate:** save RSSI curve points at 3 or 4 steady speeds as described above. Heading drift on the accelerometer should get visibly smaller.
4. **Drive on RSSI:** enable `USE_RSSI_RPM`. Steady arc and Volt `2.0` mean RSSI is driving. A strobing arc and Volt `0.0` mean it has fallen back. The red bands in the log show where.

## Tuning reference (donut_config.h)

| Define | Default | Effect |
|---|---|---|
| `RSSI_RPM_SEARCH_FRACTION` | 0.3 | Search band around the corrected accel RPM |
| `RSSI_RPM_MIN_CONFIDENCE` | 0.20 | Lock threshold |
| `RSSI_RPM_NUM_HARMONICS` | 4 | Harmonics summed per candidate frequency |
| `RSSI_RPM_MAX_SUBHARMONIC_RATIO` | 0.8 | Rejects peaks that are harmonics of a slower rotation || `RSSI_RPM_MAX_AGE_US` | 200000 | How long a lock outlives its last good estimate |
| `RSSI_RPM_HARMONIC` | 1 | RSSI cycles per rotation of the fundamental. Leave at 1: the harmonic sum handles multi-lobe patterns. |
| `RSSI_RPM_MIN_SEED_RPM` | 300 | No search below this accel RPM |
| `RSSI_RPM_FALLBACK_LED_STROBE_US` | 80000 | Fallback LED strobe period; comment out to disable |
| `RPM_CURVE_SAVE_FROM_RSSI` | defined | CH10 saves the measured factor when available |
| `RSSI_CAL_MAX_SPREAD` | 0.05 | Steadiness needed for an RSSI curve point |
