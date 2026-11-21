#ifndef DONUT_CONFIG_H
#define DONUT_CONFIG_H

#include "donut_dependencies.h"

void donut_init_bot_state();

uint8_t donut_is_throttle_zero();

uint8_t donut_is_killswitch_active();

uint8_t donut_get_curr_drive_mode();

uint8_t donut_should_spin_clockwise();

// Call after anything that blocked the main loop (flash writes, the USB dump, 'z'): the receiver
// input that arrived meanwhile isn't trusted, so the bot can't go live until INPUT_LOCKOUT_MS has
// passed, the receiver has been healthy for INPUT_LOCKOUT_RX_HEALTHY_MS since, and the killswitch
// has been seen killed after the lockout (a switch flipped to armed during the block is ignored).
void donut_note_loop_blocked(void);

//----------BOT TYPE----------

//#include "donut_1lb_config.h"
#include "donut_3lb_config.h"

//----------END BOT TYPE----------


//----------DEBUG SETTINGS----------

// #define RUNNING_A_TEST

//#define FAKE_RPM 60 // the fake rpm to be used in get_fake_rpm (in some capacity)

// #define LIE_ABOUT_RPM // fakes RPM data
// #define LIE_ABOUT_INPUT // fakes receiver data
// #define NO_MOTOR_SPINNING // prevents motors from spinning up if defined
#define BYPASS_MIN_TRANSLATION_RPM // lets us attempt to translate under MIN_TRANSLATION_RPM if defined

//----------END DEBUG----------


//---------------LED SETTINGS---------------

#define SLOW_BLINK 500
#define FAST_BLINK 50
#define REPEAT_BLINK 250

//---------------END LED SETTINGS---------------


//----------ACCELEROMETER SETTINGS----------

#define ACCEL_1_I2C_PORT i2c0
#define ACCEL_1_I2C_SDA 4
#define ACCEL_1_I2C_SCL 5

#define ACCEL_2_I2C_PORT i2c1
#define ACCEL_2_I2C_SDA 10
#define ACCEL_2_I2C_SCL 11

//----------END ACCELEROMETER SETTINGS----------


//----------REICEVER SETTINGS----------

#define RECEIVER_UART_ID uart1
#define RECEIVER_UART_TX_PIN 8
#define RECEIVER_UART_RX_PIN 9

#define RECEIVER_LOWEST_CHANNEL_VALUE 222 // actual lowest is 172 for knobs and 212 for joysticks
#define RECEIVER_HIGHEST_CHANNEL_VALUE 1780 // actual highest is 1790
#define RECEIVER_MIDDLEST_CHANNEL_VALUE 1001 // (222 + 1780)/2 = 1001

// failsafe if no RC channels frame arrives for this long (frames normally arrive every ~1ms at 1kHz,
// ~20ms at the slowest 50Hz ELRS rate). Catches a receiver that stops talking without reporting low LQ.
#define RECEIVER_RC_TIMEOUT_US 100000

// the killswitch (CH5) has to read the other way continuously for this long before it takes effect,
// in either direction. A single bad RC frame mid-spin otherwise kills the bot, and the kill latches
// require_zero_throttle, so it stays dead until the stick comes back to zero (seen in testing: a
// mid-spin cut with no failsafe and the switch untouched). 50ms is ~50 frames at 1kHz (2-3 at
// 50Hz), and nothing next to a multi-second spindown.
// Filtered glitches are logged as G events.
#define KILLSWITCH_DEBOUNCE_MS 50

//----------END REICEVER SETTINGS----------


//----------RSSI RPM SETTINGS----------

// uses get_rpm_rssi (RSSI estimate, falling back to get_rpm_2accel) for the 3lb drive loop
// #define USE_RSSI_RPM

#define RSSI_RPM_GRID_INTERVAL_US 1000     // RSSI resample spacing: matches the 1kHz packet rate, Nyquist = 30000 RPM per RSSI cycle/rotation
#define RSSI_RPM_BUF_SIZE 512              // grid samples analysed: 512 * 1ms = 512ms window
#define RSSI_RPM_MAX_GAP_US 250000         // link stats gap that discards the history
#define RSSI_RPM_UPDATE_INTERVAL_US 50000  // how often a new estimate is started (the scan then runs one frequency per main loop)
#define RSSI_RPM_MIN_SEED_RPM 300          // don't search until the accel says we're spinning
#define RSSI_RPM_SEARCH_FRACTION 0.3f      // search accel_rpm * (1 +/- this)
#define RSSI_RPM_SEARCH_STEPS 48           // frequencies evaluated across that band
#define RSSI_RPM_HARMONIC 1                // RSSI cycles per rotation of the fundamental (keep 1: the summed harmonics cover multi-lobe patterns)
#define RSSI_RPM_NUM_HARMONICS 4           // power at f, 2f, 3f, 4f is summed per candidate (3lb RSSI: 3rd harmonic is the strongest)
#define RSSI_RPM_MIN_CONFIDENCE 0.20f      // minimum confidence for get_rpm_rssi to trust the estimate
                                           // (first_spin.csv replay: 99.6% locked while powered, 0 false locks)
#define RSSI_RPM_MAX_SUBHARMONIC_RATIO 0.8f // reject a peak whose f/2 or f/3 has >= this fraction of its power: it's a harmonic
#define RSSI_RPM_MAX_AGE_US 200000         // older than this and get_rpm_rssi falls back to accel
// get_rpm_rssi shifts the estimate from the middle of its window to now using the accel speed
// trend, and only drives on it while the speed is changing slower than this and the result is
// within RSSI_RPM_MAX_ACCEL_DIFF of the corrected accel rpm (spin-ups and coast-downs stay on accel)
#define RSSI_RPM_MAX_SLOPE_RPM_S 300.0     // ~2.5% lag correction at 3000 rpm; spinup_two ran 220-440 rpm/s
#define RSSI_RPM_MAX_ACCEL_DIFF 0.05       // RSSI and accel agree within ~1% at steady speed

// while live at zero throttle in melty mode, keep reading the accel rpm at this interval so the
// RSSI search stays centred on the real speed during a coast-down (it used to freeze at the last
// powered value)
#define IDLE_RPM_REFRESH_US 20000

// While USE_RSSI_RPM is driving but has fallen back to the accelerometer (or hasn't locked yet),
// the heading LED strobes on/off with this period instead of showing a steady arc.
// Solid heading arc = RSSI, strobing arc = accel fallback. Comment out to disable.
#define RSSI_RPM_FALLBACK_LED_STROBE_US 80000

// CH10 (RPM_CURVE_SAVE_SWITCH) while RSSI is locked and the speed is steady saves the MEASURED
// factor (rssi rpm / raw accel rpm) as the curve point instead of the hand-trimmed one. Works in
// shadow mode too (USE_RSSI_RPM off), so you can calibrate the accel curve with RSSI before trusting
// RSSI to drive. Commented out = CH10 always saves the hand trim (current setting: tuning by hand
// until the harmonic-sum RSSI lock has been checked against real spins).
// #define RPM_CURVE_SAVE_FROM_RSSI
// Phase-locked heading from the RSSI pattern (donut_rssi_heading.h). RSSI_HEADING runs the tracker
// and logs it (H/J records) without touching the drive; add RSSI_HEADING_DRIVE to time the heading
// LED and translation from it while it's locked (the rpm-based timing takes over whenever it isn't).
#define RSSI_HEADING
// #define RSSI_HEADING_DRIVE
#define RSSI_HEADING_WINDOW_US 200000      // RSSI analysed per update (~4-10 rotations at 1200-3000 rpm)
#define RSSI_HEADING_UPDATE_US 50000       // update rate (20 Hz)
#define RSSI_HEADING_HARMONICS 4           // pattern harmonics matched (the 3rd carries most of the precision)
#define RSSI_HEADING_KP 0.5                // fraction of the measured phase error corrected per update
#define RSSI_HEADING_KS 0.005              // accel-to-true-rpm scale correction per update (error / update period)
#define RSSI_HEADING_TEMPLATE_TAU_US 2000000 // how slowly the learned pattern adapts
#define RSSI_HEADING_Q_MIN 0.5f            // match quality below this: skip the update
#define RSSI_HEADING_MAX_ERR_DEG 60        // phase error above this: skip the update (no real rotation jumps that far)
#define RSSI_HEADING_LOCK_UPDATES 10       // accepted updates in a row to lock (0.5 s)
#define RSSI_HEADING_LOSE_UPDATES 10       // rejected updates in a row to lose the lock
#define RSSI_HEADING_MAX_RPM_DIFF 0.05     // tracker rpm this far from the RSSI rpm: restart from the RSSI rpm
                                           // (the lag-corrected RSSI rpm scatters ~3% during a spin-up)
#define RSSI_HEADING_LATENCY_US 0          // RSSI measurement to arrival delay, subtracted from timestamps
                                           // (uncalibrated: 0.5 ms = 9 deg of heading at 3000 rpm)
#define RSSI_CAL_MAX_SPREAD 0.05f          // max (max-min)/mean of accel rpm, and change between RSSI estimates, over the window

//----------END RSSI RPM SETTINGS----------


//----------RPM CORRECTION CURVE SETTINGS (3lb)----------

// corrected rpm = accel rpm * clamp(curve(accel rpm) * trim). See donut_rpm_curve.h for tuning steps.
#define RPM_CORRECTION_MIN 0.5
#define RPM_CORRECTION_MAX 1.5
#define RPM_TRIM_RATE_PER_SEC 0.25          // left stick X trim speed at FULL stick; scales down with deflection (was 0.15)
                                            // (half stick ~0.03/s, since the stick is cubed)
#define RPM_TRIM_DEADZONE 0.02              // left stick X (after input_remapping, i.e. cubed) deadzone:
                                            // 0.02 = ~27% of the stick's half-travel
#define RPM_CURVE_MAX_POINTS 8
#define RPM_CURVE_MERGE_FRACTION 0.05f      // saving within 5% of an existing point's rpm updates that point
#define RPM_CURVE_MIN_SAVE_RPM 300          // ignore save presses below this accel rpm
#define RPM_CURVE_CLEAR_HOLD_MS 3000        // hold the save switch this long while killed to clear the curve
#define RPM_CURVE_PERSIST_DELAY_MS 2000     // killswitch must be on this long before writing to flash
#define RPM_CURVE_FLASH_OFFSET (1024 * 1024 - 4096) // one sector just below the run logger's region

//----------END RPM CORRECTION CURVE SETTINGS----------


//----------TRACTION CONTROL SETTINGS (3lb)----------

// Caps the melty base throttle during spin-up so the wheels don't burn out (donut_traction.h).
// While it can act, the left stick goes straight to the throttle (no THROTTLE_PC_P ramp).
// CH9 toggles it on/off; the transmitter's SWITCH_F middle page shows the state as the drive mode
// decimal (x.0 off, x.1 uncalibrated, x.2 no ESC telemetry, x.3 watching, x.4 limiting).
#define TRACTION_CONTROL
#define TC_ENABLED_AT_BOOT true

// Motor rpm (eRPM / MOTOR_POLE_PAIRS) per bot rpm when the wheels aren't slipping. 0 = not
// calibrated: everything is measured and logged, but the throttle is never limited. Calibrate
// from a log of a few gentle steady spins: dump_log.py --plot prints a suggested value.
// 2.50 from third_spin.csv: the ratio at throttle cut, where the wheels push least (2.61 steady at
// 61% throttle = ~4.5% slip). Too high a value hides real slip, so err low.
// It scales with 1 / wheel diameter: re-measure the wheels as the urethane wears and update
// TC_WHEEL_DIAMETER_MM (1 mm less diameter = 1.9% higher ratio). Geometry alone (60.7 mm contact
// radius / 26.15 mm wheel radius) gives 2.32; the measured 2.50 means the wheels effectively roll
// ~5 mm further out than their centre line, or on a smaller radius than the calipers measure.
#define TC_WHEEL_DIAMETER_MM 52.3
#define TC_MOTOR_RPM_PER_BOT_RPM (2.50 * 52.3 / TC_WHEEL_DIAMETER_MM)

#define TC_TARGET_SLIP 0.30         // wheels allowed this much faster than the bot (0.30 = 30%). 0.15 was in
                                    // control but took 6.1 s to 2000 rpm; off (60-100% slip) hit 2000 in ~2.9 s
                                    // but went out of control (spinup_two/three.csv). Sweeping for the trade-off.
// The S1 slider (CH12) sets the slip target live instead of TC_TARGET_SLIP: slider bottom =
// TC_SLIP_KNOB_MIN, top = TC_SLIP_KNOB_MAX. Shown on the switch C middle page in the Curr field
// (30.0 = 30%, replacing the killswitch flag there) and logged ('S' records, tc_target column).
// Comment out to use the fixed TC_TARGET_SLIP.
#define TC_SLIP_KNOB
#define TC_SLIP_KNOB_MIN 0.10
#define TC_SLIP_KNOB_MAX 1.00

#define TC_MIN_REF_RPM 300         // slip is measured against at least this rpm (no divide by ~0)
#define TC_ACCEL_TRUST_RPM 500      // accelerometer rpm below this is ignored (offset error / noise)
#define TC_LAUNCH_RPM_PER_S 1000    // launch ramp: how fast the bot can spin up on grip, up to TC_ACCEL_TRUST_RPM
                                    // (3000 ran far ahead of the bot: spinup_one.csv launch burnout)
#define TC_FILTER_TAU_US 15000      // low pass on both wheel and accel rpm (matched delays); 5 ms let
                                    // 10 ms wheel chatter through, which the cap can't follow anyway
#define TC_ACCEL_INTERVAL_US 2000   // accelerometer read rate for traction control (each read ~0.6ms)
#define TC_MAX_ERPM_AGE_US 5000     // ESC eRPM older than this = no telemetry, stop limiting
#define TC_CUT_GAIN 20.0            // cap drop per second per unit of slip over target
#define TC_KP 0.5                   // extra immediate cap drop per unit of slip over target
#define TC_RECOVER_PER_S 1.5        // cap rise per second while under target slip
#define TC_MIN_CEILING 0.05         // never cap the throttle below this (0.10 still slipped ~20% at 500 rpm)
#define TC_LOG_INTERVAL_US 10000    // M/T/U/V log records at 100 Hz while in melty with throttle
                                    // (200 Hz cut the log buffer to ~21 s)

//----------END TRACTION CONTROL SETTINGS----------


//----------FLASH LOGGER SETTINGS----------

// Records RSSI samples + RPM estimates in RAM while the bot is live, saves the run to flash once
// the killswitch has been on for LOGGER_SAVE_DELAY_MS (LED goes solid while saving, a few seconds),
// and dumps it over USB serial later: d = dump CSV, x = erase, s = status, h = help.
// Saving only happens on the killswitch, so kill the bot and wait for the save before cutting power.
#define USE_FLASH_LOGGER

#define LOGGER_BUF_BYTES (384 * 1024)      // RAM ring buffer, 8 bytes/record: ~37 s at 1kHz link stats + 100Hz A/D/C,
                                           // ~28 s while traction control logs (melty with throttle)
#define LOGGER_FLASH_OFFSET (1024 * 1024)  // log region starts 1MB into flash (firmware is ~120KB);
                                           // fits any flash chip >= 2MB
#define LOGGER_SAVE_DELAY_MS 2000          // killswitch must be on this long before saving
#define LOGGER_STATUS_INTERVAL_US 10000    // throttle + accel RPM record rate (100 Hz)
#define LOGGER_RSSI_RPM_INTERVAL_US 50000  // RSSI RPM estimate record rate (20 Hz)

//----------END FLASH LOGGER SETTINGS----------


//----------MOTOR SETTINGS----------

// ESC pins are connected as follows:
// MOTOR1_PIN was 2, MOTOR2_PIN was 3

// #define FLASHING_MOTORS 

#define MOTOR1_PIN 2 
#define MOTOR2_PIN 3

#define MOTOR1_PIO pio0
#define MOTOR2_PIO pio1

#define DSHOT_SPEED 600 // could be 150, 300, 450, 600 + some more

// motor magnet count / 2, converts the ESC's bidirectional DShot eRPM to motor shaft RPM.
// CHECK THIS against the motor spec: 7 is the usual 14-magnet (12N14P) outrunner.
#define MOTOR_POLE_PAIRS 7

// after boot, only zero throttle is sent for this long so the ESCs arm; the bot can't go live
// until it's over. Never block the main loop for this: a blocked loop doesn't read the killswitch.
#define MOTOR_ESC_ARM_HOLD_MS 3000

// after a flash write (log save, curve save) or another blocking operation, ignore all input this
// long, then require a healthy receiver (no failsafe, RC frames under INPUT_LOCKOUT_MAX_FRAME_AGE_US
// old) for INPUT_LOCKOUT_RX_HEALTHY_MS continuously, and the killswitch seen killed, before the bot
// can go live again
#define INPUT_LOCKOUT_MS 2000
#define INPUT_LOCKOUT_RX_HEALTHY_MS 1000
#define INPUT_LOCKOUT_MAX_FRAME_AGE_US 20000

#define DRIVE_MODE_MELTY 1
#define DRIVE_MODE_TANK 2

//----------END MOTOR SETTINGS----------
                  

//----------TELEMETRY SETTINGS----------

#define SHOULD_SEND_TELEMETRY_TO_TRANSMITTER

// #define OUTPUT_DIAGNOSTICS

// #define TIME_SINCE_BOOT_DIAGNOSTICS
#define ACCEL_DIAGNOSTICS
// #define RSSI_RPM_DIAGNOSTICS

// Link stats / RSSI debugging over USB serial. These print on their own schedule, so leave
// OUTPUT_DIAGNOSTICS off while using them (it prints every loop and would skew the timing).
// RSSI_LINK_DIAGNOSTICS: one "LINK ..." summary line per second (rates, gaps, RSSI, loop time)
// RSSI_RAW_DUMP: every link stats sample as "R,<timestamp_us>,<rssi>,<lq>" CSV lines
 #define RSSI_LINK_DIAGNOSTICS
// #define RSSI_RAW_DUMP

// one "ESC ..." line per second: bidirectional DShot eRPM/RPM and reply error counts per motor
// #define ESC_TELEMETRY_DIAGNOSTICS

// one "ACCEL REST ..." line per second while the bot is not live (killed/failsafed): whether the
// ~330 rpm the accelerometers read at standstill is a leftover sensor offset or noise. Bench only:
// it reads the accelerometers at 500 Hz while killed, which the bot otherwise doesn't do.
#define ACCEL_REST_DIAGNOSTICS
// #define FULL_CONTROLLER_DIAGNOSTICS
// #define OTHER_DIAGNOSTICS

#define TELEMETRY_MOTOR1 0
#define TELEMETRY_MOTOR2 1
#define TELEMETRY_MAIN 2

//----------END TELEMETRY SETTINGS----------


//----------MISC----------

#define WATCH_DOG_TIMEOUT_MS 500 // changed to prevent 200 microsecond delay from having any chance of triggering watchdog

//----------END MISC----------


#endif