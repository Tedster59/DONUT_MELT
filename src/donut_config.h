#ifndef DONUT_CONFIG_H
#define DONUT_CONFIG_H

#include "donut_dependencies.h"

void donut_init_bot_state();

uint8_t donut_is_throttle_zero();

uint8_t donut_is_killswitch_active();

uint8_t donut_get_curr_drive_mode();

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

//----------END REICEVER SETTINGS----------


//----------RSSI RPM SETTINGS----------

// uses get_rpm_rssi (RSSI estimate, falling back to get_rpm_2accel) for the 3lb drive loop
// #define USE_RSSI_RPM

#define RSSI_RPM_GRID_INTERVAL_US 1000     // RSSI resample spacing: matches the 1kHz packet rate, Nyquist = 30000 RPM per RSSI cycle/rotation
#define RSSI_RPM_BUF_SIZE 512              // grid samples analysed: 512 * 1ms = 512ms window
#define RSSI_RPM_MAX_GAP_US 250000         // link stats gap that discards the history
#define RSSI_RPM_UPDATE_INTERVAL_US 50000  // how often the estimate is recomputed (~0.1-0.2ms of CPU each)
#define RSSI_RPM_MIN_SEED_RPM 300          // don't search until the accel says we're spinning
#define RSSI_RPM_SEARCH_FRACTION 0.3f      // search accel_rpm * (1 +/- this)
#define RSSI_RPM_SEARCH_STEPS 48           // frequencies evaluated across that band
#define RSSI_RPM_HARMONIC 1                // RSSI cycles per rotation (2 if the antenna pattern has two lobes)
#define RSSI_RPM_MIN_CONFIDENCE 0.25f      // minimum confidence for get_rpm_rssi to trust the estimate
#define RSSI_RPM_MAX_AGE_US 200000         // older than this and get_rpm_rssi falls back to accel

//----------END RSSI RPM SETTINGS----------


//----------RPM CORRECTION CURVE SETTINGS (3lb)----------

// corrected rpm = accel rpm * clamp(curve(accel rpm) * trim). See donut_rpm_curve.h for tuning steps.
#define RPM_CORRECTION_MIN 0.5
#define RPM_CORRECTION_MAX 1.5
#define RPM_TRIM_RATE_PER_SEC 0.35          // left stick X trim speed (matches the old per-loop step at ~3.5kHz)
#define RPM_TRIM_DEADZONE 0.05              // left stick X (after input_remapping) deadzone
#define RPM_CURVE_MAX_POINTS 8
#define RPM_CURVE_MERGE_FRACTION 0.05f      // saving within 5% of an existing point's rpm updates that point
#define RPM_CURVE_MIN_SAVE_RPM 300          // ignore save presses below this accel rpm
#define RPM_CURVE_CLEAR_HOLD_MS 3000        // hold the save switch this long while killed to clear the curve
#define RPM_CURVE_PERSIST_DELAY_MS 2000     // killswitch must be on this long before writing to flash
#define RPM_CURVE_FLASH_OFFSET (1024 * 1024 - 4096) // one sector just below the run logger's region

//----------END RPM CORRECTION CURVE SETTINGS----------


//----------FLASH LOGGER SETTINGS----------

// Records RSSI samples + RPM estimates in RAM while the bot is live, saves the run to flash once
// the killswitch has been on for LOGGER_SAVE_DELAY_MS (LED goes solid while saving, a few seconds),
// and dumps it over USB serial later: d = dump CSV, x = erase, s = status, h = help.
// Saving only happens on the killswitch, so kill the bot and wait for the save before cutting power.
#define USE_FLASH_LOGGER

#define LOGGER_BUF_BYTES (384 * 1024)      // RAM ring buffer, 8 bytes/record: ~40 s at 1kHz link stats
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