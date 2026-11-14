#ifndef DONUT_RPM_CURVE_H
#define DONUT_RPM_CURVE_H

#include "donut_config.h"

// RPM correction curve for the 2-accelerometer RPM (3lb).
//
// The accelerometer RPM is not linearly accurate across the speed range, so the corrected RPM is
//   corrected = raw_rpm * clamp(curve(raw_rpm) * trim, RPM_CORRECTION_MIN, RPM_CORRECTION_MAX)
// where curve() linearly interpolates between up to RPM_CURVE_MAX_POINTS saved (raw rpm, factor)
// points (flat beyond the first/last point, 1.0 when empty) and trim is the live left-stick-X
// adjustment.
//
// Tuning: spin at a steady speed, trim with left stick X until the heading holds, then press the
// CH10 momentary (RPM_CURVE_SAVE_SWITCH). That stores curve*trim at the current raw RPM (updating
// a point within RPM_CURVE_MERGE_FRACTION, otherwise adding one, otherwise replacing the nearest)
// and resets trim to 1.0, so nothing jumps. Repeat at other speeds.
//
// With RPM_CURVE_SAVE_FROM_RSSI: if the RSSI estimate is locked and the speed has been steady for the
// last ~0.5 s, CH10 instead saves the measured factor (rssi rpm / raw accel rpm, averaged over the
// RSSI window), no hand trimming needed: hold the speed steady for a second, then press. This works
// in shadow mode too. While RSSI is driving, left stick X trim is ignored and CH10 does nothing
// unless the speed is steady enough to measure.
// Points are written to flash once the killswitch has been on for RPM_CURVE_PERSIST_DELAY_MS, and
// reloaded at boot. Holding CH10 for RPM_CURVE_CLEAR_HOLD_MS while killed clears the curve
// (back to the default: no points, trim 1.0, i.e. factor 1.0).

void rpm_curve_init(void);

// total correction factor (curve * trim, clamped) for a raw accelerometer rpm
double rpm_curve_factor(double raw_rpm);

// left stick X trim; stick is -1..1, dt_us is the time since the previous call
void rpm_curve_adjust_trim(double stick, uint64_t dt_us);

// call every main loop: handles the save/clear input and persisting to flash
void rpm_curve_update(bot_state_t* bot_state, bool live, bool killed);

uint8_t rpm_curve_point_count(void);
double rpm_curve_trim(void);

#endif
