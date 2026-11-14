#ifndef DONUT_RSSI_RPM_H
#define DONUT_RSSI_RPM_H

#include "donut_config.h"

// Estimates bot RPM from the periodic pattern the spinning antenna puts on the
// receiver's uplink RSSI. The accelerometer RPM (bot_state->rpm, kept current by
// get_rpm_2accel) is used as the starting point: only a narrow band of frequencies
// around it is searched, which also lets us reject aliases / harmonics.

void rssi_rpm_init(bot_state_t* user_bot_state);

// feed one link statistics frame (called from the CRSF link stats callback)
void rssi_rpm_add_sample(uint64_t timestamp_us, uint8_t rssi);

// recomputes the estimate at most once per RSSI_RPM_UPDATE_INTERVAL_US; call every loop
void rssi_rpm_update(void);

// same signature as get_rpm/get_rpm_2accel so it can be handed to drive_update_bot_state.
// Uses the RSSI RPM when it is recent and confident, otherwise falls back to get_rpm_2accel.
double get_rpm_rssi(double right_x_percent, double accel_offset_cm);

// true while there is a recent (RSSI_RPM_MAX_AGE_US), confident, non-edge RSSI estimate
bool rssi_rpm_is_locked(void);

// RSSI_STATE_UNLOCKED / _SHADOW / _DRIVING
uint8_t rssi_rpm_state(void);

// RPM curve calibration from RSSI: true when RSSI is locked AND the speed has been steady over the
// whole RSSI window (accel spread and consecutive RSSI estimates within RSSI_CAL_MAX_SPREAD).
// raw_rpm is the raw accelerometer rpm averaged over the same window the RSSI estimate covers,
// factor = rssi rpm / raw_rpm, i.e. exactly the curve correction factor at raw_rpm.
bool rssi_rpm_get_calibration(double* raw_rpm, double* factor);

#endif
