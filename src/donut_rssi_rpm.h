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

#endif
