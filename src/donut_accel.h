#ifndef DONUT_ACCEL_H
#define DONUT_ACCEL_H

#include "donut_config.h"

typedef struct {
    float x;
    float y;
} Vector2D;

void accel_init(accelerometer_t* accel_1, accelerometer_t* accel_2, bot_state_t* user_bot_state);

double get_rpm(double right_x_percent, double accel_offset_cm);

double get_fake_rpm(double right_x_percent, double accel_offset_cm);

double get_rpm_2accel(double right_x_percent, double accel_offset_cm);

// 3lb: raw two-accelerometer rpm with no smoothing, correction or bot_state side effects
double accel_read_raw_rpm_2accel(void);

// 3lb: accelerometer 1 minus accelerometer 2 (X/Y, g, offsets applied), the vector the rpm comes from
Vector2D accel_read_delta_g(void);

// 3lb: 'z' USB command. Averages both sensors for 5 s (blocking, bot must not be live) and prints
// the ACCEL_x_OFFSET #defines for donut_3lb_config.h
void accel_measure_offsets(void);

#endif