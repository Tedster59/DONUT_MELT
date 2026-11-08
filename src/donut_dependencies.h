#ifndef DONUT_DEPENDENCIES_H
#define DONUT_DEPENDENCIES_H

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// this_rotations_start_time_us value meaning "no rotation in progress": the next melty loop reads
// a fresh RPM and starts timing from scratch (was written as -1, which wraps to this anyway)
#define ROTATION_NOT_STARTED UINT64_MAX

typedef struct {
    uint8_t is_failsafed;
    uint8_t require_zero_throttle;

    uint64_t this_rotations_start_time_us;

    uint32_t max_rpm;
    uint32_t rpm;

    // raw_accel_g_value * 100 with the decimals cut off
    uint16_t accel_g_value;
    double accel_offset_cm;

    // latest RSSI-based RPM estimate (0 when unavailable) and how tone-like the
    // RSSI was at that frequency (~1.0 = pure sinusoid, near 0 = noise)
    uint32_t rssi_rpm;
    float rssi_rpm_confidence;
} bot_state_t;

typedef struct {
    double curr_target;
    double curr_value;
    double P;
    double D;
    double last_error;
} pc_state_t;

#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"

#include "pico/stdlib.h"
#include "pico/time.h"

#include "H3LIS331DL.h"

#include "led_driver.h"
#include "motor_driver.h"
#include "receiver.h"

#include "donut_pc.h"
#include "donut_accel.h"
#include "donut_rssi_rpm.h"
#include "donut_drive.h"
#include "donut_telemetry.h"

#endif