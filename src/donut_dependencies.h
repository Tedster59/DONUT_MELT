#ifndef DONUT_DEPENDENCIES_H
#define DONUT_DEPENDENCIES_H

#include <math.h>
#include <stdio.h>
#include <stdint.h>

#define RPM_WINDOW_SIZE 16

typedef struct {
    uint8_t is_failsafed;
    uint8_t require_zero_throttle;

    uint64_t this_rotations_start_time_us;

    uint32_t max_rpm;
    uint32_t rpm;
    uint32_t rpm_buff[10];

    // raw_accel_g_value * 100 with the decimals cut off
    uint16_t accel_g_value;
    double accel_offset_cm;
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
#include "donut_drive.h"
#include "donut_telemetry.h"

#endif