#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "c_pico_dshot.h"
#include "donut_config.h"

void motor_init_all(int dshot_speed, int motor1_pin, PIO motor1_pio, int motor2_pin, PIO motor2_pio, bot_state_t* user_bot_state);

void motor_motor1_set_throttle(uint16_t throttle);

void motor_motor2_set_throttle(uint16_t throttle);

void motor_set_throttle_for_all(uint16_t throttle);

void motor_stop_all();

// Bidirectional DShot telemetry for one motor (1 or 2). The ESCs answer every DShot frame
// (5 kHz) with their electrical RPM; rpm = erpm / MOTOR_POLE_PAIRS is the motor shaft speed.
// The eRPM has no direction (it's the same forward and reverse in 3D mode).
typedef struct {
    uint32_t erpm;
    uint32_t rpm;
    uint32_t erpm_age_us;     // time since the last valid eRPM packet (UINT32_MAX = never)
    uint32_t erpm_packets;    // running totals since boot
    uint32_t checksum_errors;
    uint32_t missing;
    uint32_t other_packets;
} motor_telemetry_t;

// returns false if no eRPM packet has been received yet from that motor
bool motor_get_telemetry(uint8_t motor, motor_telemetry_t* out);

void motor_update_bot_state();

#endif