#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "c_pico_dshot.h"
#include "donut_config.h"

void motor_init_all(int dshot_speed, int motor1_pin, PIO motor1_pio, int motor2_pin, PIO motor2_pio, bot_state_t* user_bot_state);

void motor_motor1_set_throttle(uint16_t throttle);

void motor_motor2_set_throttle(uint16_t throttle);

void motor_set_throttle_for_all(uint16_t throttle);

void motor_stop_all();

// true for MOTOR_ESC_ARM_HOLD_MS after motor_init_all: only zero throttle is sent meanwhile so the
// ESCs can arm (donut_is_live() stays false)
bool motor_esc_arm_hold_active(void);

// Stop sending DShot entirely (the ESCs see a clean signal loss and stop) / start again with zero
// throttle and a fresh MOTOR_ESC_ARM_HOLD_MS. Flash writes suspend the outputs: with interrupts off
// for each erase the DShot stream would otherwise break up into garbage the ESCs can misread.
void motor_outputs_suspend(void);
void motor_outputs_resume(void);
bool motor_outputs_suspended(void);

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