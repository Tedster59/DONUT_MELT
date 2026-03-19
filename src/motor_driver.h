#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

// Used in header file and c file
#include "c_pico_dshot.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "donut_config.h"

// In motor_driver.h or donut_config.h
typedef struct {
    uint16_t motor1_throttle;
    uint16_t motor2_throttle;
    uint32_t send_count;
} motor_debug_snapshot_t;

extern volatile motor_debug_snapshot_t motor_debug_snapshot;

void motor_init_all(int dshot_speed, int motor1_pin, PIO motor1_pio, int motor2_pin, PIO motor2_pio, bot_state_t* user_bot_state);

void motor_motor1_set_throttle(uint16_t throttle);

void motor_motor2_set_throttle(uint16_t throttle);

void motor_set_throttle_for_all(uint16_t throttle);

void motor_stop_all();

void motor_update_bot_state();

void motor_shutdown_all();

#endif