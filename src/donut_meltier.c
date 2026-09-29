#include "donut_config.h"

static bot_state_t bot_state = {
    #ifdef LIE_ABOUT_INPUT
        .is_failsafed = 0,
        .require_zero_throttle = 0,
    #else
        .is_failsafed = 1,
        .require_zero_throttle = 1,
    #endif

    .this_rotations_start_time_us = -1,
    .max_rpm = 0,
    .rpm = 0,
    .accel_g_value = 0,
    
    #ifdef DONUT_3LB_CONFIG
        .accel_offset_cm = 0.99
    #else 
        .accel_offset_cm = 0
    #endif
};

static pc_state_t throttle_pc_state = {
    .curr_target = 0,
    .curr_value = 0,
    .P = THROTTLE_PC_P,
    .last_error = 0
};

static accelerometer_t accel_1 = {
    .accelerometer_address = ACCELEROMETER_ADDRESS,
    .i2c_port = ACCEL_1_I2C_PORT,
    .i2c_sda = ACCEL_1_I2C_SDA,
    .i2c_scl = ACCEL_1_I2C_SCL
};

static accelerometer_t accel_2 = {
    .accelerometer_address = ACCELEROMETER_ADDRESS,
    .i2c_port = ACCEL_2_I2C_PORT,
    .i2c_sda = ACCEL_2_I2C_SDA,
    .i2c_scl = ACCEL_2_I2C_SCL
};

#ifdef LIE_ABOUT_INPUT
    uint8_t donut_is_throttle_zero() { 
        return 0;
    }

    uint8_t donut_is_killswitch_active() {
        return 0;
    }

    uint8_t donut_get_curr_drive_mode() {
        return DRIVE_MODE_MELTY;
    }

    uint8_t donut_should_spin_clockwise() {
        return 0;
    }
#else 
    uint8_t donut_is_throttle_zero() { 
        return receiver_is_channel_near_value(LEFT_JOYSTICK_Y, RECEIVER_LOWEST_CHANNEL_VALUE, 125);
    }

    uint8_t donut_is_killswitch_active() {
        return receiver_is_channel_near_value(SWITCH_E, RECEIVER_HIGHEST_CHANNEL_VALUE, 50);
    }

    uint8_t donut_get_curr_drive_mode() {
        if (receiver_is_channel_near_value(SWITCH_C, RECEIVER_HIGHEST_CHANNEL_VALUE, 50)) {
            return DRIVE_MODE_TANK;
        }
        return DRIVE_MODE_MELTY;
    }

    // there's a chance this should be "counter_clockwise" - Cai
    uint8_t donut_should_spin_clockwise() {
        if (receiver_is_channel_near_value(SWITCH_B, RECEIVER_LOWEST_CHANNEL_VALUE, 50)) {
            return 0;
        }
        return 1;
    }
#endif

// takes 0..1 to -1..1 (cubed for smoothing)
double input_remapping(double input) {
    input = input * 2; // 0..2
    input = input - 1; // -1..1
    return pow(input, 3); // still -1..1 but smoothed now
}

void when_flashing_motors() {
    led_repeat_blink(10);
}

void init_bot_systems() {
    #ifndef LIE_ABOUT_INPUT
        receiver_init(RECEIVER_UART_ID, RECEIVER_UART_TX_PIN, RECEIVER_UART_RX_PIN, 70, 105, &bot_state);
        accel_init(&accel_1, &accel_2, &bot_state);
    #endif
    
    motor_init_all(DSHOT_SPEED, MOTOR1_PIN, MOTOR1_PIO, MOTOR2_PIN, MOTOR2_PIO, &bot_state);

    watchdog_enable(WATCH_DOG_TIMEOUT_MS, 0);
}

// a just connected or just powered on bot starts here, this is also where failsafed bots go
void when_failsafe_on() {
    motor_stop_all();

    bot_state.require_zero_throttle = 1;

    bot_state.rpm = 0;
    bot_state.max_rpm = 0;
    bot_state.accel_g_value = 0;
    #ifdef DONUT_3LB_CONFIG
        bot_state.accel_offset_cm = 1;
    #else
        bot_state.accel_offset_cm = 0;
    #endif

    if (bot_state.is_failsafed == 0 && donut_is_throttle_zero()){
        bot_state.require_zero_throttle = 0;
    }

    led_time_blink(SLOW_BLINK);
}

void when_failsafe_off() {
    int8_t direction = 1;
    if (donut_should_spin_clockwise()) {
        direction = -1;
    }

    drive_update_bot_state(
        &bot_state, 
        &throttle_pc_state,
        #ifdef LIE_ABOUT_INPUT
            1,
            0.0,
            0.0,
            input_remapping(0),
        #else 
            direction * pow(receiver_get_percent_for_channel(LEFT_JOYSTICK_Y), 2), 
            input_remapping(receiver_get_percent_for_channel(LEFT_JOYSTICK_X)), 
            input_remapping(receiver_get_percent_for_channel(RIGHT_JOYSTICK_Y)), 
            input_remapping(receiver_get_percent_for_channel(RIGHT_JOYSTICK_X)),
        #endif

        #ifdef LIE_ABOUT_RPM
            get_fake_rpm
        #else
            #ifdef DONUT_3LB_CONFIG
                get_rpm_2accel
            #else
                get_rpm
            #endif
        #endif
    );
}

void always() {
    #ifndef LIE_ABOUT_INPUT
        #ifdef OUTPUT_DIAGNOSTICS
            telemetry_output_diagnostics(&bot_state);
        #endif

        #ifdef SHOULD_SEND_TELEMETRY_TO_TRANSMITTER
            telemetry_send_telemetry(&bot_state);
        #endif

        receiver_update();
    #endif

    watchdog_update(); // keep watchdog happy
}

#ifndef RUNNING_A_TEST
int main() {
    stdio_init_all();

    // calling led_init() early so when_flashing_motors() can use it
    led_init(HEADING_LIGHT_STRIP_PIN, HEADING_LIGHT_STRIP_PIN2);

    #ifdef FLASHING_MOTORS
        while(1){
            when_flashing_motors();
        }
    #endif

    init_bot_systems();

    while(1){
        if (bot_state.is_failsafed || bot_state.require_zero_throttle || donut_is_killswitch_active()){
            when_failsafe_on();
        } else {
            when_failsafe_off();
        }

        always();
    }

    return 0;
}
#endif
