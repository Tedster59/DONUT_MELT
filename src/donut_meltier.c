#include "donut_config.h"

static bot_state_t bot_state = {
    #ifdef LIE_ABOUT_INPUT
        .is_failsafed = 0,
        .require_zero_throttle = 0,
    #else
        .is_failsafed = 1,
        .require_zero_throttle = 1,
    #endif

    .this_rotations_start_time_us = ROTATION_NOT_STARTED,
    .max_rpm = 0,
    .rpm = 0,
    .accel_g_value = 0,

    // 1lb only (cm added to the accel mount radius). The 3lb uses the RPM correction curve instead,
    // which is loaded from flash at boot (donut_rpm_curve.c).
    .accel_offset_cm = 0,
    .rpm_correction = 1.0f
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

    // SWITCH_E is CH5/AUX1, which ELRS treats as its arm switch (high = armed) and sends in
    // every packet. Live = high so ELRS's armed state matches the bot actually running
    // (dynamic power, Race telemetry, etc.). Anything that isn't clearly high, including
    // the all-zero channels before the first RC frame arrives, counts as killed.
    static bool killswitch_raw_active() {
        return !receiver_is_channel_near_value(SWITCH_E, RECEIVER_HIGHEST_CHANNEL_VALUE, 50);
    }

    // Debounced killswitch: the raw reading has to disagree with the current state for
    // KILLSWITCH_DEBOUNCE_MS before it takes effect, so one bad RC frame can neither kill a spinning
    // bot nor arm a killed one. (If RC frames stop arriving altogether, RECEIVER_RC_TIMEOUT_US
    // failsafes instead, so stale channels can't hold the bot live.)
    static bool killswitch_active = true;            // boots killed
    static uint64_t killswitch_pending_since_us = 0; // 0 = raw reading agrees with killswitch_active
    static uint16_t killswitch_pending_ch5 = 0;      // CH5 when the disagreement started

    static void killswitch_update() {
        uint64_t now = time_us_64();
        bool raw = killswitch_raw_active();
        if (raw == killswitch_active) {
            if (killswitch_pending_since_us != 0) {
                // flipped back before the debounce ran out: a glitch, filtered
                #ifdef USE_FLASH_LOGGER
                    logger_log_event('G', killswitch_active ? LOG_GLITCH_READ_ARMED : LOG_GLITCH_READ_KILLED,
                        (uint32_t)(now - killswitch_pending_since_us), killswitch_pending_ch5);
                #endif
                killswitch_pending_since_us = 0;
            }
            return;
        }
        if (killswitch_pending_since_us == 0) {
            killswitch_pending_since_us = now;
            killswitch_pending_ch5 = receiver_get_channel(SWITCH_E);
            return;
        }
        if (now - killswitch_pending_since_us >= (uint64_t)KILLSWITCH_DEBOUNCE_MS * 1000) {
            killswitch_active = raw;
            #ifdef USE_FLASH_LOGGER
                logger_log_event(raw ? 'K' : 'k', 0, (uint32_t)(now - killswitch_pending_since_us), killswitch_pending_ch5);
            #endif
            killswitch_pending_since_us = 0;
        }
    }

    uint8_t donut_is_killswitch_active() {
        return killswitch_active;
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
        #ifdef USE_FLASH_LOGGER
            logger_init();
        #endif
    #endif

    rpm_curve_init();

    motor_init_all(DSHOT_SPEED, MOTOR1_PIN, MOTOR1_PIO, MOTOR2_PIN, MOTOR2_PIO, &bot_state);

    watchdog_enable(WATCH_DOG_TIMEOUT_MS, 0);
}

// ---------- input lockout after the main loop was blocked (see donut_note_loop_blocked) ----------
static uint64_t loop_blocked_us = 0;        // when the last blocking operation ended (0 = none since boot)
static uint64_t rx_healthy_since_us = 0;    // start of the current stretch of healthy receiver input
static bool killed_seen_after_lockout = true;

void donut_note_loop_blocked(void) {
    loop_blocked_us = time_us_64();
    killed_seen_after_lockout = false;
}

static bool input_lockout_active(void) {
    if (loop_blocked_us == 0) {
        return false;
    }
    uint64_t now = time_us_64();
    if (now - loop_blocked_us < (uint64_t)INPUT_LOCKOUT_MS * 1000) {
        return true;
    }
    // healthy for long enough, counting only from after the block (input buffered during the block
    // looks fresh when it's finally parsed)
    uint64_t healthy_from = rx_healthy_since_us > loop_blocked_us ? rx_healthy_since_us : loop_blocked_us;
    if (rx_healthy_since_us == 0 || now - healthy_from < (uint64_t)INPUT_LOCKOUT_RX_HEALTHY_MS * 1000) {
        return true;
    }
    return !killed_seen_after_lockout;
}

// runs every loop: receiver health, the killswitch check that ends the lockout, and restarting the
// motor outputs once a flash write is over
static void input_lockout_update(void) {
    uint64_t now = time_us_64();
    bool healthy = !bot_state.is_failsafed && receiver_rc_frame_age_us() < INPUT_LOCKOUT_MAX_FRAME_AGE_US;
    if (!healthy) {
        rx_healthy_since_us = 0;
    } else if (rx_healthy_since_us == 0) {
        rx_healthy_since_us = now;
    }
    if (loop_blocked_us != 0 && now - loop_blocked_us >= (uint64_t)INPUT_LOCKOUT_MS * 1000 &&
        donut_is_killswitch_active()) {
        killed_seen_after_lockout = true;
    }
    // flash writes stop the DShot output; the loop running again means the writing is over
    if (motor_outputs_suspended()) {
        motor_outputs_resume();
    }
}

// true when the drive code is allowed to run the motors
static bool donut_is_live() {
    return !(bot_state.is_failsafed || bot_state.require_zero_throttle || donut_is_killswitch_active() ||
             motor_esc_arm_hold_active() || input_lockout_active());
}

// a just connected or just powered on bot starts here, this is also where failsafed bots go
void when_failsafe_on() {
    motor_stop_all();

    bot_state.require_zero_throttle = 1;

    // forget the previous spin's rotation timing so the first melty loop after going live
    // reads a fresh RPM instead of reusing the last spin's RPM and start time
    bot_state.this_rotations_start_time_us = ROTATION_NOT_STARTED;

    bot_state.rpm = 0;
    bot_state.max_rpm = 0;
    bot_state.accel_g_value = 0;
    bot_state.rpm_source = RPM_SOURCE_ACCEL;
    bot_state.drive_rpm = 0;
    bot_state.accel_rpm_corrected = 0;
    traction_reset();
    // (the rpm correction is no longer reset here: the trim survives kills, saved points survive
    // power cycles, and holding CH10 for 3 s while killed clears back to the default)

    if (bot_state.is_failsafed == 0 && donut_is_throttle_zero()){
        bot_state.require_zero_throttle = 0;
    }

    // fast blink = locked out after a flash write until the receiver and killswitch check out
    led_time_blink(input_lockout_active() || motor_esc_arm_hold_active() ? FAST_BLINK : SLOW_BLINK);
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
            #if defined(DONUT_3LB_CONFIG) && defined(USE_RSSI_RPM)
                get_rpm_rssi
            #elif defined(DONUT_3LB_CONFIG)
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
        killswitch_update();
        input_lockout_update();
        rssi_rpm_update();

        #if defined(RSSI_LINK_DIAGNOSTICS) || defined(RSSI_RAW_DUMP)
            telemetry_output_link_diagnostics(&bot_state);
        #endif

        #ifdef ESC_TELEMETRY_DIAGNOSTICS
            telemetry_output_esc_diagnostics();
        #endif

        #if defined(ACCEL_REST_DIAGNOSTICS) && defined(DONUT_3LB_CONFIG)
            telemetry_output_accel_rest_diagnostics(donut_is_live());
        #endif

        #ifdef USE_FLASH_LOGGER
            logger_update(&bot_state, donut_is_live(), donut_is_killswitch_active());
        #endif

        #ifdef DONUT_3LB_CONFIG
            rpm_curve_update(&bot_state, donut_is_live(), donut_is_killswitch_active());
            traction_update_switch(&bot_state);
        #endif
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
        if (!donut_is_live()){
            when_failsafe_on();
        } else {
            when_failsafe_off();
        }

        always();
    }

    return 0;
}
#endif
