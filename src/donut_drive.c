#include "donut_drive.h"

// takes into account curr_drive_mode whereas donut_is_throttle_zero does not 
uint8_t drive_is_throttle_zero() {
    if (donut_get_curr_drive_mode() == DRIVE_MODE_MELTY) {
        return donut_is_throttle_zero();
    }

    if (donut_get_curr_drive_mode() == DRIVE_MODE_TANK) {
        return receiver_is_channel_near_value(RIGHT_JOYSTICK_X, RECEIVER_MIDDLEST_CHANNEL_VALUE, 150)
        && receiver_is_channel_near_value(RIGHT_JOYSTICK_Y, RECEIVER_MIDDLEST_CHANNEL_VALUE, 150);
    }
}

uint8_t is_close_enough(double num1, double num2, double tolerance) {
    return fabs(num1 - num2) <= tolerance;
}

// goes from rpm -> microseconds per rotation
double rpm_to_upr(double rpm) {
    if (rpm == 0) {
        return 0; 
    }
    return (uint64_t) 60000000.0 / rpm;
}

uint16_t percentThrottleToThrottleCommand(double percent_throttle) {
    uint16_t throttle;

    // -1..0 -> 999..0 AND 0..1 -> 1001..2000
    // 0 percent_throttle or 0.5 percent_throttle becomes a 0 throttle command
    if (percent_throttle > 0) {
        throttle = 1001 + 999*percent_throttle;
    } else if (percent_throttle < 0) {
        throttle = -999*percent_throttle;
    } else {
        throttle = 0;
    }

    return throttle;
}

void handle_idle(bot_state_t* bot_state, double left_y_percent, double right_y_percent, double right_x_percent) {
    motor_stop_all();
}

void handle_tank(bot_state_t* bot_state, double left_y_percent, double right_y_percent, double right_x_percent) {
    uint16_t left_throttle = percentThrottleToThrottleCommand(left_y_percent);
    uint16_t right_throttle = percentThrottleToThrottleCommand(right_y_percent);

    #ifndef NO_MOTOR_SPINNING
        motor_motor1_set_throttle(left_throttle);
        motor_motor2_set_throttle(right_throttle);
    #endif
}

void handle_one_stick_tank(bot_state_t* bot_state, double right_y_percent, double right_x_percent) {
    if (right_y_percent != 0) {
        handle_tank(bot_state, right_y_percent, -right_y_percent, right_x_percent);
        return;
    } 

    handle_tank(bot_state, -right_x_percent, -right_x_percent, right_x_percent);
}

void handle_all_spin(bot_state_t* bot_state, double left_y_percent, double right_y_percent, uint64_t time_elapsed_this_rotation_us, uint64_t us_per_rotation, double half_rotation_time, double motor_off_edge_time) {
    double distance_to_edge = fmin(1 - fabs(left_y_percent), fabs(left_y_percent));  

    // here's a desmos link for what the calculated motor throttles 
    // look like for various values of left_y_percent and right_y_percent
    // link here: https://www.desmos.com/calculator/iwp9mwik4o

    // the names "more" and "less" are true in the case of moving forwards 
    // and reversed in the case of moving backwards

    // Sinusoidal control: modulate it based on where you are in the rotation:

    double angle = (2 * M_PI) * ((double)time_elapsed_this_rotation_us / (double)us_per_rotation);
    double sinVal = cos(angle);

    double more_motor_percent_throttle = left_y_percent + distance_to_edge * right_y_percent * sinVal;
    double less_motor_percent_throttle = left_y_percent + distance_to_edge * -right_y_percent * sinVal;

    // printf("more_motor_percent_throttle=%lf | ", more_motor_percent_throttle);
    // printf("less_motor_percent_throttle=%lf \n", less_motor_percent_throttle);

    handle_tank(bot_state, more_motor_percent_throttle, less_motor_percent_throttle, 0);
    
    // No longer needed with sinusoidal control. it auto-handles it :)

    // if (time_elapsed_this_rotation_us >= motor_off_edge_time &&
    //    time_elapsed_this_rotation_us <= half_rotation_time - motor_off_edge_time) {
    //    handle_tank(bot_state, more_motor_percent_throttle, less_motor_percent_throttle, 0);
    // } else if (time_elapsed_this_rotation_us >= half_rotation_time + motor_off_edge_time &&
    //    time_elapsed_this_rotation_us <= us_per_rotation - motor_off_edge_time) {
    //    handle_tank(bot_state, less_motor_percent_throttle, more_motor_percent_throttle, 0);    
    //}
}

void handle_spin_led(uint64_t time_elapsed_this_rotation_us, uint64_t us_per_rotation, uint64_t led_on_us) {
    // time_elapsed_this_rotation_us == 0 is "forward" for the sake of spin led purposes

    // if facing left side or right side of 0 within led_on_us/2 us
    if (time_elapsed_this_rotation_us >= us_per_rotation - led_on_us/2 || 
        time_elapsed_this_rotation_us <= led_on_us/2) {
        led_set_and_update_state(1);
    } else {
        led_set_and_update_state(0);
    }
}

void handle_spin(bot_state_t* bot_state, double left_y_percent, double right_y_percent, double right_x_percent, double (*get_rpm)(double, double)) {
    // we only want to calculate RPM once per rotation
    // Accordingly, there's technically some other stuff 
    // we could avoid recalculating but it's mostly just 
    // math calls so I am gonna ignore it for now! - Cai
    static double rpm; 
    
    if (bot_state->this_rotations_start_time_us == -1) {
        rpm = get_rpm(right_x_percent, bot_state->accel_offset_cm);
        bot_state->this_rotations_start_time_us = time_us_64();
    }

    // if we aren't fast enough to translate, spin up as fast as possible (can be bypassed)
    #ifndef BYPASS_MIN_TRANSLATION_RPM
        if (rpm < MIN_TRANSLATION_RPM) {
            handle_tank(bot_state, 1, 1, 0);
            led_set_and_update_state(1);
            return;
        }
    #endif

    double us_per_rotation = rpm_to_upr(rpm);
    double time_elapsed_this_rotation_us = time_us_64() - bot_state->this_rotations_start_time_us;

    double led_on_us = fmin(MAX_LED_PERCENT_DURATION, fmax(MIN_LED_PERCENT_DURATION, fabs(left_y_percent))) * us_per_rotation;

    double half_rotation_time = us_per_rotation/2;
    double motor_off_edge_time = (half_rotation_time - MOTOR_ON_PERCENT_DURATION*us_per_rotation)/2;

    if (is_close_enough(right_y_percent, 0, 0.125)) {
        // this should just fully spin in circles
        handle_all_spin(bot_state, left_y_percent, 0, time_elapsed_this_rotation_us, us_per_rotation, half_rotation_time, motor_off_edge_time);
    } else {
        // this should translate in the direction dictated by right_y_percent
        handle_all_spin(bot_state, left_y_percent, right_y_percent, time_elapsed_this_rotation_us, us_per_rotation, half_rotation_time, motor_off_edge_time);
    }

    // the heading led triggers at an offset time from the "0" point because of LED_OFFSET_PERCENT 
    // if the robot is spinning in reverse then the offset time needs to change to us_per_rotation - led_offset_us

    // this makes the led blink at the "end" of a rotation as opposed to the "start" 
    // The above line assumes that LED_OFFSET_PERCENT < 0.5, (it still works if it is > 0.5 tho because modulo) 

    // desmos graph with the math: https://www.desmos.com/calculator/cpiiyqxjgb
    
    float led_offset_us = LED_OFFSET_PERCENT*us_per_rotation;
    if (left_y_percent < 0) {
        led_offset_us = us_per_rotation-led_offset_us;
    }
    handle_spin_led(fmod(time_elapsed_this_rotation_us+led_offset_us, us_per_rotation), us_per_rotation, led_on_us);

    // if we have completed a rotation, get ready for next rotation
    if (time_elapsed_this_rotation_us >= us_per_rotation) {
        // doing more expensive calls before resetting time_elapsed_this_rotation_us
        bot_state->this_rotations_start_time_us = time_us_64();
        rpm = get_rpm(right_x_percent, bot_state->accel_offset_cm);
    }
}

// -1..1 -> -max..max
double rescalePercentThrottle(double percent_throttle, double max, bool use_deadzone) {
    if (use_deadzone && percent_throttle >= -0.05 && percent_throttle <= 0.05) {
        return 0;
    }
    return percent_throttle * max;
}

// assumes all percents given are -1..1
void drive_update_bot_state(bot_state_t* bot_state, pc_state_t* throttle_pc_state, double left_y_percent, double left_x_percent, double right_y_percent, double right_x_percent, double (*get_rpm)(double, double)) { 
    static uint64_t last_loop = 0;

    // printf("throttle_pc_state->curr_value=%lf \n",throttle_pc_state->curr_value);
    pc_update_pc_state(throttle_pc_state, time_us_64() - last_loop);

    #ifdef CAN_ADJUST_ACCEL_MOUNT_RADIUS
        if (left_x_percent <= -0.25 || left_x_percent >= 0.25) {
            float delta = ACCEL_OFFSET_SENSITIVITY*(left_x_percent/fabs(left_x_percent));
            bot_state->accel_offset_cm += delta;
            if (ACCEL_MOUNT_RADIUS_CM + bot_state->accel_offset_cm <= 0) {
                bot_state->accel_offset_cm -= delta;
            }
        }
    #endif
    
    // getting rpm
    #ifdef LIE_ABOUT_RPM
        double raw_rpm = get_rpm(right_x_percent, bot_state->accel_offset_cm);
    #else 
        double raw_rpm = get_rpm(0, bot_state->accel_offset_cm);
    #endif
    bot_state->rpm = raw_rpm;

    // keeping track of max rpm
    if (bot_state->rpm > bot_state->max_rpm) {
        bot_state->max_rpm = bot_state->rpm;
    }

    if(drive_is_throttle_zero()) {
        if (donut_get_curr_drive_mode() == DRIVE_MODE_MELTY) {
            led_repeat_blink(4);
        } else {
            led_repeat_blink(5);
        }

        throttle_pc_state->curr_target = 0;
        throttle_pc_state->curr_value = 0;
        handle_idle(bot_state, left_y_percent, right_y_percent, right_x_percent);
        last_loop = time_us_64();
        return;
    }

    if (donut_get_curr_drive_mode() == DRIVE_MODE_MELTY) {
        throttle_pc_state->curr_target = rescalePercentThrottle(left_y_percent, MELTY_MAX_THROTTLE, false);
        handle_spin(
            bot_state, 
            throttle_pc_state->curr_value, 
            rescalePercentThrottle(right_y_percent, MELTY_MAX_TRANSLATION_AGGRESSION, true), 
            right_x_percent, 
            get_rpm
        );
        last_loop = time_us_64();
        return;
    } 
    
    if (donut_get_curr_drive_mode() == DRIVE_MODE_TANK) {
        led_repeat_blink(3);
        handle_one_stick_tank(
            bot_state, 
            rescalePercentThrottle(right_y_percent, TANK_DRIVE_MAX_THROTTLE, true), 
            rescalePercentThrottle(right_x_percent, TANK_DRIVE_TURNING_MAX_THROTTLE, true)
        );
        last_loop = time_us_64();
        return;
    }
}
