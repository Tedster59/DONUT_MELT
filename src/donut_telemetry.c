#include "donut_telemetry.h"

uint32_t get_seconds_since_boot() {
    return to_ms_since_boot(get_absolute_time()) / 1000.0;
}

void telemetry_output_diagnostics(bot_state_t* bot_state) {
    printf("\n---------DIAGNOSTICS START-------------\n\n");

    #ifdef TIME_SINCE_BOOT_DIAGNOSTICS
        printf("DIAGNOSTICS AT %d SECS SINCE BOOT \n\n", get_seconds_since_boot());
    #endif

    #ifdef ACCEL_DIAGNOSTICS
        printf("rpm: %d | ", bot_state->rpm);
        printf("accel_g_value: %d | ", bot_state->accel_g_value);
        printf("accel_offset_cm: %lf \n\n", bot_state->accel_offset_cm);
    #endif

    #ifdef RSSI_RPM_DIAGNOSTICS
        printf("rssi_rpm: %lu | ", (unsigned long)bot_state->rssi_rpm);
        printf("rssi_rpm_confidence: %f \n\n", bot_state->rssi_rpm_confidence);
    #endif

    #ifdef FULL_CONTROLLER_DIAGNOSTICS
        printf("--------FULL CONTROLLER INFO---------\n");
        printf("RIGHT_JOYSTICK_X: %d | ", receiver_get_channel(RIGHT_JOYSTICK_X));
        printf("RIGHT_JOYSTICK_Y: %d \n", receiver_get_channel(RIGHT_JOYSTICK_Y));
        printf("LEFT_JOYSTICK_Y: %d | ", receiver_get_channel(LEFT_JOYSTICK_Y));
        printf("LEFT_JOYSTICK_X: %d \n", receiver_get_channel(LEFT_JOYSTICK_X));
        printf("SWITCH_E: %d | ", receiver_get_channel(SWITCH_E));
        printf("SWITCH_B: %d | ", receiver_get_channel(SWITCH_B));
        printf("SWITCH_C: %d | ", receiver_get_channel(SWITCH_C));
        printf("SWITCH_F: %d \n", receiver_get_channel(SWITCH_F));
        printf("KNOB_S1: %d | ", receiver_get_channel(KNOB_S1));
        printf("KNOB_S2: %d \n\n", receiver_get_channel(KNOB_S2));
    #endif

    #ifdef OTHER_DIAGNOSTICS
        printf("--------OTHER INFO---------\n");
        printf("is_failsafed: %d \n", bot_state->is_failsafed);
        printf("require_zero_throttle: %d \n", bot_state->require_zero_throttle);
        printf("get_curr_drive_mode: %d \n", drive_get_curr_drive_mode());
        printf("is_killswitch_active: %d \n", donut_is_killswitch_active());
        printf("is_throttle_zero: %d \n", donut_is_throttle_zero());
        printf("left_stick_y_percent: %lf \n", receiver_get_percent_for_channel(LEFT_JOYSTICK_Y));
        printf("right_stick_y_percent: %lf \n\n", receiver_get_percent_for_channel(RIGHT_JOYSTICK_Y));
    #endif

    printf("---------DIAGNOSTICS END-------------\n\n");
}

#define LINK_DIAGNOSTICS_INTERVAL_US 1000000
#define RAW_DUMP_LINES_PER_LOOP 4 // keeps each loop's printing short so UART bytes aren't missed

void telemetry_output_link_diagnostics(bot_state_t* bot_state) {
    static uint64_t window_start_us = 0;
    static uint64_t last_call_us = 0;
    static uint32_t loops = 0;
    static uint32_t max_loop_us = 0;
    static uint32_t last_rc_frames = 0;
    static uint32_t last_link_frames = 0;

    uint64_t now = time_us_64();

    // main loop timing: the Pico's UART FIFO is 32 bytes (~0.76ms at 420k baud), so a
    // loop slower than that can drop CRSF bytes and lose frames to CRC errors
    if (last_call_us != 0 && now - last_call_us > max_loop_us) {
        max_loop_us = (uint32_t)(now - last_call_us);
    }
    loops++;

    #ifdef RSSI_RAW_DUMP
        // CSV lines: R,<timestamp_us>,<rssi (-dBm)>,<link quality %>
        receiver_raw_sample_t sample;
        for (int i = 0; i < RAW_DUMP_LINES_PER_LOOP && receiver_pop_raw_sample(&sample); i++) {
            printf("R,%llu,%u,%u\n", sample.timestamp_us, sample.rssi, sample.link_quality);
        }
    #endif

    if (window_start_us == 0) {
        window_start_us = now;
    }

    #ifdef RSSI_LINK_DIAGNOSTICS
        if (now - window_start_us >= LINK_DIAGNOSTICS_INTERVAL_US) {
            float secs = (now - window_start_us) / 1000000.0f;
            receiver_link_info_t info = receiver_take_link_info();
            uint32_t link_frames = info.link_stats_frames - last_link_frames;

            printf("LINK %.1fs | link_stats %.1f Hz", secs, link_frames / secs);
            if (link_frames > 1) {
                printf(" (gap min %lu max %lu us)", (unsigned long)info.min_gap_us, (unsigned long)info.max_gap_us);
            }
            printf(" | rc %.1f Hz", (info.rc_frames - last_rc_frames) / secs);
            if (link_frames > 0) {
                printf(" | rssi -%u dBm (window -%u..-%u) lq %u%% snr %d",
                    info.last.rssi, info.rssi_min, info.rssi_max, info.last.link_quality, info.last.snr);
            }
            printf(" | loop %.0f Hz max %lu us", loops / secs, (unsigned long)max_loop_us);
            printf(" | rpm accel %lu rssi %lu conf %.2f",
                (unsigned long)bot_state->rpm, (unsigned long)bot_state->rssi_rpm, bot_state->rssi_rpm_confidence);
            #ifdef RSSI_RAW_DUMP
                printf(" | raw dropped %lu", (unsigned long)receiver_raw_samples_overwritten());
            #endif
            printf("\n");

            last_rc_frames = info.rc_frames;
            last_link_frames = info.link_stats_frames;
            window_start_us = now;
            loops = 0;
            max_loop_us = 0;
            now = time_us_64(); // don't count our own printing as loop time
        }
    #endif

    last_call_us = now;
}

// this function and send_telemetry work together to switch between different sets of telemetry to send to transmitter 
uint8_t get_telemetry_state() {
    if (receiver_get_channel(SWITCH_F) >= RECEIVER_HIGHEST_CHANNEL_VALUE){
        return TELEMETRY_MOTOR1;
    } else if (receiver_get_channel(SWITCH_F) <= RECEIVER_LOWEST_CHANNEL_VALUE){
        return TELEMETRY_MAIN;
    } else {
        return TELEMETRY_MOTOR2;
    }
}

void telemetry_send_telemetry(bot_state_t* bot_state) {
    switch(get_telemetry_state()){
        case TELEMETRY_MOTOR1:

            #ifdef LIE_ABOUT_RPM
                int areWeLyingAboutRPM = 1*10; // the *10 is so it shows up as 1.0 on the transmitter
            #else
                int areWeLyingAboutRPM = 0;
            #endif

            receiver_send_telemetry(areWeLyingAboutRPM,bot_state->accel_g_value,bot_state->rpm,1);
            break;
        case TELEMETRY_MOTOR2:
            // multiplying by 10 to get the numbers to show up as 1.0, 2.0, etc. on the transmitter
            receiver_send_telemetry(donut_get_curr_drive_mode()*10,donut_is_killswitch_active()*10,bot_state->max_rpm,2);
            break;
        case TELEMETRY_MAIN:
            receiver_send_telemetry(bot_state->is_failsafed*10,(bot_state->accel_offset_cm < 0)*10,(uint32_t) fabs(bot_state->accel_offset_cm*10000),3);
            break;
    }
}
