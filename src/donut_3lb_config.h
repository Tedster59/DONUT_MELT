#ifndef DONUT_3LB_CONFIG
#define DONUT_3LB_CONFIG

//---------------LED SETTINGS---------------

// Onboard Pico2 led is on pin 25
// For 3lb:
//  Our usual top AND bottom led pin is 20
#define HEADING_LIGHT_STRIP_PIN	25

// 3lb
#define HEADING_LIGHT_STRIP_PIN2 20

//---------------END LED SETTINGS---------------


//----------DONUT DRIVE SETTINGS---------

// The accelerometer RPM correction (left_stick_x trim + saved curve points on CH10) and its
// limits now live under "RPM CORRECTION CURVE SETTINGS" in donut_config.h (donut_rpm_curve.c).

#define ACCEL_MOUNT_RADIUS_CM 0 // this being 0 prevents accel_offset from going negative in 3lb mode

// this value will need to be tuned to a good speed - Cai
#define HEADING_CONTROL_SENSITIVITY 0.039 // tunes how fast the heading led moves left or right


// V2.0 board Teddy has: 1x = -4.99, 1y = -4.14, 2x = -5.02, 2y = -4.18

// V3.1.1 board 1: 1x = -3.61, 1y = -4.33, 2x = -3.59, 2y = -4.31
// V3.1.1 board 2: 1x = -2.11, 1y = -1.12, 2x = -2.08, 2y = -1.21
// V3.1.1 board 3: 1x = -1.78, 1y = -0.88, 2x = -1.80, 2y = -0.87
// V3.1.1 board 4: 1x = -2.73, 1y = -2.61, 2x = -2.73, 2y = -2.61

#define ACCEL_1_X_OFFSET -2.11
#define ACCEL_1_Y_OFFSET -1.12

#define ACCEL_2_X_OFFSET -2.08
#define ACCEL_2_Y_OFFSET -1.21


#define THROTTLE_PC_P 0.8 // was 0.5 for working translation test with 1lb on 4/29/26 - Cai

// CAN_ADJUST_ACCEL_MOUNT_RADIUS is defined here so you can use the left_stick_x to trim the
// get_rpm_2accel RPM correction factor (rate: RPM_TRIM_RATE_PER_SEC in donut_config.h)
#define CAN_ADJUST_ACCEL_MOUNT_RADIUS

#define MOTOR_ON_PERCENT_DURATION 0.5 // This might technically be a half of a half - Cai (I still have no idea what this comment means - also Cai)
#define MIN_TRANSLATION_RPM 400

// 0.25 for 1lb donut with 90 degree LED offset, 0.45 or so for 3lb donut with 45 deg LED.
#define LED_OFFSET_PERCENT 0.25
#define MIN_LED_PERCENT_DURATION 0.25
#define MAX_LED_PERCENT_DURATION 0.5

// MELTY_MAX_TRANSLATION_AGGRESSION is a number from 0..1 which represents how different 
// the throttle sent to each motor can be during a half-rotation
// 1 = very different
// 0 = can't differ at all
#define MELTY_MAX_TRANSLATION_AGGRESSION 0.8 
#define MELTY_MAX_THROTTLE 1.0

#define TANK_DRIVE_MAX_THROTTLE 0.125
#define TANK_DRIVE_TURNING_MAX_THROTTLE 0.04


//----------END DONUT DRIVE SETTINGS---------


#endif