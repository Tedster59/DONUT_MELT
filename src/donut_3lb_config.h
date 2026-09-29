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

// Some notes about RPM_MULTIPLIER LIMITS and how they works for the 3lb:
// left_stick_x "perma" adjusts the rpm say the accelerometer reading is 75% 
// of what it should be you could bump it up to use 1.25 times what it is reading 

// RPM_MULTIPLIER LIMITS limit how far that adjustment can go 
// (i.e. can pretend to use a max of 2x the reading, or a min of .25x the reading, etc.) 

#define RPM_MULTIPLIER_UPPER_LIMIT 1.1
#define RPM_MULTIPLIER_LOWER_LIMIT 0.25

#define ACCEL_MOUNT_RADIUS_CM 0 // this being 0 prevents accel_offset from going negative in 3lb mode

// this value will need to be tuned to a good speed - Cai
#define HEADING_CONTROL_SENSITIVITY 0.039 // tunes how fast the heading led moves left or right


// V2.0 board Teddy has: 1x = -5.25, 1y = -4.04, 2x = -5.27, 2y = -4.01

// V3.1.1 board 1: 1x = -3.61, 1y = -4.33, 2x = -3.59, 2y = -4.31
// V3.1.1 board 2: 1x = -2.11, 1y = -1.12, 2x = -2.08, 2y = -1.21
// V3.1.1 board 3: 1x = -1.78, 1y = -0.88, 2x = -1.80, 2y = -0.87
// V3.1.1 board 4: 1x = -2.73, 1y = -2.61, 2x = -2.73, 2y = -2.61

#define ACCEL_1_X_OFFSET -2.11
#define ACCEL_1_Y_OFFSET -1.12

#define ACCEL_2_X_OFFSET -2.08
#define ACCEL_2_Y_OFFSET -1.21


#define THROTTLE_PC_P 0.8 // was 0.5 for working translation test with 1lb on 4/29/26 - Cai

// CAN_ADJUST_ACCEL_MOUNT_RADIUS is defined here so you can use the 
// left_stick_x to adjust the get_rpm_2accel RPM reading by a constant factor
#define CAN_ADJUST_ACCEL_MOUNT_RADIUS
#define ACCEL_OFFSET_SENSITIVITY 0.0001

#define MOTOR_ON_PERCENT_DURATION 0.5 // This might technically be a half of a half - Cai (I still have no idea what this comment means - also Cai)
#define MIN_TRANSLATION_RPM 400

#define LED_OFFSET_PERCENT 0.45
#define MIN_LED_PERCENT_DURATION 0.25
#define MAX_LED_PERCENT_DURATION 0.5

// MELTY_MAX_TRANSLATION_AGGRESSION is a number from 0..1 which represents how different 
// the throttle sent to each motor can be during a half-rotation
// 1 = very different
// 0 = can't differ at all
#define MELTY_MAX_TRANSLATION_AGGRESSION 0.6 
#define MELTY_MAX_THROTTLE 1.0

#define TANK_DRIVE_MAX_THROTTLE 0.125
#define TANK_DRIVE_TURNING_MAX_THROTTLE 0.04


//----------END DONUT DRIVE SETTINGS---------


#endif