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

#define RPM_MULTIPLIER_UPPER_LIMIT 2
#define RPM_MULTIPLIER_LOWER_LIMIT 0.25

#define ACCEL_MOUNT_RADIUS_CM 0 // this being 0 prevents accel_offset from going negative in 3lb mode

// this value will need to be tuned to a good speed - Cai
#define HEADING_CONTROL_SENSITIVITY 0.025 // tunes how fast the heading led moves left or right

#define ACCEL_1_X_OFFSET -7
#define ACCEL_1_Y_OFFSET -5

#define ACCEL_2_X_OFFSET -7
#define ACCEL_2_Y_OFFSET -5


#define THROTTLE_PC_P 1 // was 0.5 for working translation test with 1lb on 4/29/26 - Cai

// CAN_ADJUST_ACCEL_MOUNT_RADIUS is defined here so you can use the 
// left_stick_x to adjust the get_rpm_2accel RPM reading by a constant factor
#define CAN_ADJUST_ACCEL_MOUNT_RADIUS
#define ACCEL_OFFSET_SENSITIVITY 0.0001

#define MOTOR_ON_PERCENT_DURATION 0.5 // This might technically be a half of a half - Cai (I still have no idea what this comment means - also Cai)
#define MIN_TRANSLATION_RPM 400

#define LED_OFFSET_PERCENT 0.25
#define MIN_LED_PERCENT_DURATION 0.25
#define MAX_LED_PERCENT_DURATION 0.5

// MELTY_MAX_TRANSLATION_AGGRESSION is a number from 0..1 which represents how different 
// the throttle sent to each motor can be during a half-rotation
// 1 = very different
// 0 = can't differ at all
#define MELTY_MAX_TRANSLATION_AGGRESSION 0.75  
#define MELTY_MAX_THROTTLE 1

#define TANK_DRIVE_MAX_THROTTLE 0.125
#define TANK_DRIVE_TURNING_MAX_THROTTLE 0.0625


//----------END DONUT DRIVE SETTINGS---------


#endif