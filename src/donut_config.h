#ifndef DONUT_CONFIG_H
#define DONUT_CONFIG_H

#include "donut_dependencies.h"

void donut_init_bot_state();

uint8_t donut_is_throttle_zero();

uint8_t donut_is_killswitch_active();

uint8_t donut_get_curr_drive_mode();

//----------BOT TYPE----------

//#include "donut_1lb_config.h"
#include "donut_3lb_config.h"

//----------END BOT TYPE----------


//----------DEBUG SETTINGS----------

// #define RUNNING_A_TEST

//#define FAKE_RPM 60 // the fake rpm to be used in get_fake_rpm (in some capacity)

// #define LIE_ABOUT_RPM // fakes RPM data
// #define LIE_ABOUT_INPUT // fakes receiver data
// #define NO_MOTOR_SPINNING // prevents motors from spinning up if defined
#define BYPASS_MIN_TRANSLATION_RPM // lets us attempt to translate under MIN_TRANSLATION_RPM if defined

//----------END DEBUG----------


//---------------LED SETTINGS---------------

#define SLOW_BLINK 500
#define FAST_BLINK 50
#define REPEAT_BLINK 250

//---------------END LED SETTINGS---------------


//----------ACCELEROMETER SETTINGS----------

#define ACCEL_1_I2C_PORT i2c0
#define ACCEL_1_I2C_SDA 4
#define ACCEL_1_I2C_SCL 5

#define ACCEL_2_I2C_PORT i2c1
#define ACCEL_2_I2C_SDA 10
#define ACCEL_2_I2C_SCL 11

//----------END ACCELEROMETER SETTINGS----------


//----------REICEVER SETTINGS----------

#define RECEIVER_UART_ID uart1
#define RECEIVER_UART_TX_PIN 8
#define RECEIVER_UART_RX_PIN 9

#define RECEIVER_LOWEST_CHANNEL_VALUE 222 // actual lowest is 172 for knobs and 212 for joysticks
#define RECEIVER_HIGHEST_CHANNEL_VALUE 1780 // actual highest is 1790
#define RECEIVER_MIDDLEST_CHANNEL_VALUE 1001 // (222 + 1780)/2 = 1001

//----------END REICEVER SETTINGS----------


//----------MOTOR SETTINGS----------

// ESC pins are connected as follows:
// MOTOR1_PIN was 2, MOTOR2_PIN was 3

// #define FLASHING_MOTORS 

#define MOTOR1_PIN 2 
#define MOTOR2_PIN 3

#define MOTOR1_PIO pio0
#define MOTOR2_PIO pio1

#define DSHOT_SPEED 600 // could be 150, 300, 450, 600 + some more

#define DRIVE_MODE_MELTY 1
#define DRIVE_MODE_TANK 2

//----------END MOTOR SETTINGS----------
                  

//----------TELEMETRY SETTINGS----------

#define SHOULD_SEND_TELEMETRY_TO_TRANSMITTER

// #define OUTPUT_DIAGNOSTICS

// #define TIME_SINCE_BOOT_DIAGNOSTICS
#define ACCEL_DIAGNOSTICS
// #define FULL_CONTROLLER_DIAGNOSTICS
// #define OTHER_DIAGNOSTICS

#define TELEMETRY_MOTOR1 0
#define TELEMETRY_MOTOR2 1
#define TELEMETRY_MAIN 2

//----------END TELEMETRY SETTINGS----------


//----------MISC----------

#define WATCH_DOG_TIMEOUT_MS 500 // changed to prevent 200 microsecond delay from having any chance of triggering watchdog

//----------END MISC----------


#endif