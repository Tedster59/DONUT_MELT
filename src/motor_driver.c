#include "motor_driver.h"

static BidirDShotX1* MOTOR1;
static BidirDShotX1* MOTOR2;

volatile uint16_t MOTOR1_throttle;
volatile uint16_t MOTOR2_throttle;

static bot_state_t* _user_bot_state;

// In motor_driver.c
volatile motor_debug_snapshot_t motor_debug_snapshot;

static struct repeating_timer set_throttle_timer;

bool set_throttle_callback(struct repeating_timer *t){
    BidirDShotX1_sendThrottle(MOTOR1, MOTOR1_throttle);
    BidirDShotX1_sendThrottle(MOTOR2, MOTOR2_throttle);

    // lightweight snapshot — no printf, just a struct copy
    motor_debug_snapshot.motor1_throttle = MOTOR1_throttle;
    motor_debug_snapshot.motor2_throttle = MOTOR2_throttle;
    motor_debug_snapshot.send_count++;

    return true;
}

// throttle range is 0-2000, 0 in 3D mode (forward-backward) is stopped
void motor_motor1_set_throttle(uint16_t throttle){
    MOTOR1_throttle = throttle;
}

void motor_motor2_set_throttle(uint16_t throttle){
    MOTOR2_throttle = throttle;
}

void motor_set_throttle_for_all(uint16_t throttle){
    motor_motor1_set_throttle(throttle);
    motor_motor2_set_throttle(throttle);
}

void motor_stop_all(){
    motor_set_throttle_for_all(0);
}

void motor_init_all(int dshot_speed, int motor1_pin, PIO motor1_pio, int motor2_pin, PIO motor2_pio, bot_state_t* user_bot_state){
    MOTOR1 = BidirDShotX1_constructor(motor1_pin, dshot_speed, motor1_pio, -1);
    MOTOR2 = BidirDShotX1_constructor(motor2_pin, dshot_speed, motor2_pio, -1);

    _user_bot_state = user_bot_state;

    motor_stop_all();

    // negative time means that the callback is called at a consistent rate regardless of time taken during callback call
    add_repeating_timer_us(-200, set_throttle_callback, NULL, &set_throttle_timer);

    // arm escs by sending 0 throttle for at least 3 seconds
    sleep_ms(3000);
}

void motor_shutdown_all(){
    cancel_repeating_timer(&set_throttle_timer);
}
