#include "motor_driver.h"

static BidirDShotX1* MOTOR1;
static BidirDShotX1* MOTOR2;

volatile uint16_t MOTOR1_throttle;
volatile uint16_t MOTOR2_throttle;

static bot_state_t* _user_bot_state;

static struct repeating_timer set_throttle_timer;

bool set_throttle_callback(struct repeating_timer *t) {
    BidirDShotX1_sendThrottle(MOTOR1, MOTOR1_throttle);
    BidirDShotX1_sendThrottle(MOTOR2, MOTOR2_throttle);
    // led_toggle();
    return true;
}

// throttle range is 0-2000, 0 in 3D mode (forward-backward) is stopped
void motor_motor1_set_throttle(uint16_t throttle) {
    MOTOR1_throttle = throttle;
}

void motor_motor2_set_throttle(uint16_t throttle) {
    MOTOR2_throttle = throttle;
}

void motor_set_throttle_for_all(uint16_t throttle) {
    motor_motor1_set_throttle(throttle);
    motor_motor2_set_throttle(throttle);
}

void motor_stop_all() {
    motor_set_throttle_for_all(0);
}

void motor_init_all(int dshot_speed, int motor1_pin, PIO motor1_pio, int motor2_pin, PIO motor2_pio, bot_state_t* user_bot_state) {
    MOTOR1 = BidirDShotX1_constructor(motor1_pin, dshot_speed, motor1_pio, -1);
    MOTOR2 = BidirDShotX1_constructor(motor2_pin, dshot_speed, motor2_pio, -1);

    _user_bot_state = user_bot_state;

    motor_stop_all();

    // negative time means that the callback is called at a consistent rate regardless of time taken during callback call
    add_repeating_timer_us(-200, set_throttle_callback, NULL, &set_throttle_timer);

    // After a watchdog-type reboot (our watchdog timing out mid-run, or the bootrom reboot after a
    // UF2 flash), the ESCs may already be powered and armed. The init sequence below sends 1000,
    // which the DShot library turns into 1047: FULL REVERSE in 3D mode. An armed ESC would drive
    // that for a full second, so skip the sequence and hold zero throttle (already being sent by
    // the timer above). Armed ESCs keep running on zero; ESCs that dropped out re-arm on zero.
    if (watchdog_caused_reboot()) {
        return;
    }

    // Cold power-on only: initing escs by going 0 - 1000 - 0 for some reason, it shouldn't work
    // like this but it does. Likely harmless here only because the ESCs are still booting and
    // ignore the 1000 (full reverse) -- see the watchdog check above.
    motor_set_throttle_for_all(0);
    sleep_ms(1000);

    motor_set_throttle_for_all(1000);
    sleep_ms(1000);

    motor_set_throttle_for_all(0);
    sleep_ms(3000);
}
