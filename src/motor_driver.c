#include "motor_driver.h"
#include "hardware/sync.h"

static BidirDShotX1* MOTOR1;
static BidirDShotX1* MOTOR2;

volatile uint16_t MOTOR1_throttle;
volatile uint16_t MOTOR2_throttle;

static bot_state_t* _user_bot_state;

static struct repeating_timer set_throttle_timer;

// Bidirectional DShot telemetry, written only from the timer interrupt below. Every field is
// 32 bits so each one is read atomically; motor_get_telemetry copies them with interrupts off
// so the snapshot is consistent too.
typedef struct {
    uint32_t erpm;
    uint32_t last_erpm_us;   // time_us_32() of the last valid eRPM packet (0 = none yet)
    uint32_t erpm_packets;
    uint32_t checksum_errors;
    uint32_t missing;        // frames the ESC didn't answer
    uint32_t other_packets;  // extended telemetry (temperature, voltage, ...) when enabled on the ESC
} motor_telemetry_state_t;

static volatile motor_telemetry_state_t _telemetry[2];

// The ESC answers each frame ~30us after it ends, and the answer has finished arriving well
// within the 200us frame interval, so reading right before the next send picks up exactly the
// answer to the previous frame.
static void read_telemetry(BidirDShotX1* motor, volatile motor_telemetry_state_t* t) {
    uint32_t value;
    switch (BidirDShotX1_getTelemetryPacket(motor, &value)) {
        case BIDIR_ERPM:
            t->erpm = value;
            t->last_erpm_us = time_us_32();
            t->erpm_packets++;
            break;
        case BIDIR_NO_PACKET:
            t->missing++;
            break;
        case BIDIR_CHECKSUM_ERROR:
            t->checksum_errors++;
            break;
        default:
            t->other_packets++;
            break;
    }
}

bool set_throttle_callback(struct repeating_timer *t) {
    read_telemetry(MOTOR1, &_telemetry[0]);
    read_telemetry(MOTOR2, &_telemetry[1]);

    BidirDShotX1_sendThrottle(MOTOR1, MOTOR1_throttle);
    BidirDShotX1_sendThrottle(MOTOR2, MOTOR2_throttle);
    // led_toggle();
    return true;
}

bool motor_get_telemetry(uint8_t motor, motor_telemetry_t* out) {
    if (motor < 1 || motor > 2) {
        return false;
    }
    uint32_t irq = save_and_disable_interrupts();
    const volatile motor_telemetry_state_t* t = &_telemetry[motor - 1];
    uint32_t erpm = t->erpm;
    uint32_t last_erpm_us = t->last_erpm_us;
    out->erpm_packets = t->erpm_packets;
    out->checksum_errors = t->checksum_errors;
    out->missing = t->missing;
    out->other_packets = t->other_packets;
    restore_interrupts(irq);

    out->erpm = erpm;
    out->erpm_age_us = (last_erpm_us == 0) ? UINT32_MAX : time_us_32() - last_erpm_us;
    out->rpm = erpm / MOTOR_POLE_PAIRS;
    return last_erpm_us != 0;
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
    if (BidirDShotX1_initError(MOTOR1) || BidirDShotX1_initError(MOTOR2)) {
        printf("MOTOR DShot init failed (motor1 %d, motor2 %d)\n",
            BidirDShotX1_initError(MOTOR1), BidirDShotX1_initError(MOTOR2));
    }

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
