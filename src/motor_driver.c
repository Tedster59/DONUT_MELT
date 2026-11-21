#include "motor_driver.h"
#include "hardware/sync.h"

static BidirDShotX1* MOTOR1;
static BidirDShotX1* MOTOR2;

volatile uint16_t MOTOR1_throttle;
volatile uint16_t MOTOR2_throttle;

static bot_state_t* _user_bot_state;

static struct repeating_timer set_throttle_timer;

static uint64_t _motors_init_us = 0;
static bool _outputs_running = false;

// DShot frame period. POSITIVE so the next frame is scheduled from when this one was sent: with a
// negative period the SDK schedules from the previous target time, so after any stretch with
// interrupts off (a 45-400 ms flash sector erase) it fired back to back to catch up, ~225 calls
// after one erase. Each call restarts the PIO mid-frame (sendRaw12Bit), so the ESCs got bursts of
// chopped, spliced frames; with DShot's 4-bit CRC about 1 in 16 of those passes as a random
// throttle, which the ESC then holds through the next erase. That is the most likely cause of the
// motors running at full power for several seconds after a kill (during the log save).
#define DSHOT_FRAME_INTERVAL_US 200

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

    add_repeating_timer_us(DSHOT_FRAME_INTERVAL_US, set_throttle_callback, NULL, &set_throttle_timer);
    _outputs_running = true;

    // Only zero throttle is ever sent at boot, and boot never blocks. DShot ESCs arm on zero-throttle
    // frames, which the timer above sends from now on; donut_is_live() keeps the bot down until
    // MOTOR_ESC_ARM_HOLD_MS after this, so the ESCs have time to arm before any throttle is allowed.
    //
    // This used to send 0 - 1000 - 0 with sleep_ms() between (5 s). The DShot library turns 1000
    // into 1047: FULL REVERSE in 3D mode. It was only skipped after a watchdog reboot, so any other
    // reset with the ESCs still powered (brownout, supply glitch, RUN pin, a harness connector
    // bouncing) drove both motors at full reverse for 1 s, with the main loop asleep for 5 s and
    // the killswitch unread. A reset that the full-reverse current itself causes repeats it.
    _motors_init_us = time_us_64();
}

void motor_outputs_suspend(void) {
    if (!_outputs_running) {
        return;
    }
    cancel_repeating_timer(&set_throttle_timer);
    _outputs_running = false;
    motor_stop_all();
}

void motor_outputs_resume(void) {
    if (_outputs_running) {
        return;
    }
    motor_stop_all();
    add_repeating_timer_us(DSHOT_FRAME_INTERVAL_US, set_throttle_callback, NULL, &set_throttle_timer);
    _outputs_running = true;
    _motors_init_us = time_us_64(); // the ESCs saw the signal stop: give them the arm hold again
}

bool motor_outputs_suspended(void) {
    return !_outputs_running;
}

bool motor_esc_arm_hold_active(void) {
    return time_us_64() - _motors_init_us < (uint64_t)MOTOR_ESC_ARM_HOLD_MS * 1000;
}
