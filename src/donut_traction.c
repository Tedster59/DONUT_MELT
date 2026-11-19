#include "donut_traction.h"

#ifdef TRACTION_CONTROL

static bool enabled = TC_ENABLED_AT_BOOT;
static uint8_t state = TC_STATE_OFF;

static uint64_t last_call_us = 0;     // 0 = reset, next call starts a new spin-up
static uint64_t launch_start_us = 0;
static uint64_t last_accel_us = 0;
static uint64_t last_log_us = 0;

static double accel_rpm = 0;          // filtered, curve-corrected accelerometer rpm
static double wheel_motor_rpm = 0;    // filtered max motor rpm
static double ceiling_int = 1.0;      // integrated throttle cap
static double ceiling = 1.0;          // cap actually applied (integrator minus proportional term)
static double reference_rpm = 0;
static double slip = 0;
static bool slip_valid = false;

static bool button_armed = false;
static bool button_held = false;

// first-order low pass with time constant TC_FILTER_TAU_US; the same filter on the wheel and bot
// rpm keeps their delays matched, so a spin-up doesn't look like slip just because one lags
static double low_pass(double prev, double x, uint64_t dt_us) {
    double k = (double)dt_us / (TC_FILTER_TAU_US + (double)dt_us);
    return prev + k * (x - prev);
}

static double clamp(double x, double lo, double hi) {
    return fmin(fmax(x, lo), hi);
}

void traction_reset(void) {
    last_call_us = 0;
    ceiling_int = 1.0;
    ceiling = 1.0;
    slip_valid = false;
    if (state == TC_STATE_LIMITING) {
        state = TC_STATE_WATCHING;
    }
}

bool traction_controls_throttle(void) {
    return state == TC_STATE_WATCHING || state == TC_STATE_LIMITING;
}

uint8_t traction_state(void) {
    return state;
}

static uint16_t clamp_u16(double x) {
    return (uint16_t)clamp(x, 0, UINT16_MAX);
}

static void log_tick(uint64_t now, const motor_telemetry_t* m1, const motor_telemetry_t* m2) {
    #ifdef USE_FLASH_LOGGER
        if (now - last_log_us < TC_LOG_INTERVAL_US) {
            return;
        }
        last_log_us = now;
        logger_log_record('M', 1, clamp_u16(m1->rpm));
        logger_log_record('M', 2, clamp_u16(m2->rpm));
        logger_log_record('T', state, clamp_u16(accel_rpm));
        logger_log_record('U', (uint8_t)clamp(ceiling * 100.0 + 0.5, 0, 255), clamp_u16(reference_rpm));
        if (slip_valid) {
            // slip = a/100 - 1, so -1.00..+1.55 fits in a byte
            logger_log_record('V', (uint8_t)clamp((slip + 1.0) * 100.0 + 0.5, 0, 255),
                clamp_u16(wheel_motor_rpm / TC_MOTOR_RPM_PER_BOT_RPM));
        }
    #endif
}

double traction_limit_throttle(double throttle) {
    uint64_t now = time_us_64();
    bool fresh_start = (last_call_us == 0);
    uint64_t dt_us = fresh_start ? 0 : now - last_call_us;
    if (dt_us > 10000) {
        dt_us = 10000; // a long stall (flash save etc.) shouldn't make one giant step
    }
    last_call_us = now;

    if (fresh_start) {
        launch_start_us = now;
        last_accel_us = 0;
        accel_rpm = 0;
        wheel_motor_rpm = 0;
        ceiling_int = 1.0;
        ceiling = 1.0;
    }

    // ---- measurements (also gathered while off/uncalibrated, so the log can calibrate) ----

    motor_telemetry_t m1, m2;
    bool have1 = motor_get_telemetry(1, &m1);
    bool have2 = motor_get_telemetry(2, &m2);
    bool telemetry_ok = have1 && have2 &&
        m1.erpm_age_us <= TC_MAX_ERPM_AGE_US && m2.erpm_age_us <= TC_MAX_ERPM_AGE_US;
    if (telemetry_ok) {
        double motor_rpm = fmax(m1.rpm, m2.rpm); // the faster wheel is the one slipping
        wheel_motor_rpm = fresh_start ? motor_rpm : low_pass(wheel_motor_rpm, motor_rpm, dt_us);
    }

    if (last_accel_us == 0 || now - last_accel_us >= TC_ACCEL_INTERVAL_US) {
        uint64_t accel_dt = last_accel_us == 0 ? 0 : now - last_accel_us;
        last_accel_us = now;
        double raw = accel_read_raw_rpm_2accel();
        if (raw >= 0 && isfinite(raw)) {
            double corrected = raw * rpm_curve_factor(raw);
            accel_rpm = accel_dt == 0 ? corrected : low_pass(accel_rpm, corrected, accel_dt);
        }
    }

    // below TC_ACCEL_TRUST_RPM the accelerometer reading is mostly offset error and noise, which
    // would let the wheels slip at launch, so only the launch ramp counts there. Above it the
    // accelerometer alone is used: a ramp running ahead of the real bot would allow slip.
    double launch_rpm = fmin(TC_LAUNCH_RPM_PER_S * ((now - launch_start_us) / 1000000.0), TC_LAUNCH_RAMP_MAX_RPM);
    reference_rpm = (accel_rpm >= TC_ACCEL_TRUST_RPM) ? accel_rpm : launch_rpm;

    // ---- state ----

    if (!enabled) {
        state = TC_STATE_OFF;
    } else if (!(TC_MOTOR_RPM_PER_BOT_RPM > 0)) {
        state = TC_STATE_UNCALIBRATED;
    } else if (!telemetry_ok) {
        state = TC_STATE_NO_TELEMETRY;
    } else if (state < TC_STATE_WATCHING) {
        state = TC_STATE_WATCHING;
    }

    slip_valid = telemetry_ok && TC_MOTOR_RPM_PER_BOT_RPM > 0;
    if (slip_valid) {
        double wheel_rpm = wheel_motor_rpm / TC_MOTOR_RPM_PER_BOT_RPM;
        slip = (wheel_rpm - reference_rpm) / fmax(reference_rpm, TC_MIN_REF_RPM);
    }

    if (!traction_controls_throttle()) {
        ceiling_int = 1.0;
        ceiling = 1.0;
        log_tick(now, &m1, &m2);
        return throttle;
    }

    // ---- controller: a throttle cap that drops fast on slip and recovers slowly ----

    double magnitude = fabs(throttle);
    double dt = dt_us / 1000000.0;
    double error = slip - TC_TARGET_SLIP;
    if (error > 0) {
        // start cutting from the throttle actually applied, not from a cap far above it
        ceiling_int = fmin(ceiling_int, magnitude);
        ceiling_int -= TC_CUT_GAIN * error * dt;
    } else {
        ceiling_int += TC_RECOVER_PER_S * dt;
    }
    ceiling_int = clamp(ceiling_int, TC_MIN_CEILING, 1.0);
    ceiling = clamp(ceiling_int - TC_KP * fmax(error, 0), TC_MIN_CEILING, 1.0);

    state = (magnitude > ceiling) ? TC_STATE_LIMITING : TC_STATE_WATCHING;
    log_tick(now, &m1, &m2);

    if (magnitude <= ceiling) {
        return throttle;
    }
    return throttle > 0 ? ceiling : -ceiling;
}

void traction_update_switch(bot_state_t* bot_state) {
    // same pattern as the CH10 curve button: only trust real channel data, and only accept a press
    // after the button has been seen released
    uint16_t ch = receiver_get_channel(MOMENTARY_CH9);
    if (bot_state->is_failsafed) {
        button_armed = false;
        button_held = false;
    } else if (!button_armed) {
        button_armed = ch < RECEIVER_MIDDLEST_CHANNEL_VALUE + 200;
    } else if (!button_held && ch > RECEIVER_MIDDLEST_CHANNEL_VALUE + 400) {
        button_held = true;
        enabled = !enabled;
        // when switched on, start as "not controlling" (old ramp) until the next
        // traction_limit_throttle call has checked calibration and telemetry
        state = enabled ? TC_STATE_NO_TELEMETRY : TC_STATE_OFF;
        printf("TRACTION control %s\n", enabled ? "ON" : "OFF");
    } else if (button_held && ch < RECEIVER_MIDDLEST_CHANNEL_VALUE + 200) {
        button_held = false;
    }
}

#else // TRACTION_CONTROL not defined: everything passes through

double traction_limit_throttle(double throttle) { return throttle; }
void traction_reset(void) {}
bool traction_controls_throttle(void) { return false; }
void traction_update_switch(bot_state_t* bot_state) { (void)bot_state; }
uint8_t traction_state(void) { return TC_STATE_OFF; }

#endif
