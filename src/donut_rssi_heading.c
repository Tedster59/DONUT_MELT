#include "donut_rssi_heading.h"

#ifdef RSSI_HEADING

#define H RSSI_HEADING_HARMONICS
#define SAMPLE_BUF 256     // the window (~200 samples at 1 kHz) plus what arrives while it's processed
#define CHUNK 48           // samples processed per main loop call (keeps each call well under 100 us)
#define SEARCH_STEPS 72    // phase error candidates per update (5 deg apart, then refined)
#define UPDATE_S (RSSI_HEADING_UPDATE_US / 1000000.0)
#define TWO_PI_F 6.28318530718f

_Static_assert(RSSI_HEADING_WINDOW_US / 1000 + 40 <= SAMPLE_BUF, "RSSI_HEADING_WINDOW_US too long for SAMPLE_BUF");

// ---------- raw RSSI samples ----------
static uint32_t s_t[SAMPLE_BUF]; // arrival time (low 32 bits of time_us_64) minus RSSI_HEADING_LATENCY_US
static uint8_t s_v[SAMPLE_BUF];
static uint16_t s_head = 0;
static uint16_t s_count = 0;

// ---------- rotation model: phase(t) = phase_ref + freq_hz * (t - t_ref) ----------
static bot_state_t* _user_bot_state;
static uint8_t state = RSSI_HEADING_IDLE;
static double phase_ref = 0;     // turns, kept in [0, 1)
static uint64_t t_ref_us = 0;
static double freq_hz = 0;
static double accel_scale = 1;   // true rpm / corrected accel rpm, learned from the RSSI phase

// pattern template (harmonic phasors in the model frame, fundamental pinned to phase 0)
static float tmpl_re[H], tmpl_im[H];
static bool tmpl_valid = false;
static int8_t tmpl_dir = -1;

// heading offset ("forward" relative to the pattern frame) per spin direction, turns
static double offset[2];
static bool offset_valid[2] = { false, false };

static uint16_t good_run = 0;    // consecutive accepted updates
static uint64_t last_locked_us = 0; // last update that ended LOCKED (0 = not since the last restart)
static uint16_t bad_run = 0;     // consecutive rejected updates
static uint8_t rpm_diff_run = 0; // consecutive updates with the RSSI rpm > RSSI_HEADING_MAX_RPM_DIFF away
static uint64_t last_update_us = 0;
static uint64_t last_rssi_ok_us = 0;

// ---------- the window being processed (spread over several calls) ----------
typedef enum { HD_WAIT, HD_ACCUM, HD_FINISH } hd_step_t;
static hd_step_t step = HD_WAIT;
static uint32_t win_end;         // window end time (32-bit us)
static double win_phase_end;     // model phase at the window end
static double win_freq;
static uint16_t win_idx;         // next ring index to process
static uint16_t win_left;        // samples left to process
static uint16_t win_n;
static float acc_v;
static float acc_ve_re[H], acc_ve_im[H]; // sum of v * e^{-j 2 pi h phase}
static float acc_e_re[H], acc_e_im[H];   // sum of e^{-j 2 pi h phase} (to remove the window mean)

static double frac(double x) {
    return x - floor(x);
}

static int current_dir(void) {
    return donut_should_spin_clockwise() ? 1 : 0;
}

void rssi_heading_init(bot_state_t* user_bot_state) {
    _user_bot_state = user_bot_state;
    s_head = 0;
    s_count = 0;
    state = RSSI_HEADING_IDLE;
    step = HD_WAIT;
}

void rssi_heading_add_sample(uint64_t timestamp_us, uint8_t rssi) {
    s_t[s_head] = (uint32_t)(timestamp_us - RSSI_HEADING_LATENCY_US);
    s_v[s_head] = rssi;
    s_head = (s_head + 1) % SAMPLE_BUF;
    if (s_count < SAMPLE_BUF) {
        s_count++;
    }
}

// ACQUIRING within RSSI_HEADING_HOLD_US of the last LOCKED update: the model still has the angle,
// it's just going through a stretch of rejected updates (bouncing, tilting)
static bool holding(uint64_t now) {
    return state == RSSI_HEADING_ACQUIRING && last_locked_us != 0 &&
           now - last_locked_us < (uint64_t)RSSI_HEADING_HOLD_US;
}

uint8_t rssi_heading_state(void) {
    return holding(time_us_64()) ? RSSI_HEADING_HOLDING : state;
}

double rssi_heading_rpm(void) {
    return state == RSSI_HEADING_IDLE ? 0 : freq_hz * 60.0;
}

static void go_idle(void) {
    state = RSSI_HEADING_IDLE;
    good_run = 0;
    bad_run = 0;
    rpm_diff_run = 0;
    last_locked_us = 0; // the angle is gone: no holding
}

static void log_update(float quality, double err_turns) {
    #ifdef USE_FLASH_LOGGER
        double rpm10 = fmin(fmax(rssi_heading_rpm() * 10.0, 0), 65535);
        logger_log_record('H', rssi_heading_state(), (uint16_t)rpm10);
        double err10 = fmin(fmax(err_turns * 3600.0, -32767), 32767); // 0.1 deg
        logger_log_record('J', (uint8_t)fminf(fmaxf(quality * 100.0f + 0.5f, 0), 255), (uint16_t)(int16_t)err10 + 32768u);
    #else
        (void)quality;
        (void)err_turns;
    #endif
}

// starts a new update: checks there is something to track, advances the model to now and
// captures the window. Returns false when there is nothing to process this time.
static bool start_window(uint64_t now) {
    double rssi_rpm, slope;
    bool rssi_ok = rssi_rpm_current(&rssi_rpm, &slope) && rssi_rpm >= RSSI_RPM_MIN_SEED_RPM;
    double accel_rpm = _user_bot_state->accel_rpm_corrected;
    bool accel_ok = accel_rpm >= RSSI_RPM_MIN_SEED_RPM;

    if (rssi_ok) {
        last_rssi_ok_us = now;
    } else if (now - last_rssi_ok_us > 500000 &&
               (!accel_ok || !(state == RSSI_HEADING_LOCKED || holding(now)))) {
        // no RSSI rpm for a while: stopped, or the link is gone. While locked, the RSSI rpm is only a
        // cross-check: its confidence dips below threshold for ~0.3 s now and then while the phase
        // match is still fine (heading_drive2.csv, 107 s and 121 s: going idle there threw away a
        // good lock and re-learned the angle, a small heading jump each time). A dead link still
        // ends the lock: windows without enough samples count as rejected updates below.
        go_idle();
    }

    int dir = current_dir();
    if (dir != tmpl_dir) {
        // the pattern is traversed the other way round: relearn it
        tmpl_dir = dir;
        tmpl_valid = false;
        go_idle();
    }

    if (state == RSSI_HEADING_IDLE) {
        // start from the RSSI rpm, and only while the speed is steady enough to lock onto
        if (!rssi_ok || !accel_ok || fabs(slope) > RSSI_RPM_MAX_SLOPE_RPM_S) {
            return false;
        }
        freq_hz = rssi_rpm / 60.0;
        accel_scale = rssi_rpm / accel_rpm;
        phase_ref = 0;
        t_ref_us = now;
        state = RSSI_HEADING_ACQUIRING;
        good_run = 0;
        bad_run = 0;
    }

    // advance the model to now at the speed it had, then take the new speed from the accelerometer:
    // it follows speed changes within a rotation or two (translation costs ~100-200 rpm in a second,
    // which a phase loop alone fell behind on and lost the lock: heading_shadow.csv), and the RSSI
    // phase corrects its scale (accel_scale) and pins the angle
    phase_ref = frac(phase_ref + freq_hz * (double)(now - t_ref_us) / 1000000.0);
    t_ref_us = now;
    if (accel_ok) {
        freq_hz = accel_scale * accel_rpm / 60.0;
    }
    bool rpm_disagrees = rssi_ok && fabs(freq_hz * 60.0 - rssi_rpm) / rssi_rpm > RSSI_HEADING_MAX_RPM_DIFF;
    rpm_diff_run = rpm_disagrees ? rpm_diff_run + 1 : 0;
    if (rpm_diff_run >= RSSI_HEADING_RESEED_UPDATES) {
        // the speed wandered off (accel noise, or locked onto noise): take the speed and scale from
        // the RSSI rpm again but keep the angle, and earn the lock back. Only when the RSSI rpm has
        // disagreed for several updates in a row: a single RSSI estimate can be a harmonic alias
        // (heading_drive1.csv: one 1504 among ~2005s, exactly 3/4) and one update at the wrong speed
        // turns the heading 150 deg.
        rpm_diff_run = 0;
        freq_hz = rssi_rpm / 60.0;
        if (accel_ok) {
            accel_scale = rssi_rpm / accel_rpm;
        }
        state = RSSI_HEADING_ACQUIRING;
        good_run = 0;
    }

    // the samples in (now - window, now], oldest first
    uint32_t now32 = (uint32_t)now;
    uint16_t n = 0;
    while (n < s_count) {
        uint16_t i = (s_head + SAMPLE_BUF - 1 - n) % SAMPLE_BUF;
        int32_t age = (int32_t)(now32 - s_t[i]);
        if (age >= RSSI_HEADING_WINDOW_US) {
            break;
        }
        n++;
    }
    if (n < (RSSI_HEADING_WINDOW_US / 1000) * 6 / 10) {
        // link gap: not enough samples for a measurement. Counts as a rejected update, so a lock
        // can't outlive the link (it drops after RSSI_HEADING_LOSE_UPDATES, then the hold runs out)
        bad_run++;
        good_run = 0;
        if (state == RSSI_HEADING_LOCKED && bad_run >= RSSI_HEADING_LOSE_UPDATES) {
            state = RSSI_HEADING_ACQUIRING;
        }
        if (bad_run >= 4 * RSSI_HEADING_LOSE_UPDATES) {
            go_idle();
        }
        return false;
    }

    win_end = now32;
    win_phase_end = phase_ref;
    win_freq = freq_hz;
    win_idx = (s_head + SAMPLE_BUF - n) % SAMPLE_BUF;
    win_left = n;
    win_n = n;
    acc_v = 0;
    for (int h = 0; h < H; h++) {
        acc_ve_re[h] = acc_ve_im[h] = acc_e_re[h] = acc_e_im[h] = 0;
    }
    return true;
}

static void accumulate_chunk(void) {
    for (int k = 0; k < CHUNK && win_left > 0; k++, win_left--) {
        uint16_t i = win_idx;
        win_idx = (win_idx + 1) % SAMPLE_BUF;
        double dt = (double)(int32_t)(s_t[i] - win_end) / 1000000.0; // <= 0
        float ph = (float)frac(win_phase_end + win_freq * dt);
        float c1 = cosf(TWO_PI_F * ph);
        float s1 = -sinf(TWO_PI_F * ph); // e^{-j 2 pi phase}
        float v = (float)s_v[i];
        float cr = c1, ci = s1;
        for (int h = 0; h < H; h++) {
            acc_ve_re[h] += v * cr;
            acc_ve_im[h] += v * ci;
            acc_e_re[h] += cr;
            acc_e_im[h] += ci;
            float nr = cr * c1 - ci * s1; // next harmonic: multiply by e^{-j 2 pi phase} again
            ci = cr * s1 + ci * c1;
            cr = nr;
        }
        acc_v += v;
    }
}

static void finish_window(void) {
    // harmonic phasors with the window mean removed
    float mean = acc_v / win_n;
    float z_re[H], z_im[H];
    float z_norm2 = 0;
    for (int h = 0; h < H; h++) {
        z_re[h] = (acc_ve_re[h] - mean * acc_e_re[h]) / win_n;
        z_im[h] = (acc_ve_im[h] - mean * acc_e_im[h]) / win_n;
        z_norm2 += z_re[h] * z_re[h] + z_im[h] * z_im[h];
    }

    if (!tmpl_valid) {
        // first window in this direction: it becomes the template, rotated so the fundamental has phase 0
        float a1 = atan2f(z_im[0], z_re[0]);
        for (int h = 0; h < H; h++) {
            float r = -(h + 1) * a1;
            tmpl_re[h] = z_re[h] * cosf(r) - z_im[h] * sinf(r);
            tmpl_im[h] = z_re[h] * sinf(r) + z_im[h] * cosf(r);
        }
        tmpl_valid = true;
        log_update(0, 0);
        return;
    }

    // match: the shift d maximising S(d) = Re sum_h conj(C_h) z_h e^{j 2 pi h d}
    float w_re[H], w_im[H];
    float c_norm2 = 0;
    for (int h = 0; h < H; h++) {
        w_re[h] = tmpl_re[h] * z_re[h] + tmpl_im[h] * z_im[h];  // conj(C) * z
        w_im[h] = tmpl_re[h] * z_im[h] - tmpl_im[h] * z_re[h];
        c_norm2 += tmpl_re[h] * tmpl_re[h] + tmpl_im[h] * tmpl_im[h];
    }
    float S[SEARCH_STEPS];
    int best = 0;
    for (int k = 0; k < SEARCH_STEPS; k++) {
        float d = (float)k / SEARCH_STEPS - 0.5f;
        float sum = 0;
        for (int h = 0; h < H; h++) {
            float a = TWO_PI_F * (h + 1) * d;
            sum += w_re[h] * cosf(a) - w_im[h] * sinf(a);
        }
        S[k] = sum;
        if (sum > S[best]) {
            best = k;
        }
    }
    float a = S[(best + SEARCH_STEPS - 1) % SEARCH_STEPS];
    float b = S[best];
    float c = S[(best + 1) % SEARCH_STEPS];
    float den = a - 2.0f * b + c;
    float refine = den < 0 ? fmaxf(-0.5f, fminf(0.5f, 0.5f * (a - c) / den)) : 0;
    double shift = ((double)best + refine) / SEARCH_STEPS - 0.5; // rotation aligning z with the template
    double err = -shift;                                         // turns the model is behind the pattern
    float quality = b / (sqrtf(c_norm2 * z_norm2) + 1e-12f);

    bool rejected = quality < RSSI_HEADING_Q_MIN || fabs(err) > RSSI_HEADING_MAX_ERR_DEG / 360.0;
    if (!rejected) {
        phase_ref = frac(phase_ref + RSSI_HEADING_KP * err);
        // a steady phase error means the speed is off by err / UPDATE_S turns/s: fold that into
        // the accel scale (slowly, so accel noise isn't learned)
        if (freq_hz > 0) {
            accel_scale *= 1.0 + RSSI_HEADING_KS * err / UPDATE_S / freq_hz;
        }

        // learn the pattern: fold in the measurement aligned to the model, then re-pin the fundamental
        float alpha = (float)(RSSI_HEADING_UPDATE_US / (double)RSSI_HEADING_TEMPLATE_TAU_US);
        for (int h = 0; h < H; h++) {
            float r = TWO_PI_F * (h + 1) * (float)shift;
            float ar = z_re[h] * cosf(r) - z_im[h] * sinf(r);
            float ai = z_re[h] * sinf(r) + z_im[h] * cosf(r);
            tmpl_re[h] = (1 - alpha) * tmpl_re[h] + alpha * ar;
            tmpl_im[h] = (1 - alpha) * tmpl_im[h] + alpha * ai;
        }
        float a1 = atan2f(tmpl_im[0], tmpl_re[0]);
        for (int h = 0; h < H; h++) {
            float r = -(h + 1) * a1;
            float nr = tmpl_re[h] * cosf(r) - tmpl_im[h] * sinf(r);
            tmpl_im[h] = tmpl_re[h] * sinf(r) + tmpl_im[h] * cosf(r);
            tmpl_re[h] = nr;
        }
        good_run++;
        bad_run = 0;
    } else {
        bad_run++;
        good_run = 0;
    }

    if (state == RSSI_HEADING_ACQUIRING && good_run >= RSSI_HEADING_LOCK_UPDATES) {
        state = RSSI_HEADING_LOCKED;
    } else if (state == RSSI_HEADING_LOCKED && bad_run >= RSSI_HEADING_LOSE_UPDATES) {
        state = RSSI_HEADING_ACQUIRING;
    }
    if (bad_run >= 4 * RSSI_HEADING_LOSE_UPDATES) {
        go_idle();
    }
    if (state == RSSI_HEADING_LOCKED) {
        last_locked_us = time_us_64();
    }
    log_update(quality, err);
}

void rssi_heading_update(void) {
    switch (step) {
        case HD_WAIT: {
            uint64_t now = time_us_64();
            if (now - last_update_us < RSSI_HEADING_UPDATE_US) {
                return;
            }
            last_update_us = now;
            if (start_window(now)) {
                step = HD_ACCUM;
            }
            return;
        }
        case HD_ACCUM:
            accumulate_chunk();
            if (win_left == 0) {
                step = HD_FINISH;
            }
            return;
        case HD_FINISH:
            finish_window();
            step = HD_WAIT;
            return;
    }
}

bool rssi_heading_timing(uint64_t now_us, double right_x_percent, double rpm_timing_fraction,
                         double* us_per_rotation, double* elapsed_us) {
    static uint64_t last_call_us = 0;
    double dt = last_call_us == 0 ? 0 : fmin((double)(now_us - last_call_us) / 1000000.0, 0.1);
    last_call_us = now_us;

    #ifndef RSSI_HEADING_DRIVE
        (void)dt; (void)right_x_percent; (void)rpm_timing_fraction; (void)us_per_rotation; (void)elapsed_us;
        return false; // shadow mode: track and log only
    #else
        // locked, or holding through a short stretch of rejected updates
        if (!(state == RSSI_HEADING_LOCKED || holding(now_us)) || !(freq_hz > 0)) {
            return false;
        }
        int dir = current_dir();
        double phase = phase_ref + freq_hz * (double)(now_us - t_ref_us) / 1000000.0;
        if (!offset_valid[dir]) {
            // first lock in this direction: start where the rpm-based timing has the heading
            offset[dir] = frac(rpm_timing_fraction - phase);
            offset_valid[dir] = true;
        }
        // right stick X: same heading rate as the rpm-based trim (rpm * x * sensitivity)
        offset[dir] = frac(offset[dir] + freq_hz * right_x_percent * HEADING_CONTROL_SENSITIVITY * dt);
        *us_per_rotation = 1000000.0 / freq_hz;
        *elapsed_us = frac(phase + offset[dir]) * *us_per_rotation;
        return true;
    #endif
}

#else // RSSI_HEADING not defined

void rssi_heading_init(bot_state_t* user_bot_state) { (void)user_bot_state; }
void rssi_heading_add_sample(uint64_t timestamp_us, uint8_t rssi) { (void)timestamp_us; (void)rssi; }
void rssi_heading_update(void) {}
bool rssi_heading_timing(uint64_t now_us, double right_x_percent, double rpm_timing_fraction,
                         double* us_per_rotation, double* elapsed_us) {
    (void)now_us; (void)right_x_percent; (void)rpm_timing_fraction; (void)us_per_rotation; (void)elapsed_us;
    return false;
}
uint8_t rssi_heading_state(void) { return RSSI_HEADING_IDLE; }
double rssi_heading_rpm(void) { return 0; }

#endif
