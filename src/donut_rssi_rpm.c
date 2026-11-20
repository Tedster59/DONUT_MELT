#include "donut_rssi_rpm.h"

static bot_state_t* _user_bot_state;

// ---------- uniformly resampled RSSI history ----------
// The CRSF library hands us link stats with no timestamp and at an irregular rate,
// so each frame is timestamped on arrival and linearly interpolated onto a fixed
// RSSI_RPM_GRID_INTERVAL_US grid. A uniform grid is what makes Goertzel usable below.

static float grid_rssi[RSSI_RPM_BUF_SIZE];
static uint16_t grid_head = 0;    // next index to write
static uint16_t grid_count = 0;   // number of valid samples (saturates at RSSI_RPM_BUF_SIZE)
static uint64_t grid_last_ts = 0; // timestamp of the newest grid sample

// most recent raw (un-gridded) sample, the left end of the next interpolation segment
static bool has_raw = false;
static uint64_t raw_last_ts = 0;
static float raw_last_rssi = 0;

// ---------- estimator state ----------
static float window[RSSI_RPM_BUF_SIZE];        // Hann window
static float work[RSSI_RPM_BUF_SIZE];          // detrended + windowed copy of the grid
static float band_power[RSSI_RPM_SEARCH_STEPS]; // harmonic-sum power per candidate frequency
static float tone_norm;                        // scales power so a signal entirely in the summed harmonics scores ~1.0

// The scan is spread over many main loop calls (one candidate frequency per call) so no single
// call stalls the loop long enough to overflow the CRSF UART FIFO or jitter the motor timing.
typedef enum {
    SCAN_IDLE,         // waiting for the next RSSI_RPM_UPDATE_INTERVAL_US tick
    SCAN_BAND,         // evaluating band_power[scan_k]
    SCAN_SUB_HALF,     // harmonic-sum power at half the best frequency
    SCAN_SUB_THIRD,    // ... and at a third of it
} scan_phase_t;

static scan_phase_t scan_phase = SCAN_IDLE;
static uint64_t scan_start_us;  // when the window in work[] was captured
static int scan_k;
static int scan_best;
static float scan_lo_hz;
static float scan_step_hz;
static float scan_energy;
static float scan_sub_power;    // max harmonic-sum power at best/2 and best/3

static uint64_t last_update_us = 0;
static uint64_t last_valid_us = 0;
static double last_valid_rpm = 0;
static double last_valid_slope = 0; // corrected accel rpm trend over that window, rpm/s

// ---------- RPM curve calibration ----------
// Raw (uncorrected) accel rpm sampled at every update tick, covering the same span as the RSSI
// window, so the RSSI estimate can be compared against the accel rpm over the same period.
#define CAL_HISTORY ((RSSI_RPM_BUF_SIZE * RSSI_RPM_GRID_INTERVAL_US + RSSI_RPM_UPDATE_INTERVAL_US - 1) / RSSI_RPM_UPDATE_INTERVAL_US)
static float accel_hist[CAL_HISTORY];
static uint8_t accel_hist_head = 0;
static uint8_t accel_hist_count = 0;
static uint64_t cal_valid_us = 0;
static double cal_raw_rpm = 0;
static double cal_factor = 0;

static void grid_reset(void) {
    grid_head = 0;
    grid_count = 0;
}

static void grid_push(float value, uint64_t timestamp_us) {
    grid_rssi[grid_head] = value;
    grid_head = (grid_head + 1) % RSSI_RPM_BUF_SIZE;
    if (grid_count < RSSI_RPM_BUF_SIZE) {
        grid_count++;
    }
    grid_last_ts = timestamp_us;
}

void rssi_rpm_init(bot_state_t* user_bot_state) {
    _user_bot_state = user_bot_state;

    float sum_w = 0;
    float sum_w2 = 0;
    for (int i = 0; i < RSSI_RPM_BUF_SIZE; i++) {
        window[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (RSSI_RPM_BUF_SIZE - 1));
        sum_w += window[i];
        sum_w2 += window[i] * window[i];
    }
    // For x = A*cos(wt) windowed by w: peak power ~= (A/2 * sum_w)^2, energy ~= A^2/2 * sum_w2.
    // power/energy * tone_norm is therefore ~1 for a pure tone and ~3/N for white noise. Summing
    // RSSI_RPM_NUM_HARMONICS harmonics, it is ~1 when all the energy is in those harmonics and
    // ~3*H/N for white noise.
    tone_norm = 2.0f * sum_w2 / (sum_w * sum_w);

    grid_reset();
    has_raw = false;
}

void rssi_rpm_add_sample(uint64_t timestamp_us, uint8_t rssi) {
    float value = (float)rssi;

    if (has_raw && timestamp_us <= raw_last_ts) {
        return; // out of order / duplicate, nothing to interpolate across
    }

    if (!has_raw || timestamp_us - raw_last_ts > RSSI_RPM_MAX_GAP_US) {
        // first sample, or the link stalled long enough that interpolating across
        // the gap would be fiction: throw the history away and re-anchor the grid here
        grid_reset();
        grid_push(value, timestamp_us);
    } else {
        // fill in every grid point between the previous raw sample and this one
        float span = (float)(timestamp_us - raw_last_ts);
        uint64_t target_ts = grid_last_ts + RSSI_RPM_GRID_INTERVAL_US;
        while (target_ts <= timestamp_us) {
            float ratio = (float)(target_ts - raw_last_ts) / span;
            grid_push(raw_last_rssi + (value - raw_last_rssi) * ratio, target_ts);
            target_ts += RSSI_RPM_GRID_INTERVAL_US;
        }
    }

    has_raw = true;
    raw_last_ts = timestamp_us;
    raw_last_rssi = value;
}

// power of x at an arbitrary (non-bin) angular frequency omega, in radians per sample
static float goertzel_power(const float* x, int n, float omega) {
    float coeff = 2.0f * cosf(omega);
    float s1 = 0;
    float s2 = 0;
    for (int i = 0; i < n; i++) {
        float s = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

#define SAMPLE_RATE_HZ (1000000.0f / RSSI_RPM_GRID_INTERVAL_US)

// summed power of work[] at f, 2f, ... RSSI_RPM_NUM_HARMONICS * f (harmonics at or above
// Nyquist are skipped). The spinning antenna's RSSI pattern isn't a sine: on the 3lb the 3rd
// harmonic carries about twice the power of the fundamental, so the fundamental alone holds
// under 10% of the RSSI energy and could never reach a useful confidence.
static float harmonic_power(float f_hz) {
    float total = 0;
    for (int m = 1; m <= RSSI_RPM_NUM_HARMONICS; m++) {
        float fm = f_hz * m;
        if (fm >= 0.49f * SAMPLE_RATE_HZ) {
            break;
        }
        total += goertzel_power(work, RSSI_RPM_BUF_SIZE, 2.0f * (float)M_PI * fm / SAMPLE_RATE_HZ);
    }
    return total;
}

// a calibration point is only taken when the speed was steady across the whole RSSI window: the
// RSSI estimate averages ~0.5 s, the accel rpm only a few rotations, so during a spin-up the two
// would be measuring different speeds
static void update_calibration(uint64_t now, double rssi_rpm, bool prev_recent, double prev_rpm) {
    if (accel_hist_count < CAL_HISTORY || !prev_recent) {
        return;
    }
    float lo = accel_hist[0];
    float hi = accel_hist[0];
    float sum = 0;
    for (int i = 0; i < CAL_HISTORY; i++) {
        lo = fminf(lo, accel_hist[i]);
        hi = fmaxf(hi, accel_hist[i]);
        sum += accel_hist[i];
    }
    float mean = sum / CAL_HISTORY;
    if (mean < RSSI_RPM_MIN_SEED_RPM || (hi - lo) / mean > RSSI_CAL_MAX_SPREAD) {
        return;
    }
    if (fabs(rssi_rpm - prev_rpm) / rssi_rpm > RSSI_CAL_MAX_SPREAD) {
        return;
    }
    cal_valid_us = now;
    cal_raw_rpm = mean;
    cal_factor = rssi_rpm / mean;
}

// Speed trend over the RSSI window from the accel history (least-squares slope, rpm/s, in
// corrected rpm). The RSSI estimate averages the whole ~0.5 s window, so while the speed changes
// it describes the window's middle, not now; this slope moves it forward to the present.
static double accel_slope_rpm_per_s(void) {
    if (accel_hist_count < CAL_HISTORY) {
        return 0;
    }
    double mean_x = (CAL_HISTORY - 1) / 2.0;
    double mean_y = 0;
    for (int i = 0; i < CAL_HISTORY; i++) {
        mean_y += accel_hist[(accel_hist_head + i) % CAL_HISTORY]; // oldest first
    }
    mean_y /= CAL_HISTORY;
    double num = 0;
    double den = 0;
    for (int i = 0; i < CAL_HISTORY; i++) {
        double dx = i - mean_x;
        num += dx * (accel_hist[(accel_hist_head + i) % CAL_HISTORY] - mean_y);
        den += dx * dx;
    }
    double per_sample = num / den;
    return per_sample * (1000000.0 / RSSI_RPM_UPDATE_INTERVAL_US) * (double)_user_bot_state->rpm_correction;
}

static void publish(uint32_t rpm, float confidence) {
    _user_bot_state->rssi_rpm = rpm;
    _user_bot_state->rssi_rpm_confidence = confidence;
}

// captures the current RSSI window into work[] and sets up the band scan.
// Returns false (after publishing "no estimate") when there is nothing to scan.
static bool start_scan(uint64_t now) {
    accel_hist[accel_hist_head] = (float)_user_bot_state->rpm;
    accel_hist_head = (accel_hist_head + 1) % CAL_HISTORY;
    if (accel_hist_count < CAL_HISTORY) {
        accel_hist_count++;
    }

    // need a full, current window of history
    if (grid_count < RSSI_RPM_BUF_SIZE || now - grid_last_ts > RSSI_RPM_MAX_GAP_US) {
        publish(0, 0);
        return false;
    }

    // starting point from the accelerometer. Use the curve-corrected rpm: bot_state->rpm is the raw
    // accel rpm, and with a curve factor outside 1 +/- RSSI_RPM_SEARCH_FRACTION the real rpm would
    // be outside the search band and RSSI could never lock
    double seed_rpm = _user_bot_state->rpm * (double)_user_bot_state->rpm_correction;
    if (seed_rpm < RSSI_RPM_MIN_SEED_RPM) {
        publish(0, 0);
        return false;
    }

    // copy oldest -> newest, remove the mean (DC) and apply the window
    uint16_t start = (grid_head + RSSI_RPM_BUF_SIZE - grid_count) % RSSI_RPM_BUF_SIZE;
    float mean = 0;
    for (int i = 0; i < RSSI_RPM_BUF_SIZE; i++) {
        mean += grid_rssi[(start + i) % RSSI_RPM_BUF_SIZE];
    }
    mean /= RSSI_RPM_BUF_SIZE;

    float energy = 0;
    for (int i = 0; i < RSSI_RPM_BUF_SIZE; i++) {
        work[i] = (grid_rssi[(start + i) % RSSI_RPM_BUF_SIZE] - mean) * window[i];
        energy += work[i] * work[i];
    }
    if (energy <= 0) {
        publish(0, 0); // perfectly flat RSSI, nothing to find
        return false;
    }

    // search seed * (1 +/- RSSI_RPM_SEARCH_FRACTION), in RSSI cycles per second
    float seed_hz = (float)(seed_rpm / 60.0) * RSSI_RPM_HARMONIC;
    float lo_hz = seed_hz * (1.0f - RSSI_RPM_SEARCH_FRACTION);
    float hi_hz = fminf(seed_hz * (1.0f + RSSI_RPM_SEARCH_FRACTION), 0.49f * SAMPLE_RATE_HZ);
    if (hi_hz <= lo_hz) {
        publish(0, 0); // seed is above what the grid can represent
        return false;
    }

    scan_start_us = now;
    scan_lo_hz = lo_hz;
    scan_step_hz = (hi_hz - lo_hz) / (RSSI_RPM_SEARCH_STEPS - 1);
    scan_energy = energy;
    scan_k = 0;
    scan_best = 0;
    return true;
}

static float scan_freq(int k) {
    return scan_lo_hz + k * scan_step_hz;
}

// all powers are in: pick the peak, apply the lock checks and publish
static void finish_scan(void) {
    int best = scan_best;

    // parabolic interpolation between search steps for sub-step resolution
    float offset = 0;
    bool peak_on_edge = (best == 0 || best == RSSI_RPM_SEARCH_STEPS - 1);
    if (!peak_on_edge) {
        float a = band_power[best - 1];
        float b = band_power[best];
        float c = band_power[best + 1];
        float denom = a - 2.0f * b + c;
        if (denom < 0) {
            offset = fmaxf(-0.5f, fminf(0.5f, 0.5f * (a - c) / denom));
        }
    }

    float peak_hz = scan_lo_hz + (best + offset) * scan_step_hz;
    double rpm = peak_hz * 60.0 / RSSI_RPM_HARMONIC;
    float confidence = band_power[best] / scan_energy * tone_norm;

    publish((uint32_t)rpm, confidence);

    // Lock checks:
    // - a peak pinned to the band edge most likely means the real peak is outside the band
    //   (accel seed is off by more than RSSI_RPM_SEARCH_FRACTION)
    // - if half or a third of the peak frequency explains the RSSI about as well, the peak is a
    //   harmonic of a slower real rotation (seen when the seed was stale during a coast-down)
    bool is_harmonic = scan_sub_power >= RSSI_RPM_MAX_SUBHARMONIC_RATIO * band_power[best];
    if (!peak_on_edge && !is_harmonic && confidence >= RSSI_RPM_MIN_CONFIDENCE) {
        uint64_t t = scan_start_us;
        bool prev_recent = last_valid_us != 0 && t - last_valid_us <= 2 * RSSI_RPM_UPDATE_INTERVAL_US;
        double prev_rpm = last_valid_rpm;
        last_valid_us = t;
        last_valid_rpm = rpm;
        last_valid_slope = accel_slope_rpm_per_s();
        update_calibration(t, rpm, prev_recent, prev_rpm);
    }
}

void rssi_rpm_update(void) {
    switch (scan_phase) {
        case SCAN_IDLE: {
            uint64_t now = time_us_64();
            if (now - last_update_us < RSSI_RPM_UPDATE_INTERVAL_US) {
                return;
            }
            last_update_us = now;
            if (start_scan(now)) {
                scan_phase = SCAN_BAND;
            }
            return;
        }

        case SCAN_BAND:
            band_power[scan_k] = harmonic_power(scan_freq(scan_k));
            if (band_power[scan_k] > band_power[scan_best]) {
                scan_best = scan_k;
            }
            if (++scan_k == RSSI_RPM_SEARCH_STEPS) {
                scan_phase = SCAN_SUB_HALF;
            }
            return;

        case SCAN_SUB_HALF:
            scan_sub_power = harmonic_power(scan_freq(scan_best) / 2.0f);
            scan_phase = SCAN_SUB_THIRD;
            return;

        case SCAN_SUB_THIRD:
            scan_sub_power = fmaxf(scan_sub_power, harmonic_power(scan_freq(scan_best) / 3.0f));
            finish_scan();
            scan_phase = SCAN_IDLE;
            return;
    }
}

bool rssi_rpm_is_locked(void) {
    return last_valid_us != 0 && time_us_64() - last_valid_us <= RSSI_RPM_MAX_AGE_US;
}

uint8_t rssi_rpm_state(void) {
    if (!rssi_rpm_is_locked()) {
        return RSSI_STATE_UNLOCKED;
    }
    return _user_bot_state->rpm_source == RPM_SOURCE_RSSI ? RSSI_STATE_DRIVING : RSSI_STATE_SHADOW;
}

bool rssi_rpm_get_calibration(double* raw_rpm, double* factor) {
    if (cal_valid_us == 0 || time_us_64() - cal_valid_us > RSSI_RPM_MAX_AGE_US) {
        return false;
    }
    *raw_rpm = cal_raw_rpm;
    *factor = cal_factor;
    return true;
}

double get_rpm_rssi(double right_x_percent, double accel_offset_cm) {
    // always run the accel path: it keeps bot_state->rpm (our search seed) current
    // and is the fallback whenever the RSSI estimate is stale or unconfident.
    // (get_rpm_2accel sets rpm_source = ACCEL and drive_rpm = the corrected accel rpm)
    double accel_rpm = get_rpm_2accel(right_x_percent, accel_offset_cm);

    if (!rssi_rpm_is_locked()) {
        return accel_rpm;
    }

    // The estimate describes the middle of its ~0.5 s window: move it to now with the speed trend
    // (spinup_two.csv: RSSI read 3-10% low all through the spin-up without this)
    double window_mid_us = last_valid_us - RSSI_RPM_BUF_SIZE * RSSI_RPM_GRID_INTERVAL_US / 2.0;
    double rpm = last_valid_rpm + last_valid_slope * ((double)time_us_64() - window_mid_us) / 1000000.0;

    // Only drive on RSSI when the speed is steady enough for that correction to be small, and the
    // result agrees with the accelerometer; otherwise the accelerometer is the better number
    double accel_corrected = _user_bot_state->accel_rpm_corrected;
    if (fabs(last_valid_slope) > RSSI_RPM_MAX_SLOPE_RPM_S ||
        (accel_corrected > 0 && fabs(rpm - accel_corrected) / accel_corrected > RSSI_RPM_MAX_ACCEL_DIFF)) {
        return accel_rpm;
    }

    // RSSI RPM is a direct measurement, so the accel correction curve/trim is not applied
    // (and the drive ignores left stick X while on RSSI); heading control via right_x_percent
    // works the same as get_rpm_2accel
    _user_bot_state->rpm_source = RPM_SOURCE_RSSI;
    _user_bot_state->drive_rpm = (uint32_t)rpm;
    return rpm + rpm * right_x_percent * HEADING_CONTROL_SENSITIVITY;
}
