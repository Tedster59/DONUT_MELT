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
static float band_power[RSSI_RPM_SEARCH_STEPS];
static float tone_norm;                        // scales peak power so a pure tone scores ~1.0

static uint64_t last_update_us = 0;
static uint64_t last_valid_us = 0;
static double last_valid_rpm = 0;

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
    // power/energy * tone_norm is therefore ~1 for a pure tone and ~3/N for white noise.
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

static void publish(uint32_t rpm, float confidence) {
    _user_bot_state->rssi_rpm = rpm;
    _user_bot_state->rssi_rpm_confidence = confidence;
}

void rssi_rpm_update(void) {
    uint64_t now = time_us_64();
    if (now - last_update_us < RSSI_RPM_UPDATE_INTERVAL_US) {
        return;
    }
    last_update_us = now;

    // need a full, current window of history
    if (grid_count < RSSI_RPM_BUF_SIZE || now - grid_last_ts > RSSI_RPM_MAX_GAP_US) {
        publish(0, 0);
        return;
    }

    // starting point from the accelerometer
    double seed_rpm = _user_bot_state->rpm;
    if (seed_rpm < RSSI_RPM_MIN_SEED_RPM) {
        publish(0, 0);
        return;
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
        return;
    }

    // search seed * (1 +/- RSSI_RPM_SEARCH_FRACTION), in RSSI cycles per second
    float sample_rate_hz = 1000000.0f / RSSI_RPM_GRID_INTERVAL_US;
    float seed_hz = (float)(seed_rpm / 60.0) * RSSI_RPM_HARMONIC;
    float lo_hz = seed_hz * (1.0f - RSSI_RPM_SEARCH_FRACTION);
    float hi_hz = fminf(seed_hz * (1.0f + RSSI_RPM_SEARCH_FRACTION), 0.49f * sample_rate_hz);
    if (hi_hz <= lo_hz) {
        publish(0, 0); // seed is above what the grid can represent
        return;
    }
    float step_hz = (hi_hz - lo_hz) / (RSSI_RPM_SEARCH_STEPS - 1);

    int best = 0;
    for (int k = 0; k < RSSI_RPM_SEARCH_STEPS; k++) {
        float omega = 2.0f * (float)M_PI * (lo_hz + k * step_hz) / sample_rate_hz;
        band_power[k] = goertzel_power(work, RSSI_RPM_BUF_SIZE, omega);
        if (band_power[k] > band_power[best]) {
            best = k;
        }
    }

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

    float peak_hz = lo_hz + (best + offset) * step_hz;
    double rpm = peak_hz * 60.0 / RSSI_RPM_HARMONIC;
    float confidence = band_power[best] / energy * tone_norm;

    publish((uint32_t)rpm, confidence);

    // a peak pinned to the band edge most likely means the real peak is outside the
    // band (accel seed is off by more than RSSI_RPM_SEARCH_FRACTION), so don't trust it
    if (!peak_on_edge && confidence >= RSSI_RPM_MIN_CONFIDENCE) {
        last_valid_us = now;
        last_valid_rpm = rpm;
    }
}

double get_rpm_rssi(double right_x_percent, double accel_offset_cm) {
    // always run the accel path: it keeps bot_state->rpm (our search seed) current
    // and is the fallback whenever the RSSI estimate is stale or unconfident
    double accel_rpm = get_rpm_2accel(right_x_percent, accel_offset_cm);

    if (last_valid_us == 0 || time_us_64() - last_valid_us > RSSI_RPM_MAX_AGE_US) {
        return accel_rpm;
    }

    // RSSI RPM is a direct measurement, so the left-stick accel trim multiplier is not
    // applied; heading control via right_x_percent works the same as get_rpm_2accel
    return last_valid_rpm + last_valid_rpm * right_x_percent * HEADING_CONTROL_SENSITIVITY;
}
