#include "receiver.h"

static uint16_t _user_channels[16];
static bot_state_t* _user_bot_state;
static crsf_instance _user_crsf_instance;

// frame counters + per-window link stats info, read by the RSSI link diagnostics
static receiver_link_info_t _link_info = {
    .min_gap_us = UINT32_MAX,
    .rssi_min = UINT8_MAX,
};
static uint64_t _last_link_stats_us = 0;

#ifdef RSSI_RAW_DUMP
    // raw link stats samples waiting to be printed; overwritten (and counted) if printing falls behind
    #define RAW_DUMP_BUF_SIZE 512
    static receiver_raw_sample_t _raw_buf[RAW_DUMP_BUF_SIZE];
    static uint16_t _raw_head = 0;
    static uint16_t _raw_count = 0;
    static uint32_t _raw_overwritten = 0;
#endif

void on_rc_channels(const uint16_t channels[16]) {
    // printf("on_rc_channels was called \n");
    _link_info.rc_frames++;
    for (int i = 0; i < 16; i++){
        _user_channels[i] = channels[i];
    }
}

uint16_t receiver_get_channel(Channel channel) {
    return _user_channels[channel];
}

uint8_t receiver_is_channel_near_value(Channel channel, uint16_t value, uint16_t tolerance) {
    return fabs(receiver_get_channel(channel) - value) <= tolerance;
}

double receiver_get_percent_for_channel(Channel channel) {
    return fmin(1.0, fmax(0.0, ((double)(receiver_get_channel(channel) - RECEIVER_LOWEST_CHANNEL_VALUE)) / ((double)RECEIVER_HIGHEST_CHANNEL_VALUE - RECEIVER_LOWEST_CHANNEL_VALUE)));
}

void on_link_stats(const link_statistics_t link_stats) {
    // the crsf library doesn't timestamp frames, so stamp on arrival. This runs from
    // receiver_update() in the main loop, so jitter is ~one loop iteration.
    uint64_t now_us = time_us_64();
    rssi_rpm_add_sample(now_us, link_stats.rssi);

    _link_info.link_stats_frames++;
    _link_info.last = link_stats;
    if (_last_link_stats_us != 0) {
        uint64_t gap64 = now_us - _last_link_stats_us;
        uint32_t gap_us = gap64 > UINT32_MAX ? UINT32_MAX : (uint32_t)gap64;
        if (gap_us < _link_info.min_gap_us) _link_info.min_gap_us = gap_us;
        if (gap_us > _link_info.max_gap_us) _link_info.max_gap_us = gap_us;
    }
    _last_link_stats_us = now_us;
    if (link_stats.rssi < _link_info.rssi_min) _link_info.rssi_min = link_stats.rssi;
    if (link_stats.rssi > _link_info.rssi_max) _link_info.rssi_max = link_stats.rssi;

    #ifdef RSSI_RAW_DUMP
        if (_raw_count == RAW_DUMP_BUF_SIZE) {
            _raw_overwritten++;
        } else {
            _raw_count++;
        }
        _raw_buf[_raw_head] = (receiver_raw_sample_t){ .timestamp_us = now_us, .rssi = link_stats.rssi, .link_quality = link_stats.link_quality };
        _raw_head = (_raw_head + 1) % RAW_DUMP_BUF_SIZE;
    #endif

    // _user_bot_state->crsf_link_quality = link_stats.link_quality;
    // _user_bot_state->crsf_link_quality = link_stats.rssi;
    // _user_bot_state->crsf_snr = link_stats.snr;
    // _user_bot_state->crsf_tx_power = link_stats.tx_power;
}

receiver_link_info_t receiver_take_link_info() {
    receiver_link_info_t info = _link_info;
    // totals keep counting; the min/max fields restart for the next window
    _link_info.min_gap_us = UINT32_MAX;
    _link_info.max_gap_us = 0;
    _link_info.rssi_min = UINT8_MAX;
    _link_info.rssi_max = 0;
    return info;
}

#ifdef RSSI_RAW_DUMP
    bool receiver_pop_raw_sample(receiver_raw_sample_t* out) {
        if (_raw_count == 0) {
            return false;
        }
        *out = _raw_buf[(_raw_head + RAW_DUMP_BUF_SIZE - _raw_count) % RAW_DUMP_BUF_SIZE];
        _raw_count--;
        return true;
    }

    uint32_t receiver_raw_samples_overwritten() {
        return _raw_overwritten;
    }
#endif

void on_failsafe(const bool failsafe) {
    _user_bot_state->is_failsafed = failsafe;
}

void receiver_send_telemetry(uint16_t diag1, uint16_t diag2, uint32_t diag3, uint8_t diag4) {
    //crsf_telem_set_battery_data(&_user_crsf_instance, 3, 2, 1, 50);
    crsf_telem_set_battery_data(&_user_crsf_instance, diag1, diag2, diag3, diag4);
}

void receiver_init(uart_inst_t* uart_id, int tx_pin, int rx_pin, int link_quality_threshold, int rssi_threshold, bot_state_t* user_bot_state) {
    crsf_init(&_user_crsf_instance);

    crsf_set_link_quality_threshold(&_user_crsf_instance, link_quality_threshold);
    crsf_set_rssi_threshold(&_user_crsf_instance, rssi_threshold);

    // printf("receiver_init pre_callbacks \n");

    crsf_set_on_rc_channels(&_user_crsf_instance, on_rc_channels);
    crsf_set_on_link_statistics(&_user_crsf_instance, on_link_stats);
    crsf_set_on_failsafe(&_user_crsf_instance, on_failsafe);

    // printf("receiver_init post_callbacks \n");

    crsf_begin(&_user_crsf_instance, uart_id, tx_pin, rx_pin);

    // printf("post crsf_begin \n");

    _user_bot_state = user_bot_state;
    rssi_rpm_init(user_bot_state);
}

void receiver_update() {
    crsf_process_frames(&_user_crsf_instance);
}
