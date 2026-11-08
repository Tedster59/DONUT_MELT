#ifndef RECEIVER_H
#define RECEIVER_H

#include "donut_config.h"
#include "crsf.h"

typedef enum {
    RIGHT_JOYSTICK_X,
    RIGHT_JOYSTICK_Y,
    LEFT_JOYSTICK_Y,
    LEFT_JOYSTICK_X,
    SWITCH_E,
    SWITCH_B,
    SWITCH_C,
    SWITCH_F,
    KNOB_S1,
    KNOB_S2
} Channel;

typedef struct {
    uint32_t rc_frames;          // total RC channel frames received since boot
    uint32_t link_stats_frames;  // total link statistics frames received since boot
    uint32_t min_gap_us;         // shortest/longest time between link stats frames this window
    uint32_t max_gap_us;
    uint8_t rssi_min;            // RSSI range this window (dBm * -1, so min is the strongest)
    uint8_t rssi_max;
    link_statistics_t last;      // most recent link stats frame
} receiver_link_info_t;

typedef struct {
    uint64_t timestamp_us;
    uint8_t rssi;
    uint8_t link_quality;
} receiver_raw_sample_t;

// returns the link counters and resets the per-window min/max fields
receiver_link_info_t receiver_take_link_info();

#ifdef RSSI_RAW_DUMP
    // pops the oldest unprinted link stats sample; false when none are waiting
    bool receiver_pop_raw_sample(receiver_raw_sample_t* out);
    uint32_t receiver_raw_samples_overwritten();
#endif

uint16_t receiver_get_channel(Channel channel);

double receiver_get_percent_for_channel(Channel channel);

uint8_t receiver_is_channel_near_value(Channel channel, uint16_t value, uint16_t tolerance);

void receiver_send_telemetry(uint16_t diag1, uint16_t diag2, uint32_t diag3, uint8_t diag4);

void receiver_init(uart_inst_t* uart_id, int tx_pin, int rx_pin, int link_quality_threshold, int rssi_threshold, bot_state_t* user_bot_state);

void receiver_update();

#endif