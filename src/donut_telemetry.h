#ifndef DONUT_TELEMETRY_H
#define DONUT_TELEMETRY_H

#include "donut_config.h"

void telemetry_output_diagnostics(bot_state_t* bot_state);

void telemetry_send_telemetry(bot_state_t* bot_state);

// call every main loop; prints a link stats summary once per second (RSSI_LINK_DIAGNOSTICS)
// and/or drains raw RSSI samples as CSV (RSSI_RAW_DUMP)
void telemetry_output_link_diagnostics(bot_state_t* bot_state);

// call every main loop; prints bidirectional DShot eRPM per motor once per second (ESC_TELEMETRY_DIAGNOSTICS)
void telemetry_output_esc_diagnostics(void);

#endif