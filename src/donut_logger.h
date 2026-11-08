#ifndef DONUT_LOGGER_H
#define DONUT_LOGGER_H

#include "donut_config.h"

// Flash run recorder (USE_FLASH_LOGGER).
// While the bot is live, link stats RSSI samples and periodic RPM/throttle snapshots are
// recorded into a RAM ring buffer (the most recent LOGGER_BUF_BYTES worth is kept). Once the
// killswitch has been on for LOGGER_SAVE_DELAY_MS, the run is written to flash, where it
// survives power-off and firmware reflashes. Over USB serial, with the bot not live:
//   d = dump the saved run as CSV, x = erase it, s = status, h = help

// Declared unconditionally: this header is pulled in from the top of donut_config.h, before
// USE_FLASH_LOGGER is defined. The definitions are compiled only with USE_FLASH_LOGGER.

void logger_init(void);

// one link stats frame (called from the receiver's link stats callback)
void logger_log_rssi(uint64_t timestamp_us, uint8_t rssi, uint8_t link_quality);

// call every main loop. live = motors may be driven; killed = killswitch active.
void logger_update(bot_state_t* bot_state, bool live, bool killed);

#endif
