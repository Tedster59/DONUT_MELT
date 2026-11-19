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

// G event reasons: which way the raw killswitch reading glitched
#define LOG_GLITCH_READ_KILLED 1 // CH5 read killed while armed
#define LOG_GLITCH_READ_ARMED 2  // CH5 read armed while killed

// L event reasons: why the bot stopped being live
#define LOG_CUT_FAILSAFE 1
#define LOG_CUT_KILLSWITCH 2

// an event from outside the logger (kinds: 'K' killswitch on, 'k' killswitch off, 'G' CH5 glitch
// filtered by the debounce). Kept only while a run is in progress, like failsafe events.
// value_us: K/k = how long CH5 read that way before it took effect, G = how long the glitch lasted.
void logger_log_event(char kind, uint8_t reasons, uint32_t value_us, uint16_t ch5);

// a record from outside the logger, timestamped now; dropped unless the bot is live (recording).
// Kinds used by traction control: 'M' motor rpm (a = motor 1/2), 'T' TC state + fast accel rpm,
// 'U' throttle cap * 100 + reference rpm, 'V' (slip + 1) * 100 + wheel rpm in bot-equivalent rpm
void logger_log_record(char kind, uint8_t a, uint16_t b);

#endif
