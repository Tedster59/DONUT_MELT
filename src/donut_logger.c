#include "donut_logger.h"

#ifdef USE_FLASH_LOGGER

#include <string.h>
#include "donut_flash.h"

#define LOG_MAGIC 0x474F4C44u // "DLOG"
#define LOG_VERSION 3 // v2: failsafe event list after the header (v1 logs still dump, without events)
                      // v3: events gain a ch5 field (v2 events still dump, with ch5 blank)

typedef struct __attribute__((packed)) {
    uint32_t t_us;  // microseconds since the run started
    char kind;      // 'R' link stats sample, 'A' accel/throttle snapshot, 'P' RSSI RPM estimate,
                    // 'D' drive source, 'C' accel correction
    uint8_t a;      // R: rssi (-dBm)     A: throttle %   P: confidence * 100   D: RSSI_STATE_*   C: correction * 100
    uint16_t b;     // R: link quality %  A: accel rpm    P: rssi rpm           D: drive rpm      C: corrected accel rpm
} log_record_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t record_size;
    uint32_t record_count;
    uint32_t dropped;      // oldest records overwritten because the RAM buffer filled
    uint32_t event_count;  // v2+
    uint32_t events_dropped;
} log_header_t;

// Failsafe events are kept in their own small list rather than the record ring buffer, so a
// failsafe early in a long run isn't overwritten by the later records (first_spin.csv lost its
// first 81 s, including the failsafe it was meant to explain). The first LOG_MAX_EVENTS are kept.
typedef struct __attribute__((packed)) {
    uint32_t t_us;      // microseconds since the run started
    char kind;          // 'F' failsafe started, 'f' failsafe cleared, 'W' this run followed a watchdog reboot,
                        // 'L' stopped being live, 'l' live again, 'K' killswitch on, 'k' killswitch off,
                        // 'G' CH5 glitch filtered by the killswitch debounce
    uint8_t reasons;    // F: FAILSAFE_REASON_* bits   L: LOG_CUT_* bits   G: LOG_GLITCH_* (which way it read)
    uint8_t lq;         // last link stats link quality %
    uint8_t rssi;       // last link stats RSSI (-dBm)
    uint32_t value_us;  // F: time since the last RC frame   f: how long the failsafe lasted
                        // l: how long the bot was not live   K/k: how long CH5 read that way first
                        // G: how long the glitch lasted
    uint16_t ch5;       // raw CH5 (killswitch): K/k/G = when the new reading started, others = when logged
} log_event_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    char kind;
    uint8_t reasons;
    uint8_t lq;
    uint8_t rssi;
    uint32_t value_us;
} log_event_v2_t; // what v2 firmware saved, so dump_saved_log can still read it

#define LOG_MAX_EVENTS 64
#define LOG_EVENTS_OFFSET (LOGGER_FLASH_OFFSET + FLASH_PAGE_SIZE) // rest of the header sector

#define LOG_RECORD_SIZE sizeof(log_record_t)
#define LOG_MAX_RECORDS (LOGGER_BUF_BYTES / LOG_RECORD_SIZE)
#define LOG_DATA_REGION_BYTES (((LOGGER_BUF_BYTES) + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE * FLASH_SECTOR_SIZE)
// layout: one header sector, then the records in chronological order
#define LOG_HEADER_OFFSET (LOGGER_FLASH_OFFSET)
#define LOG_DATA_OFFSET (LOGGER_FLASH_OFFSET + FLASH_SECTOR_SIZE)

_Static_assert(sizeof(log_record_t) == 8, "log record must stay 8 bytes");
_Static_assert(FLASH_PAGE_SIZE % sizeof(log_record_t) == 0, "records must not straddle flash pages");
_Static_assert(LOGGER_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0, "LOGGER_FLASH_OFFSET must be sector aligned");
_Static_assert(sizeof(log_header_t) <= FLASH_PAGE_SIZE, "log header must fit in one flash page");
_Static_assert(FLASH_PAGE_SIZE + LOG_MAX_EVENTS * sizeof(log_event_t) <= FLASH_SECTOR_SIZE, "events must fit in the header sector");


static log_record_t ram_log[LOG_MAX_RECORDS];
static uint32_t log_head = 0;   // next index to write
static uint32_t log_count = 0;
static uint32_t log_dropped = 0;

static bool disabled = false;
static bool recording = false;
static bool have_unsaved = false;
static uint64_t run_start_us = 0;
static uint64_t killed_since_us = 0;
static uint64_t last_status_us = 0;
static uint64_t last_rssi_rpm_us = 0;

static log_event_t events[LOG_MAX_EVENTS];
static uint32_t event_count = 0;
static uint32_t events_dropped = 0;
static bool prev_failsafed = true;     // the bot boots failsafed, which isn't an event
static bool prev_live = false;
static uint64_t not_live_since_us = 0;
static uint64_t failsafe_start_us = 0;
static bool boot_after_watchdog = false;
static uint8_t last_lq = 0;
static uint8_t last_rssi = 0;

static const log_header_t* saved_header(void) {
    return (const log_header_t*)donut_flash_read_ptr(LOG_HEADER_OFFSET);
}

static bool saved_log_valid(void) {
    const log_header_t* h = saved_header();
    return h->magic == LOG_MAGIC && h->version >= 1 && h->version <= LOG_VERSION &&
           h->record_size == LOG_RECORD_SIZE && h->record_count <= LOG_MAX_RECORDS;
}

static uint32_t saved_event_count(void) {
    const log_header_t* h = saved_header();
    if (h->version < 2 || h->event_count > LOG_MAX_EVENTS) {
        return 0; // v1 logs have no event list (and the field reads as erased flash)
    }
    return h->event_count;
}

static void push_event(uint64_t now_us, char kind, uint8_t reasons, uint32_t value_us, uint16_t ch5) {
    if (event_count == LOG_MAX_EVENTS) {
        events_dropped++; // keep the first ones: the first failsafe is usually the interesting one
        return;
    }
    events[event_count++] = (log_event_t){
        .t_us = (uint32_t)(now_us - run_start_us),
        .kind = kind,
        .reasons = reasons,
        .lq = last_lq,
        .rssi = last_rssi,
        .value_us = value_us,
        .ch5 = ch5,
    };
    have_unsaved = true;
}

// a run is in progress: live, or ended but not saved yet
static bool run_active(void) {
    return recording || have_unsaved;
}

void logger_log_event(char kind, uint8_t reasons, uint32_t value_us, uint16_t ch5) {
    if (disabled || !run_active()) {
        return;
    }
    push_event(time_us_64(), kind, reasons, value_us, ch5);
}

static void push_record(uint64_t now_us, char kind, uint8_t a, uint16_t b) {
    if (!recording) {
        return;
    }
    if (log_count == LOG_MAX_RECORDS) {
        log_dropped++; // overwrite the oldest, so the most recent part of the run is kept
    } else {
        log_count++;
    }
    ram_log[log_head] = (log_record_t){ .t_us = (uint32_t)(now_us - run_start_us), .kind = kind, .a = a, .b = b };
    log_head = (log_head + 1) % LOG_MAX_RECORDS;
    have_unsaved = true;
}

void logger_init(void) {
    // never let the log region overlap the program image
    if (!donut_flash_region_safe(LOGGER_FLASH_OFFSET, FLASH_SECTOR_SIZE + LOG_DATA_REGION_BYTES)) {
        disabled = true;
        printf("LOGGER disabled: firmware overlaps LOGGER_FLASH_OFFSET\n");
    }
    // the RAM log of the run that was going on is gone; note the reboot in the next run instead
    boot_after_watchdog = watchdog_caused_reboot();
}

void logger_log_rssi(uint64_t timestamp_us, uint8_t rssi, uint8_t link_quality) {
    last_lq = link_quality;
    last_rssi = rssi;
    push_record(timestamp_us, 'R', rssi, link_quality);
}

// ---------- flash ----------
// (donut_flash.c) Erasing/programming stalls the loop (~50ms per 4KB sector), so this only ever
// runs while killed; the watchdog is fed before every sector erase.

static void save_to_flash(void) {
    uint64_t start_us = time_us_64();
    led_set_and_update_state(1); // solid LED while saving

    uint32_t data_bytes = log_count * LOG_RECORD_SIZE;
    uint32_t erase_bytes = (data_bytes + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE * FLASH_SECTOR_SIZE;

    // the header is erased first and written last, so a power cut mid-save leaves no valid log
    donut_flash_erase_sector(LOG_HEADER_OFFSET);
    for (uint32_t off = 0; off < erase_bytes; off += FLASH_SECTOR_SIZE) {
        donut_flash_erase_sector(LOG_DATA_OFFSET + off);
    }

    uint8_t page[FLASH_PAGE_SIZE];
    const uint32_t records_per_page = FLASH_PAGE_SIZE / LOG_RECORD_SIZE;
    uint32_t oldest = (log_head + LOG_MAX_RECORDS - log_count) % LOG_MAX_RECORDS;
    for (uint32_t first = 0; first < log_count; first += records_per_page) {
        memset(page, 0xFF, sizeof(page));
        for (uint32_t j = 0; j < records_per_page && first + j < log_count; j++) {
            memcpy(page + j * LOG_RECORD_SIZE, &ram_log[(oldest + first + j) % LOG_MAX_RECORDS], LOG_RECORD_SIZE);
        }
        donut_flash_program_page(LOG_DATA_OFFSET + first * LOG_RECORD_SIZE, page);
        if ((first / records_per_page) % 16 == 0) {
            watchdog_update();
        }
    }

    // events go in the rest of the header sector (erased above with the header)
    for (uint32_t off = 0; off < event_count * sizeof(log_event_t); off += FLASH_PAGE_SIZE) {
        memset(page, 0xFF, sizeof(page));
        uint32_t n = event_count * sizeof(log_event_t) - off;
        memcpy(page, (const uint8_t*)events + off, n < FLASH_PAGE_SIZE ? n : FLASH_PAGE_SIZE);
        donut_flash_program_page(LOG_EVENTS_OFFSET + off, page);
    }

    log_header_t header = {
        .magic = LOG_MAGIC,
        .version = LOG_VERSION,
        .record_size = LOG_RECORD_SIZE,
        .record_count = log_count,
        .dropped = log_dropped,
        .event_count = event_count,
        .events_dropped = events_dropped,
    };
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &header, sizeof(header));
    donut_flash_program_page(LOG_HEADER_OFFSET, page);

    have_unsaved = false;
    printf("LOGGER saved %lu records (%lu dropped), %lu events in %lu ms\n",
        (unsigned long)log_count, (unsigned long)log_dropped, (unsigned long)event_count,
        (unsigned long)((time_us_64() - start_us) / 1000));
}

// ---------- USB serial commands ----------

static void dump_saved_log(void) {
    if (!saved_log_valid()) {
        printf("LOGGER no saved run\n");
        return;
    }
    const log_header_t* h = saved_header();
    const log_record_t* records = (const log_record_t*)donut_flash_read_ptr(LOG_DATA_OFFSET);

    uint32_t n_events = saved_event_count();
    const uint8_t* ev_base = (const uint8_t*)donut_flash_read_ptr(LOG_EVENTS_OFFSET);

    printf("# DONUT_LOG v%lu records=%lu dropped=%lu events=%lu events_dropped=%lu\n",
        (unsigned long)h->version, (unsigned long)h->record_count, (unsigned long)h->dropped,
        (unsigned long)n_events, (unsigned long)(h->version >= 2 ? h->events_dropped : 0));
    // 15 columns; every row prints all of them
    printf("kind,t_us,rssi,lq,throttle_pct,accel_rpm,rssi_rpm,confidence,rssi_state,drive_rpm,correction,accel_rpm_corrected,fs_reasons,fs_ms,ch5\n");
    for (uint32_t i = 0; i < h->record_count; i++) {
        const log_record_t* r = &records[i];
        switch (r->kind) {
            case 'R': printf("R,%lu,%u,%u,,,,,,,,,,,\n", (unsigned long)r->t_us, r->a, r->b); break;
            case 'A': printf("A,%lu,,,%u,%u,,,,,,,,,\n", (unsigned long)r->t_us, r->a, r->b); break;
            case 'P': printf("P,%lu,,,,,%u,%.2f,,,,,,,\n", (unsigned long)r->t_us, r->b, r->a / 100.0f); break;
            case 'D': printf("D,%lu,,,,,,,%u,%u,,,,,\n", (unsigned long)r->t_us, r->a, r->b); break;
            case 'C': printf("C,%lu,,,,,,,,,%.2f,%u,,,\n", (unsigned long)r->t_us, r->a / 100.0f, r->b); break;
            default: break;
        }
        if ((i & 255) == 0) {
            watchdog_update();
        }
    }
    // events (see log_event_t): F/f failsafe start/end, W watchdog reboot, L/l live lost/regained,
    // K/k killswitch on/off, G killswitch glitch filtered. fs_reasons = the event's reason bits,
    // fs_ms = its value_us in ms, ch5 = raw killswitch channel (blank for v2 logs)
    for (uint32_t i = 0; i < n_events; i++) {
        log_event_t e;
        if (h->version >= 3) {
            memcpy(&e, ev_base + i * sizeof(log_event_t), sizeof(e));
        } else {
            log_event_v2_t old;
            memcpy(&old, ev_base + i * sizeof(log_event_v2_t), sizeof(old));
            e = (log_event_t){ .t_us = old.t_us, .kind = old.kind, .reasons = old.reasons, .lq = old.lq,
                               .rssi = old.rssi, .value_us = old.value_us, .ch5 = UINT16_MAX };
        }
        if (e.kind == 0 || strchr("FfWLlKkG", e.kind) == NULL) {
            continue;
        }
        printf("%c,%lu,%u,%u,,,,,,,,,%u,%.1f,", e.kind, (unsigned long)e.t_us, e.rssi, e.lq, e.reasons,
            e.value_us / 1000.0f);
        if (e.ch5 != UINT16_MAX) {
            printf("%u", e.ch5);
        }
        printf("\n");
    }
    printf("# END\n");
}

static void print_status(void) {
    printf("LOGGER RAM: %lu records (%lu dropped), %lu events, %s | flash: ",
        (unsigned long)log_count, (unsigned long)log_dropped, (unsigned long)event_count,
        have_unsaved ? "unsaved" : "saved");
    if (saved_log_valid()) {
        printf("%lu records, %lu events\n", (unsigned long)saved_header()->record_count, (unsigned long)saved_event_count());
    } else {
        printf("empty\n");
    }
}

static void handle_serial_command(void) {
    int c = getchar_timeout_us(0);
    switch (c) {
        case 'd': dump_saved_log(); break;
        case 'x':
            donut_flash_erase_sector(LOG_HEADER_OFFSET);
            printf("LOGGER saved run erased\n");
            break;
        case 's': print_status(); break;
        case 'h':
        case '?':
            printf("LOGGER commands: d = dump saved run as CSV, x = erase it, s = status\n");
            break;
        default: break;
    }
}

void logger_update(bot_state_t* bot_state, bool live, bool killed) {
    if (disabled) {
        return;
    }
    uint64_t now = time_us_64();

    // failsafe transitions, logged while a run is in progress (live, or ended but not saved yet).
    // Checked before the live branch so the onset is seen while recording is still true.
    bool failsafed = bot_state->is_failsafed;
    if (failsafed != prev_failsafed) {
        uint16_t ch5 = receiver_get_channel(SWITCH_E);
        if (failsafed) {
            failsafe_start_us = now;
            if (run_active()) {
                push_event(now, 'F', receiver_failsafe_reasons(), receiver_rc_frame_age_us(), ch5);
            }
        } else if (run_active() && failsafe_start_us != 0) {
            uint64_t len = now - failsafe_start_us;
            push_event(now, 'f', 0, len > UINT32_MAX ? UINT32_MAX : (uint32_t)len, ch5);
        }
        prev_failsafed = failsafed;
    }

    // live transitions, whatever the cause. By the time this runs, when_failsafe_on() has already
    // latched require_zero_throttle, so that isn't a reason here: it only ever keeps the bot down
    // after a failsafe or the killswitch cut it. Reasons 0 = neither (shouldn't happen).
    if (live != prev_live) {
        uint16_t ch5 = receiver_get_channel(SWITCH_E);
        if (!live) {
            not_live_since_us = now;
            if (run_active()) {
                uint8_t why = (bot_state->is_failsafed ? LOG_CUT_FAILSAFE : 0) | (killed ? LOG_CUT_KILLSWITCH : 0);
                push_event(now, 'L', why, 0, ch5);
            }
        } else if (run_active() && not_live_since_us != 0) {
            uint64_t len = now - not_live_since_us;
            push_event(now, 'l', 0, len > UINT32_MAX ? UINT32_MAX : (uint32_t)len, ch5);
        }
        prev_live = live;
    }

    if (live) {
        if (!recording) {
            if (!have_unsaved) {
                // previous run was saved (or none yet): start a new one
                log_head = 0;
                log_count = 0;
                log_dropped = 0;
                event_count = 0;
                events_dropped = 0;
                run_start_us = now;
                last_status_us = 0;
                last_rssi_rpm_us = 0;
                if (boot_after_watchdog) {
                    push_event(now, 'W', 0, 0, receiver_get_channel(SWITCH_E));
                    boot_after_watchdog = false;
                }
            }
            // otherwise the bot went live again before saving, so keep extending the same run
            recording = true;
        }
        killed_since_us = 0;

        if (now - last_status_us >= LOGGER_STATUS_INTERVAL_US) {
            last_status_us = now;
            uint8_t throttle = (uint8_t)(receiver_get_percent_for_channel(LEFT_JOYSTICK_Y) * 100.0);
            uint16_t accel_rpm = bot_state->rpm > UINT16_MAX ? UINT16_MAX : (uint16_t)bot_state->rpm;
            push_record(now, 'A', throttle, accel_rpm);

            uint16_t drive_rpm = bot_state->drive_rpm > UINT16_MAX ? UINT16_MAX : (uint16_t)bot_state->drive_rpm;
            push_record(now, 'D', rssi_rpm_state(), drive_rpm);

            float corr = fminf(fmaxf(bot_state->rpm_correction, 0.0f), 2.55f);
            uint16_t corrected = bot_state->accel_rpm_corrected > UINT16_MAX ? UINT16_MAX : (uint16_t)bot_state->accel_rpm_corrected;
            push_record(now, 'C', (uint8_t)(corr * 100.0f + 0.5f), corrected);
        }
        if (now - last_rssi_rpm_us >= LOGGER_RSSI_RPM_INTERVAL_US) {
            last_rssi_rpm_us = now;
            float conf = fminf(fmaxf(bot_state->rssi_rpm_confidence, 0.0f), 2.55f);
            uint16_t rssi_rpm = bot_state->rssi_rpm > UINT16_MAX ? UINT16_MAX : (uint16_t)bot_state->rssi_rpm;
            push_record(now, 'P', (uint8_t)(conf * 100.0f + 0.5f), rssi_rpm);
        }
        return;
    }

    recording = false;

    // save only on a deliberate kill, never on a link failsafe: the save stalls the main loop
    // for a few seconds, which must not happen while the bot might be about to be driven again
    if (killed) {
        if (killed_since_us == 0) {
            killed_since_us = now;
        }
        if (have_unsaved && now - killed_since_us >= (uint64_t)LOGGER_SAVE_DELAY_MS * 1000) {
            save_to_flash();
        }
    } else {
        killed_since_us = 0;
    }

    handle_serial_command();
}

#endif
