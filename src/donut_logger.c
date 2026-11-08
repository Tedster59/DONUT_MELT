#include "donut_logger.h"

#ifdef USE_FLASH_LOGGER

#include <string.h>
#include "donut_flash.h"

#define LOG_MAGIC 0x474F4C44u // "DLOG"
#define LOG_VERSION 1

typedef struct __attribute__((packed)) {
    uint32_t t_us;  // microseconds since the run started
    char kind;      // 'R' link stats sample, 'A' accel/throttle snapshot, 'P' RSSI RPM estimate
    uint8_t a;      // R: rssi (-dBm)        A: throttle %      P: confidence * 100
    uint16_t b;     // R: link quality %     A: accel rpm       P: rssi rpm
} log_record_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t record_size;
    uint32_t record_count;
    uint32_t dropped;      // oldest records overwritten because the RAM buffer filled
} log_header_t;

#define LOG_RECORD_SIZE sizeof(log_record_t)
#define LOG_MAX_RECORDS (LOGGER_BUF_BYTES / LOG_RECORD_SIZE)
#define LOG_DATA_REGION_BYTES (((LOGGER_BUF_BYTES) + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE * FLASH_SECTOR_SIZE)
// layout: one header sector, then the records in chronological order
#define LOG_HEADER_OFFSET (LOGGER_FLASH_OFFSET)
#define LOG_DATA_OFFSET (LOGGER_FLASH_OFFSET + FLASH_SECTOR_SIZE)

_Static_assert(sizeof(log_record_t) == 8, "log record must stay 8 bytes");
_Static_assert(FLASH_PAGE_SIZE % sizeof(log_record_t) == 0, "records must not straddle flash pages");
_Static_assert(LOGGER_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0, "LOGGER_FLASH_OFFSET must be sector aligned");


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

static const log_header_t* saved_header(void) {
    return (const log_header_t*)donut_flash_read_ptr(LOG_HEADER_OFFSET);
}

static bool saved_log_valid(void) {
    const log_header_t* h = saved_header();
    return h->magic == LOG_MAGIC && h->version == LOG_VERSION &&
           h->record_size == LOG_RECORD_SIZE && h->record_count <= LOG_MAX_RECORDS;
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
}

void logger_log_rssi(uint64_t timestamp_us, uint8_t rssi, uint8_t link_quality) {
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

    log_header_t header = {
        .magic = LOG_MAGIC,
        .version = LOG_VERSION,
        .record_size = LOG_RECORD_SIZE,
        .record_count = log_count,
        .dropped = log_dropped,
    };
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &header, sizeof(header));
    donut_flash_program_page(LOG_HEADER_OFFSET, page);

    have_unsaved = false;
    printf("LOGGER saved %lu records (%lu dropped) in %lu ms\n",
        (unsigned long)log_count, (unsigned long)log_dropped, (unsigned long)((time_us_64() - start_us) / 1000));
}

// ---------- USB serial commands ----------

static void dump_saved_log(void) {
    if (!saved_log_valid()) {
        printf("LOGGER no saved run\n");
        return;
    }
    const log_header_t* h = saved_header();
    const log_record_t* records = (const log_record_t*)donut_flash_read_ptr(LOG_DATA_OFFSET);

    printf("# DONUT_LOG v%lu records=%lu dropped=%lu\n",
        (unsigned long)h->version, (unsigned long)h->record_count, (unsigned long)h->dropped);
    printf("kind,t_us,rssi,lq,throttle_pct,accel_rpm,rssi_rpm,confidence\n");
    for (uint32_t i = 0; i < h->record_count; i++) {
        const log_record_t* r = &records[i];
        switch (r->kind) {
            case 'R': printf("R,%lu,%u,%u,,,,\n", (unsigned long)r->t_us, r->a, r->b); break;
            case 'A': printf("A,%lu,,,%u,%u,,\n", (unsigned long)r->t_us, r->a, r->b); break;
            case 'P': printf("P,%lu,,,,,%u,%.2f\n", (unsigned long)r->t_us, r->b, r->a / 100.0f); break;
            default: break;
        }
        if ((i & 255) == 0) {
            watchdog_update();
        }
    }
    printf("# END\n");
}

static void print_status(void) {
    printf("LOGGER RAM: %lu records (%lu dropped), %s | flash: ",
        (unsigned long)log_count, (unsigned long)log_dropped, have_unsaved ? "unsaved" : "saved");
    if (saved_log_valid()) {
        printf("%lu records\n", (unsigned long)saved_header()->record_count);
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

    if (live) {
        if (!recording) {
            if (!have_unsaved) {
                // previous run was saved (or none yet): start a new one
                log_head = 0;
                log_count = 0;
                log_dropped = 0;
                run_start_us = now;
                last_status_us = 0;
                last_rssi_rpm_us = 0;
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
