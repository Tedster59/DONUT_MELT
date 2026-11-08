#include "donut_rpm_curve.h"

#include <stddef.h>
#include <string.h>
#include "donut_flash.h"

#define CURVE_MAGIC 0x56524352u // "RCRV"
#define CURVE_VERSION 1

typedef struct {
    float rpm;    // raw (uncorrected) accelerometer rpm
    float factor; // correction factor at that rpm
} curve_point_t;

// stored in the first page of the RPM_CURVE_FLASH_OFFSET sector
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    curve_point_t points[RPM_CURVE_MAX_POINTS];
    uint32_t checksum; // FNV-1a over everything above
} curve_record_t;

_Static_assert(sizeof(curve_record_t) <= FLASH_PAGE_SIZE, "curve record must fit in one flash page");
_Static_assert(RPM_CURVE_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0, "RPM_CURVE_FLASH_OFFSET must be sector aligned");

static curve_point_t points[RPM_CURVE_MAX_POINTS]; // sorted by ascending rpm
static uint8_t count = 0;
static double trim = 1.0;

static bool flash_ok = false;
static bool dirty = false;          // RAM curve differs from flash
static uint64_t killed_since_us = 0;

static bool button_armed = false; // button seen released in real (non-failsafe) channel data
static bool button_held = false;
static bool press_started_killed = false;
static bool clear_done = false;
static uint64_t press_start_us = 0;

static double clamp_factor(double f) {
    return fmin(fmax(f, RPM_CORRECTION_MIN), RPM_CORRECTION_MAX);
}

static uint32_t record_checksum(const curve_record_t* r) {
    const uint8_t* bytes = (const uint8_t*)r;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < offsetof(curve_record_t, checksum); i++) {
        h = (h ^ bytes[i]) * 16777619u;
    }
    return h;
}

// linear interpolation between points, flat beyond the ends, 1.0 with no points
static double curve_at(double rpm) {
    if (count == 0) {
        return 1.0;
    }
    if (rpm <= points[0].rpm) {
        return points[0].factor;
    }
    if (rpm >= points[count - 1].rpm) {
        return points[count - 1].factor;
    }
    for (uint8_t i = 1; i < count; i++) {
        if (rpm <= points[i].rpm) {
            double span = points[i].rpm - points[i - 1].rpm;
            if (span <= 0) {
                return points[i].factor;
            }
            double t = (rpm - points[i - 1].rpm) / span;
            return points[i - 1].factor + t * (points[i].factor - points[i - 1].factor);
        }
    }
    return points[count - 1].factor; // unreachable
}

static void sort_points(void) {
    for (uint8_t i = 1; i < count; i++) {
        curve_point_t p = points[i];
        int j = i - 1;
        while (j >= 0 && points[j].rpm > p.rpm) {
            points[j + 1] = points[j];
            j--;
        }
        points[j + 1] = p;
    }
}

void rpm_curve_init(void) {
    count = 0;
    trim = 1.0;
    dirty = false;

    flash_ok = donut_flash_region_safe(RPM_CURVE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    if (!flash_ok) {
        printf("CURVE flash region overlaps firmware, curve will not be saved\n");
        return;
    }

    const curve_record_t* r = (const curve_record_t*)donut_flash_read_ptr(RPM_CURVE_FLASH_OFFSET);
    if (r->magic != CURVE_MAGIC || r->version != CURVE_VERSION || r->count > RPM_CURVE_MAX_POINTS ||
        r->checksum != record_checksum(r)) {
        return; // nothing saved yet (or stale/corrupt): start from the default
    }
    for (uint32_t i = 0; i < r->count; i++) {
        const curve_point_t* p = &r->points[i];
        bool sane = isfinite(p->rpm) && isfinite(p->factor) && p->rpm > 0 &&
                    p->factor >= RPM_CORRECTION_MIN && p->factor <= RPM_CORRECTION_MAX &&
                    (i == 0 || p->rpm >= r->points[i - 1].rpm);
        if (!sane) {
            return; // reject the whole curve rather than use part of it
        }
    }
    memcpy(points, r->points, r->count * sizeof(curve_point_t));
    count = (uint8_t)r->count;
}

double rpm_curve_factor(double raw_rpm) {
    return clamp_factor(curve_at(raw_rpm) * trim);
}

void rpm_curve_adjust_trim(double stick, uint64_t dt_us) {
    if (fabs(stick) < RPM_TRIM_DEADZONE) {
        return;
    }
    // time-based so the trim speed doesn't depend on main loop speed (the old per-loop step got
    // ~10x faster when the loop sped up); capped so a stale timestamp can't cause a big jump
    if (dt_us > 50000) {
        dt_us = 50000;
    }
    double step = RPM_TRIM_RATE_PER_SEC * (dt_us / 1000000.0);
    trim += (stick > 0) ? step : -step;
    trim = clamp_factor(trim);
}

static void save_point(double raw_rpm) {
    double value = rpm_curve_factor(raw_rpm);

    int nearest = -1;
    double nearest_dist = 0;
    for (uint8_t i = 0; i < count; i++) {
        double d = fabs(points[i].rpm - raw_rpm);
        if (nearest < 0 || d < nearest_dist) {
            nearest = i;
            nearest_dist = d;
        }
    }

    curve_point_t p = { .rpm = (float)raw_rpm, .factor = (float)value };
    if (nearest >= 0 && nearest_dist <= raw_rpm * RPM_CURVE_MERGE_FRACTION) {
        points[nearest] = p;          // close to an existing point: update it
    } else if (count < RPM_CURVE_MAX_POINTS) {
        points[count++] = p;          // new point
    } else {
        points[nearest] = p;          // full: replace the nearest point
    }
    sort_points();

    // the saved point now holds curve*trim at this rpm, so resetting trim keeps the factor unchanged
    trim = 1.0;
    dirty = true;
    printf("CURVE saved point %u rpm -> %.3f (%u points)\n", (unsigned)raw_rpm, value, count);
}

static void clear_curve(void) {
    count = 0;
    trim = 1.0;
    dirty = true;
    printf("CURVE cleared\n");
}

static void persist(void) {
    curve_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = CURVE_MAGIC;
    rec.version = CURVE_VERSION;
    rec.count = count;
    memcpy(rec.points, points, count * sizeof(curve_point_t));
    rec.checksum = record_checksum(&rec);

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &rec, sizeof(rec));

    donut_flash_erase_sector(RPM_CURVE_FLASH_OFFSET);
    donut_flash_program_page(RPM_CURVE_FLASH_OFFSET, page);
    dirty = false;
    printf("CURVE written to flash (%u points)\n", count);
}

void rpm_curve_update(bot_state_t* bot_state, bool live, bool killed) {
    uint64_t now = time_us_64();

    // momentary switch with hysteresis; an unmapped channel sits at center and never triggers.
    // Only trust real channel data (not failsafed), and only accept a press after the button has
    // been seen released: if the button's rest position ever read high, the first frame after boot
    // would otherwise look like a press, and since the bot boots killed, a 3 s "hold" would wipe
    // the saved curve on every power-up.
    uint16_t ch = receiver_get_channel(RPM_CURVE_SAVE_SWITCH);
    if (bot_state->is_failsafed) {
        button_armed = false;
        button_held = false;
    } else if (!button_armed) {
        button_armed = ch < RECEIVER_MIDDLEST_CHANNEL_VALUE + 200;
    } else if (!button_held && ch > RECEIVER_MIDDLEST_CHANNEL_VALUE + 400) {
        button_held = true;
        press_start_us = now;
        press_started_killed = killed;
        clear_done = false;
        if (live && bot_state->rpm >= RPM_CURVE_MIN_SAVE_RPM) {
            save_point(bot_state->rpm);
        }
    } else if (button_held && ch < RECEIVER_MIDDLEST_CHANNEL_VALUE + 200) {
        button_held = false;
    }

    // hold while killed (for the whole hold) to clear
    if (button_held && press_started_killed && killed && !clear_done &&
        now - press_start_us >= (uint64_t)RPM_CURVE_CLEAR_HOLD_MS * 1000) {
        clear_curve();
        clear_done = true;
    }

    // write to flash only while killed: erasing stalls the loop (~50ms)
    if (killed) {
        if (killed_since_us == 0) {
            killed_since_us = now;
        }
        if (dirty && flash_ok && now - killed_since_us >= (uint64_t)RPM_CURVE_PERSIST_DELAY_MS * 1000) {
            persist();
        }
    } else {
        killed_since_us = 0;
    }
}

uint8_t rpm_curve_point_count(void) {
    return count;
}

double rpm_curve_trim(void) {
    return trim;
}
