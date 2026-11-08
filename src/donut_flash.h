#ifndef DONUT_FLASH_H
#define DONUT_FLASH_H

#include "donut_config.h"
#include "hardware/flash.h"

// Shared helpers for writing persistent data to onboard flash (run logger, RPM correction curve).
// Offsets are from the start of flash. Erasing/programming stalls execute-in-place, so interrupts
// (including the DShot timer) are off for each call: ~50ms per 4KB sector erase. Only call these
// while the bot is killed. The watchdog is fed before every sector erase.

// false if [offset, offset + size) would overlap the program image
bool donut_flash_region_safe(uint32_t offset, uint32_t size);

void donut_flash_erase_sector(uint32_t offset);

// programs one FLASH_PAGE_SIZE (256 byte) page; the page must already be erased
void donut_flash_program_page(uint32_t offset, const uint8_t* page);

// memory-mapped read access
const void* donut_flash_read_ptr(uint32_t offset);

#endif
