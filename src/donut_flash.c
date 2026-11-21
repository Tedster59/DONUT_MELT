#include "donut_flash.h"
#include "hardware/sync.h"

extern char __flash_binary_end; // provided by the SDK linker script

bool donut_flash_region_safe(uint32_t offset, uint32_t size) {
    (void)size;
    return (uintptr_t)&__flash_binary_end <= XIP_BASE + offset && offset % FLASH_SECTOR_SIZE == 0;
}

// Interrupts are off for each operation, which would break the DShot stream into garbage, so the
// motor outputs are stopped first (the main loop restarts them once the writing is over), and every
// write starts the input lockout (donut_note_loop_blocked): the bot can't go live again until the
// receiver has been healthy for a while and the killswitch has been seen killed.
void donut_flash_erase_sector(uint32_t offset) {
    watchdog_update();
    motor_outputs_suspend();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    donut_note_loop_blocked();
}

void donut_flash_program_page(uint32_t offset, const uint8_t* page) {
    motor_outputs_suspend();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(offset, page, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    donut_note_loop_blocked();
}

const void* donut_flash_read_ptr(uint32_t offset) {
    return (const void*)(XIP_BASE + offset);
}
