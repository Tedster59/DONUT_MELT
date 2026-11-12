#ifndef C_PICO_DSHOT_H
#define C_PICO_DSHOT_H

// Used in header file and c file
#include <stdint.h>
#include <stdio.h>
#include "hardware/pio.h"
#include "bidir_dshot_x1.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BidirDShotX1 BidirDShotX1;

typedef enum {
    BIDIR_ERPM,
	BIDIR_OTHER_VALUE,
	BIDIR_CHECKSUM_ERROR,
	BIDIR_NO_PACKET,
	BIDIR_VOLTAGE,
	BIDIR_CURRENT,
	BIDIR_TEMPERATURE,
	BIDIR_STATUS,
	BIDIR_STRESS,
	BIDIR_DEBUG_FRAME_1,
	BIDIR_DEBUG_FRAME_2,
} BidirDshotX1TelemetryType;

BidirDShotX1* BidirDShotX1_constructor(uint8_t pin, uint32_t speed, PIO pio, int8_t sm);

void BidirDShotX1_destructor(BidirDShotX1* bidirDShotX1);

void BidirDShotX1_sendThrottle(BidirDShotX1* bidirDShotX1, uint16_t throttle);

void BidirDShotX1_sendRaw11Bit(BidirDShotX1* bidirDShotX1, uint16_t data);

void BidirDShotX1_sendRaw12Bit(BidirDShotX1* bidirDShotX1, uint16_t data);

bool BidirDShotX1_checkTelemetryAvailable(BidirDShotX1* bidirDShotX1);

BidirDshotX1TelemetryType BidirDShotX1_getTelemetryErpm(BidirDShotX1* bidirDShotX1, uint32_t *erpm);

BidirDshotX1TelemetryType BidirDShotX1_getTelemetryPacket(BidirDShotX1* bidirDShotX1, uint32_t *value);

BidirDshotX1TelemetryType BidirDShotX1_getTelemetryRaw(BidirDShotX1* bidirDShotX1, uint32_t *value);

uint32_t BidirDShotX1_convertFromRaw(BidirDShotX1* bidirDShotX1, uint32_t raw, BidirDshotX1TelemetryType type);

bool BidirDShotX1_initError(BidirDShotX1* bidirDShotX1);

void c_pico_dshot_is_library_accesible();

#ifdef __cplusplus
}
#endif

#endif