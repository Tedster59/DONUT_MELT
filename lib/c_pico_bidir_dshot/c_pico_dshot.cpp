#include "c_pico_dshot.h"

BidirDshotX1TelemetryType bidirDshotTelemetryTypeToBidirDshotX1TelemetryType(BidirDshotTelemetryType bidirDshotTelemetryType){
    return static_cast<BidirDshotX1TelemetryType>(bidirDshotTelemetryType);
}

BidirDshotTelemetryType bidirDshotX1TelemetryTypeToBidirDshotTelemetryType(BidirDshotX1TelemetryType bidirDshotX1TelemetryType){
    return static_cast<BidirDshotTelemetryType>(bidirDshotX1TelemetryType);
}

extern "C" {
    BidirDShotX1* BidirDShotX1_constructor(uint8_t pin, uint32_t speed, PIO pio, int8_t sm){
        return new BidirDShotX1(pin, speed, pio, sm);
    }

    void BidirDShotX1_destructor(BidirDShotX1* bidirDShotX1){
        delete bidirDShotX1;
    }

    void BidirDShotX1_sendThrottle(BidirDShotX1* bidirDShotX1, uint16_t throttle){
        bidirDShotX1->sendThrottle(throttle);
    }

    void BidirDShotX1_sendRaw11Bit(BidirDShotX1* bidirDShotX1, uint16_t data){
        bidirDShotX1->sendRaw11Bit(data);
    }

    void BidirDShotX1_sendRaw12Bit(BidirDShotX1* bidirDShotX1, uint16_t data){
        bidirDShotX1->sendRaw12Bit(data);
    }

    bool BidirDShotX1_checkTelemetryAvailable(BidirDShotX1* bidirDShotX1){
        return bidirDShotX1->checkTelemetryAvailable();
    }

    BidirDshotX1TelemetryType BidirDShotX1_getTelemetryErpm(BidirDShotX1* bidirDShotX1, uint32_t *erpm){
        return bidirDshotTelemetryTypeToBidirDshotX1TelemetryType(bidirDShotX1->getTelemetryErpm(erpm));
    }

    BidirDshotX1TelemetryType BidirDShotX1_getTelemetryPacket(BidirDShotX1* bidirDShotX1, uint32_t *value){
        return bidirDshotTelemetryTypeToBidirDshotX1TelemetryType(bidirDShotX1->getTelemetryPacket(value));
    }

    BidirDshotX1TelemetryType BidirDShotX1_getTelemetryRaw(BidirDShotX1* bidirDShotX1, uint32_t *value){
        return bidirDshotTelemetryTypeToBidirDshotX1TelemetryType(bidirDShotX1->getTelemetryRaw(value));
    }

    uint32_t BidirDShotX1_convertFromRaw(BidirDShotX1* bidirDShotX1, uint32_t raw, BidirDshotX1TelemetryType type){
        return bidirDShotX1->convertFromRaw(raw, bidirDshotX1TelemetryTypeToBidirDshotTelemetryType(type));
    }

    bool BidirDShotX1_initError(BidirDShotX1* bidirDShotX1){
        return bidirDShotX1->initError();
    }

    void c_pico_dshot_is_library_accesible(){
        printf("Accessing the c_pico_dshot library is functioning properly! \n");
    }

}
