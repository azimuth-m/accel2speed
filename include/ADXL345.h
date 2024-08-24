#pragma once

#include <cstdint>
#include <vector>

#include "esp_event.h"

#include "SpiComms.h"
#include "InputGpio.h"

/* SPI mode */
#define SPI_MODE        (0x03u)

/* Misc */
#define CMD_BITS        (0x02u)
#define ADR_BITS        (0x06u)
#define DEVID_VAL       (0xE5u)

enum REG {
    DEVID       = 0x00u,
    BW_RATE     = 0x2Cu,
    POWER_CTL   = 0x2Du,
    INT_ENABLE  = 0x2Eu, /* | (1ULL << 7) to enable DATA_READY int out */
    INT_MAP     = 0x2Fu, /* & ~(1ULL << 7) set data out to be int1 pin */
    DATA_FORMAT = 0x31u, /* Range settings: 2, 4, 8, 16 g*/
    DATA_X0     = 0x32u,
    DATA_X1     = 0x33u,
    DATA_Y0     = 0x34u,
    DATA_Y1     = 0x35u,
    DATA_Z0     = 0x36u,
    DATA_Z1     = 0x37u,
    FIFO_CTL    = 0x38u
};

enum POS {
    D0 = 0,
    D1 = 1,
    D2 = 2,
    D3 = 3,
    D4 = 4,
    D5 = 5,
    D6 = 6,
    D7 = 7
    };

/* SPI comminucation commands. Command bits are 2 */
enum CMD {
    WRITE       = 0b00u,
    WRITE_BURST = 0b01u,
    READ        = 0b10u,
    READ_BURST  = 0b11u
};

typedef enum INT_PIN {
    INT1 = 0,
    INT2 = 1
} IntPin_t;

typedef enum ACC_RANGE {
    G2  = 0b00,
    G4  = 0b01,
    G8  = 0b10,
    G16 = 0b11
} AccRange_t;

typedef enum ACC_BANDW {
    BW16,
    BW8,
    BW4
} AccBandwidth_t;

/* Acceleration data placeholder */
typedef struct AxisAccelerationData {
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
} AxisAccelerationData_t;

class Adxl345 {
private:

    InputGpio masterIntPin_;
    Spi comms_;
    const uint8_t busSpeedHz_;

    const AccRange_t range_;
    const AccBandwidth_t bandwidth_;
    uint32_t bandwidthHz_;

    AxisAccelerationData_t rawAccelData_;
    float currentVelocity_;

    static void s_DataReadyEventHandler_(
        void* handerArgs,
        esp_event_base_t base,
        int32_t id,
        void* eventData
    );

    int32_t WakeUp_();
    int32_t EnableInterrupt_(IntPin_t intPin);
    int32_t SetSensFullResolution_();
    int32_t SetSensGRange_();
    int32_t SetSensOutDataRate_();

    esp_err_t ReadDataIntoBuffer_(void* buffer, uint8_t byteCnt);

    float Q_rsqrt_(float number);
    float FastMagnitude_(float x, float y);
    float CalcVelocityRectangular_(float a, float v, float dt);

public:

    Adxl345(
        IntPin_t intPin,
        InputGpio masterIntPin,
        int masterCsPinNum,
        uint8_t spiMode,
        uint8_t commandBitCnt,
        uint8_t addressBitCnt,
        int busSpeedHz,
        Spi comms,
        AccRange_t gRange,
        AccBandwidth_t bandwidth);
};
