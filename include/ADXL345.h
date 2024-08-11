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

    AxisAccelerationData_t rawAccelData_;

    static constexpr uint32_t bufferLen_ = 0x10;
    std::vector<float> accelMagnitudeXY_;

    const uint8_t busSpeedHz_;
    const float sampingRateS_ = 1.0 / busSpeedHz_;
    const float initialVelocity_;
    float currentVelocity_;

    static void s_DataReadyEventHandler_(
        void* handerArgs,
        esp_event_base_t base,
        int32_t id,
        void* eventData
    );

    int32_t WakeUp_();
    int32_t EnableInterrupt_(IntPin_t intPin);
    int32_t SetFullResolution_();
    int32_t SetGRange_(AccRange_t range);
    int32_t SetDataRate_();

public:
    Adxl345(
        const IntPin_t intPin,
        const InputGpio masterIntPin,
        const int masterCsPinNum,
        const uint8_t spiMode,
        const uint8_t commandBitCnt,
        const uint8_t addressBitCnt,
        const int busSpeedHz,
        const Spi comms);

    esp_err_t ReadDataIntoBuffer(void* buffer, uint8_t byteCnt);

    float Q_rsqrt(float number);
    float FastMagnitude(float x, float y);

    float CalcVelocityTrapezoidal(
        const std::vector<float>& a,
        double dt);
    float CalcVelocityRectangular(float a, float v, float dt);

};
