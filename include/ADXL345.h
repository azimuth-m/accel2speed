#pragma once

#include <cstdint>
#include <atomic>

#include "esp_event.h"

#include "SpiComms.h"

/* SPI mode */
#define SPI_MODE        (0x03u)

/* Misc */
#define CMD_BITS        (0x02u)
#define ADR_BITS        (0x06u)
// #define TRANS_BITS      (CMD_BITS + ADR_BITS)

/* SPI comminucation commands. Command bits are 2 */
#define CMD_WRITE       (0b00u)
#define CMD_WRITE_BURST (0b01u)
#define CMD_READ        (0b10u)
#define CMD_READ_BURST  (0b11u)

/* Slave addresses */
#define REG_DEVID       (0x00u)
#define REG_BW_RATE     (0x2Cu)
#define REG_POWER_CTL   (0x2Du)
#define REG_INT_ENABLE  (0x2Eu) /* | (1ULL << 7) to enable DATA_READY int out */
#define REG_INT_MAP     (0x2Fu) /* & ~(1ULL << 7) set data out to be int1 pin */
#define REG_FIFO_CTL    (0x38u)

#define REG_DATA_X0     (0x32u)
#define REG_DATA_X1     (0x33u)
#define REG_DATA_Y0     (0x34u)
#define REG_DATA_Y1     (0x35u)
#define REG_DATA_Z0     (0x36u)
#define REG_DATA_Z1     (0x37u)

/* Device ID */
#define DEVID               (0xE5u)

/* Acceleration data placeholder */
typedef struct AxisAccelerationData {
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
} AxisAccelerationData_t;

class Adxl345 {
private:
    accspi::Spi comms_;

    static constexpr uint32_t bufferLen_ = 0x10;

    /* TODO: Add low pass filter */
    // AxisAccelerationData_t rawAccelData_[bufferLen_];
    AxisAccelerationData_t rawAccelData_;
    AxisAccelerationData_t averagedAccel_;

    const uint8_t initVelocity_;
    const uint8_t curVelocity_;

    int16_t magAccel_;

    std::atomic<bool> processingData_;

    static void DataReadyEventHandler(
        void* handerArgs,
        esp_event_base_t base,
        int32_t id,
        void* eventData
    );

public:



};
