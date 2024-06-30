#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "spi_comms.h"
#include "ADXL345.h"
#include <cstdint>
#include <cinttypes>
#include <bitset>
#include <iostream>

extern "C" void app_main(void) {
    /* Init Section */
    constexpr int MOSI_V = 23;
    constexpr int MISO_V = 19;
    constexpr int SCLK_V = 18;
    constexpr int CS_V   = 5;

    accspi::Spi comms;
    esp_err_t rc = ESP_OK;

    // SPI1 is internal (not accessible)
    // SPI2 is HSPI
    // SPI3 is VSPI

    rc |= comms.init(
            SPI3_HOST,
            MOSI_V,
            MISO_V,
            SCLK_V);
    rc |= comms.registerDevice(SPI_MODE, CS_V, 8, 8, 0, 100, 0, 0, 0);
    if (ESP_OK != rc) {
        ESP_LOGD("main",
                "In function %s:                                               \
                    Failed to initialize or register an spi device.",
                __func__);
    }

    /*
     * Verify that the read register, containing the device id,
     * holds the correct value
     */
    const uint8_t deviceId = comms.readRegister(REG_DEVID);
    if (DEVID != deviceId) {
        ESP_LOGE("main",
                "In function %s:                                               \
                    Register REG_DEVID [0x%02X] holds invalid data: 0x%02X     \
                    Should be 0x%02X",
                    __func__, REG_DEVID, deviceId, DEVID);
    } else {
        ESP_LOGI("main", "Register Device Id [0x%02X] == 0x%02X. OK!",
                REG_DEVID, deviceId);
    }
    // uint8_t curAccelDataX0{0};
    // uint8_t curAccelDataX1{0};
    // uint16_t curAccelDataX{0};
    uint8_t powerControl;
    std::bitset<8> b_powerControl;

    powerControl = comms.readRegister(REG_POWER_CTL);
    b_powerControl = powerControl;
    PrintWord(powerControl);
    std::cout << std::endl << b_powerControl << std::endl;

    // powerControl |= SET_MEASURE_BIT;
    // comms.writeRegister(REG_POWER_CTL, powerControl, 0);
    // powerControl = comms.readRegister(REG_POWER_CTL);
    // b_powerControl = powerControl;
    // std::cout << b_powerControl << std::endl;
    /* Set Measure bit, to place into measurement mode */


    /* Main program loop section */
    while (true) {
        // curAccelDataX0 = comms.readRegister(REG_DATA_X0);
        // curAccelDataX1 = comms.readRegister(REG_DATA_X1);
        // curAccelDataX = curAccelDataX1;
        // curAccelDataX <<= sizeof(curAccelDataX1);
        // curAccelDataX |= curAccelDataX0;

        // std::bitset<8> x0(curAccelDataX0);
        // std::bitset<8> x1(curAccelDataX1);
        // std::bitset<16> x(curAccelDataX);

        // ESP_LOGI("main", "H: [%hhu] L: [%hhu] M: [%hd]", curAccelDataX1, curAccelDataX0, curAccelDataX);
    }
}
