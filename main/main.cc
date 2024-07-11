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
    rc |= comms.registerDevice(SPI_MODE, CS_V, CMD_BITS, ADR_BITS, 0, 100, 0, 0, 0);
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
    std::bitset<8> asd;

    // asd = comms.readRegister(REG_DEVID | (1u << 7));
    // std::cout << "Reg devid: " << asd << std::endl;

    // asd = comms.readRegister(REG_POWER_CTL | (1u << 7));
    // std::cout << "Reg power control: " << asd << std::endl;

    // comms.writeRegister(0x2c, 0x8c);
    // powerControl = comms.readRegister(REG_POWER_CTL | (1u << 7));
    // printf("Reg power control: %x\n", powerControl);

    // asd = comms.readRegister(REG_DEVID , 2);
    // std::cout << "Reg devid: " << asd << std::endl;

    asd = comms.readRegister(REG_POWER_CTL , CMD_READ);
    std::cout << "Reg rate: " << asd << std::endl;
    asd = comms.readRegister(0x2d, CMD_READ);
    std::cout << "Reg power control: " << asd << std::endl;

    comms.writeRegister(0x2c, (1u << 3), CMD_WRITE);
    comms.writeRegister(0x2d, (1u << 3), CMD_WRITE);
    
    asd = comms.readRegister(REG_POWER_CTL , CMD_READ);
    std::cout << "rate: " << asd << std::endl;
    asd = comms.readRegister(0x2D , CMD_READ);
    std::cout << "Reg power control: " << asd << std::endl;

    // if (DEVID != deviceId) {
    //     ESP_LOGE("main",
    //             "In function %s:
    //                 Register REG_DEVID [0x%02X] holds invalid data: 0x%02X
    //                 Should be 0x%02X",
    //                 __func__, REG_DEVID, deviceId, DEVID);
    // } else {
    //     ESP_LOGI("main", "Register Device Id [0x%02X] == 0x%02X. OK!",
    //             REG_DEVID, deviceId);
    // }

    /* Main program loop section */
    while (true) {
        std::bitset<16> xdata;
        xdata = comms.readRegister(REG_DATA_X0 , CMD_READ);
        xdata <<= 8;
        xdata |= comms.readRegister(REG_DATA_X1 , CMD_READ);
        std::cout << "xdata: " << xdata << std::endl;
    }
}
