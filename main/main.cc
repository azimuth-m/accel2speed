#include <iostream>
#include <bitset>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "spi_comms.h"
#include "ADXL345.h"


extern "C" void app_main(void)
{
    constexpr int MOSI_V = 23;
    constexpr int MISO_V = 19;
    constexpr int SCLK_V = 18;
    constexpr int CS_V   = 5;

    accspi::Spi comms;

    // SPI1 is internal (not accessible)
    // SPI2 is HSPI
    // SPI3 is VSPI
    esp_err_t rc = 0;
    rc = comms.Init(
            SPI3_HOST,
            MOSI_V,
            MISO_V,
            SCLK_V);
    if (ESP_OK != rc) {
        // std::cerr << "Return code NOT OK: 0x" << std::hex << rc << std::endl;
    }

    comms.RegisterDevice(SPI_MODE, CS_V, 8, 8, 0, 100, 0, 0, 0);

    std::cout << "Return code after register: 0x" << std::hex << rc << std::endl;

    while (true) {
        uint8_t RECEIVED = comms.ReadRegister(REG_DEVID);
        // std::cout << "Reg 0xff: " << std::showbase << std::hex << RECEIVED << std::endl;
        printf("Reg REG_DEVID: 0x%02X\n", RECEIVED); // DEVID CONTENT SHOULD BE 0xE5
        // ESP_LOGI();
    }

}
