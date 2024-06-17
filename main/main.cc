#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "spi_comms.h"
#include "ADXL345.h"


extern "C" void app_main(void) {
    /* Init Section */
    constexpr int MOSI_V = 23;
    constexpr int MISO_V = 19;
    constexpr int SCLK_V = 18;
    constexpr int CS_V   = 5;

    accspi::Spi comms;

    // SPI1 is internal (not accessible)
    // SPI2 is HSPI
    // SPI3 is VSPI

    esp_err_t rc = ESP_OK;
    rc |= comms.Init(
            SPI3_HOST,
            MOSI_V,
            MISO_V,
            SCLK_V);
    if (ESP_OK != rc) {
        ESP_LOGD(
                "main",
                "In function %s:                                                                    \
                    Failed to initialize spi communications. Return code: %d",
                 __func__, rc); // DEVID CONTENT SHOULD BE 0xE5
    }

    comms.RegisterDevice(SPI_MODE, CS_V, 8, 8, 0, 100, 0, 0, 0);

    uint8_t RECEIVED = comms.ReadRegister(REG_DEVID);
    ESP_LOGD("main", "Reg REG_DEVID: 0x%02X", RECEIVED); // DEVID CONTENT SHOULD BE 0xE5

    /* Main program loop section */
    while (true) {
    }

}
