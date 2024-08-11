#include <cstdint>
#include <cinttypes>
#include <bitset>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "SpiComms.h"
#include "Adxl345.h"
#include "InputGpio.h"
#include "OutputGpio.h"


extern "C" void app_main(void) {

    /* Init Section */
    constexpr int PIN_MOSI_V     = 23;
    constexpr int PIN_MISO_V     = 19;
    constexpr int PIN_SCLK_V     = 18;
    constexpr int PIN_CS_V       = 5;
    constexpr gpio_num_t PIN_DATA_READY = GPIO_NUM_21;

    Adxl345 accMeter(
            INT_PIN::INT1,
            InputGpio(
                PIN_DATA_READY,
                GPIO_PULLUP_DISABLE,
                GPIO_PULLDOWN_ENABLE),
            PIN_CS_V,
            SPI_MODE, CMD_BITS, ADR_BITS, 5e+6,
            Spi(SPI3_HOST, PIN_MOSI_V, PIN_MISO_V, PIN_SCLK_V));

    // SPI1 is internal (not accessible)
    // SPI2 is HSPI
    // SPI3 is VSPI

    while (true) {
    }
}
