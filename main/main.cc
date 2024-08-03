#include <cstdint>
#include <cinttypes>
#include <bitset>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "spi_comms.h"
#include "adxl345.h"
#include "InputGpio.h"
#include "OutputGpio.h"


static void DataReadyEventHandler(
        void* handler_args,
        esp_event_base_t base,
        int32_t id,
        void* event_data) {

    std::cout << ".: ENTERING INTERRUPT SUBROUTINE :." << std::endl;
    std::cout << "id: " << id << std::endl;
    std::cout << "base: " << base << std::endl;

    std::cout << ".: EXITING INTERRUPT SUBROUTINE :." << std::endl;

}

extern "C" void app_main(void) {

    /* Init Section */
    constexpr int PIN_MOSI_V = 23;
    constexpr int PIN_MISO_V = 19;
    constexpr int PIN_SCLK_V = 18;
    constexpr int PIN_CS_V   = 5;
    constexpr gpio_num_t PIN_DATA_READY = GPIO_NUM_21;

    // accspi::Spi comms;
    // esp_err_t rc = ESP_OK;

    // SPI1 is internal (not accessible)
    // SPI2 is HSPI
    // SPI3 is VSPI

    // rc |= comms.Init(
    //         SPI3_HOST,
    //         PIN_MOSI_V,
    //         PIN_MISO_V,
    //         PIN_SCLK_V);
    // rc |= comms.RegisterDevice(SPI_MODE, PIN_CS_V, CMD_BITS, ADR_BITS, 0, 100, 0, 0, 0);
    // if (ESP_OK != rc) {
    //     ESP_LOGD("main",
    //             "In function %s:
    //                 Failed to initialize or register an spi device.",
    //             __func__);
    // }

    /*
     * Verify that the read register, containing the device id,
     * holds the correct value
     */
    // std::bitset<8> asd;

    // asd = comms.readRegister(REG_DEVID | (1u << 7));
    // std::cout << "Reg devid: " << asd << std::endl;

    // asd = comms.readRegister(REG_POWER_CTL | (1u << 7));
    // std::cout << "Reg power control: " << asd << std::endl;

    // comms.writeRegister(0x2c, 0x8c);
    // powerControl = comms.readRegister(REG_POWER_CTL | (1u << 7));
    // printf("Reg power control: %x\n", powerControl);

    // asd = comms.readRegister(REG_DEVID , 2);
    // std::cout << "Reg devid: " << asd << std::endl;

    // asd = comms.ReadRegister(REG_POWER_CTL, CMD_READ);
    // std::cout << "Reg rate: " << asd << std::endl;
    // asd = comms.ReadRegister(0x2d, CMD_READ);
    // std::cout << "Reg power control: " << asd << std::endl;

    // comms.WriteRegister(0x2c, (1u << 3), CMD_WRITE);
    // comms.WriteRegister(0x2d, (1u << 3), CMD_WRITE);

    // asd = comms.ReadRegister(REG_POWER_CTL , CMD_READ);
    // std::cout << "rate: " << asd << std::endl;
    // asd = comms.ReadRegister(0x2D , CMD_READ);
    // std::cout << "Reg power control: " << asd << std::endl;

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

    /* Int handling */
    accgpio::InputGpio dataRdyPin(
            PIN_DATA_READY,
            GPIO_PULLUP_DISABLE,
            GPIO_PULLDOWN_ENABLE);

    auto rc = ESP_OK;
    rc |= esp_event_loop_create_default(); /* TODO: DO NOT FORGET TO CALL THIS */
    rc |= dataRdyPin.EnableInterrupt(GPIO_INTR_POSEDGE);
    rc |= dataRdyPin.SetEventHandler(&DataReadyEventHandler);
    if (ESP_OK != rc) {
        ESP_LOGE("main", "In function %s: Failed to setup interrupts",
                __func__);
    }

    /* TODO: Warning: Using a mechanical switch would require in implementing
     * a debouncer (using schmidt trigger, caps, latches, or sw debouncing).
     * Should not interfere with the int receievd by an actual ic.
     */

    /* Main program loop section */
    while (true) {
        // std::bitset<16> xdata;
        // xdata = comms.ReadRegister(REG_DATA_X0 , CMD_READ);
        // xdata <<= 8;
        // xdata |= comms.ReadRegister(REG_DATA_X1 , CMD_READ);
        // std::cout << "xdata: " << xdata << std::endl;

    // std::cout << "PIN 4 STATE: " << pin4.Read() << std::endl;
    }
}
