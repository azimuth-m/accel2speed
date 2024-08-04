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

    accspi::Spi comms;
    esp_err_t rc = ESP_OK;

    // SPI1 is internal (not accessible)
    // SPI2 is HSPI
    // SPI3 is VSPI

    rc |= comms.Init(
            SPI3_HOST,
            PIN_MOSI_V,
            PIN_MISO_V,
            PIN_SCLK_V);
    rc |= comms.RegisterDevice(SPI_MODE, PIN_CS_V, CMD_BITS, ADR_BITS, 0, 100, 0, 0, 0);
    if (ESP_OK != rc) {
        ESP_LOGD("main",
                "In function %s: Failed to initialize or register an spi device.",
                __func__);
    }

    std::bitset<8> asd;

    asd = comms.ReadRegister(REG_DEVID, CMD_READ);
    std::cout << "Reg devid: " << asd << std::endl;


    // comms.writeRegister(0x2c, 0x8c);
    // powerControl = comms.readRegister(REG_POWER_CTL | (1u << 7));
    // printf("Reg power control: %x\n", powerControl);

    // asd = comms.readRegister(REG_DEVID , 2);
    // std::cout << "Reg devid: " << asd << std::endl;

    // asd = comms.ReadRegister(REG_POWER_CTL, CMD_READ);
    // std::cout << "Reg rate: " << asd << std::endl;
    // asd = comms.ReadRegister(0x2d, CMD_READ);
    // std::cout << "Reg power control: " << asd << std::endl;

    /* Set device to measure mode */
    comms.WriteRegister(REG_POWER_CTL, (1u << 3), CMD_WRITE);

    /* Turn off LOW POWER */
    comms.WriteRegister(REG_BW_RATE, (1u << 3), CMD_WRITE);

    asd = comms.ReadRegister(0x31, CMD_READ);
    std::cout << "Reg DATA_FORMAT (0x31): " << asd << std::endl;

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
    // accgpio::InputGpio dataRdyPin(
    //         PIN_DATA_READY,
    //         GPIO_PULLUP_DISABLE,
    //         GPIO_PULLDOWN_ENABLE);

    // rc |= esp_event_loop_create_default(); /* TODO: DO NOT FORGET TO CALL THIS */
    // rc |= dataRdyPin.EnableInterrupt(GPIO_INTR_POSEDGE);
    // rc |= dataRdyPin.SetEventHandler(&DataReadyEventHandler);
    // if (ESP_OK != rc) {
    //     ESP_LOGE("main", "In function %s: Failed to setup interrupts",
    //             __func__);
    // }

    /* TODO: Warning: Using a mechanical switch would require in implementing
     * a debouncer (using schmidt trigger, caps, latches, or sw debouncing).
     * Should not interfere with the int receievd by an actual ic.
     */


    uint8_t allDataBuffer[6];
    int16_t x_raw_data;
    int16_t y_raw_data;
    int16_t z_raw_data;

    while (true) {
        comms.ReadRegisterMultipleBytes(REG_DATA_X0, allDataBuffer, sizeof(allDataBuffer), CMD_READ_BURST);
        x_raw_data = (int16_t) ((allDataBuffer[1] << 8) + allDataBuffer[0]);
        y_raw_data = (int16_t) ((allDataBuffer[3] << 8) + allDataBuffer[2]);
        z_raw_data = (int16_t) ((allDataBuffer[5] << 8) + allDataBuffer[4]);

        std::cout <<
                "X: " <<
                    (x_raw_data) * 0.00339F << " " <<
                "Y: " <<
                    (y_raw_data) * 0.00339F << " " <<
                "Z: " <<
                    (z_raw_data) * 0.0043F <<
        std::endl;

    // std::cout << "PIN 4 STATE: " << pin4.Read() << std::endl;
    }
}
