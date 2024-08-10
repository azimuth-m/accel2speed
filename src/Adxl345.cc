#include <iostream>
#include "esp_err.h"
#include "esp_log.h"

#include "Adxl345.h"
#include "InputGpio.h"

void Adxl345::s_DataReadyEventHandler_(
        void* handler_args,
        esp_event_base_t base,
        int32_t id,
        void* event_data) {
    std::cout << "ENTERING ISR" << std::endl;
    std::cout << "PIN: " << id << std::endl;
    std::cout << "EXITING ISR" << std::endl;
}

Adxl345::Adxl345(
        const intPin_t intPin,
        const InputGpio masterIntPin,
        const int masterCsPinNum,
        const uint8_t spiMode,
        const uint8_t commandBitCnt,
        const uint8_t addressBitCnt,
        const int busSpeedHz,
        const Spi comms)
    : masterIntPin_(masterIntPin),
      comms_(comms),
      initialVelocity_(0) {

    comms_.RegisterDevice(
            spiMode, masterCsPinNum, commandBitCnt, addressBitCnt, 0, busSpeedHz, 0, 0, 0);
    WakeUp_();
    EnableInterrupt_(intPin);
}

int32_t Adxl345::WakeUp_() {
    auto rc = ESP_OK;

    /* Set device to measure mode */
    rc |= comms_.SetBit(REG::POWER_CTL, 3, CMD::WRITE, CMD::READ);

    /* Turn off LOW POWER */
    rc |= comms_.SetBit(REG::BW_RATE, 3, CMD::WRITE, CMD::READ);
    return rc;
}

int32_t Adxl345::EnableInterrupt_(intPin_t intPin) {
    auto rc = ESP_OK;
    rc |= esp_event_loop_create_default();
    rc |= masterIntPin_.EnableInterrupt(GPIO_INTR_POSEDGE);
    rc |= masterIntPin_.SetEventHandler(&s_DataReadyEventHandler_);
    if (ESP_OK != rc) {
        ESP_LOGE("adxl345", "In function %s: Failed to setup master interrupts",
                __func__);
        return rc;
    }

    /* Setup slave interrupt generation */
    switch (intPin) {
        case INT_PIN::INT1: {
            rc |= comms_.SetBit(REG::INT_ENABLE, 7, CMD::WRITE, CMD::READ);
            rc |= comms_.ClearBit(REG::INT_MAP, 7, CMD::WRITE, CMD::READ);
            return rc;
            break;
        }

        case INT_PIN::INT2: {
            rc |= comms_.SetBit(REG::INT_ENABLE, 7, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::INT_MAP, 7, CMD::WRITE, CMD::READ);
            return rc;
            break;
        }

        default: {
            return rc;
            break;
        }
    }
}

esp_err_t Adxl345::ReadDataIntoBuffer(void* buffer, uint8_t byteCnt) {
    return comms_.ReadRegisterMultipleBytes(REG::DATA_X0, buffer, byteCnt, CMD::READ_BURST);
}
