#include <algorithm>
#include <vector>
#include <iostream>
#include <iomanip>

#include "esp_err.h"
#include "esp_log.h"

#include "Adxl345.h"
#include "InputGpio.h"

#define G (9.807F)
#define X_TO_G (0.0043F)
#define Y_TO_G (0.0038F)
#define Z_TO_G (0.00485F)

void Adxl345::s_DataReadyEventHandler_(
        void* handler_args,
        esp_event_base_t base,
        int32_t id,
        void* event_data) {
    Adxl345* objInstance = (Adxl345*) handler_args;

    objInstance->ReadDataIntoBuffer_(&objInstance->rawAccelData_, sizeof(rawAccelData_));

    int16_t x = std::abs(objInstance->rawAccelData_.xAccel);
    int16_t y = std::abs(objInstance->rawAccelData_.yAccel);
    int16_t z = std::abs(objInstance->rawAccelData_.zAccel);

    /* Remove static noise and clamp */
    x = std::clamp((x - 18), 0, (1 << 13));
    y = std::clamp((y - 18), 0, (1 << 13));
    z = std::clamp((z - 32), 0, (1 << 13));

    // std::cout <<
    //         std::setw(10) << "["  << X_TO_G  * G  * x  << "]" <<
    //         std::setw(10) << " [" << Y_TO_G  * G  * y  << "]" <<
    //         std::setw(10) << " [" << Z_TO_G  * G  * z  << "]" <<
    // std::endl;

    float localVel = objInstance->CalcVelocityRectangular_(
            objInstance->FastMagnitude_(x * X_TO_G * G, y * Y_TO_G * G),
            objInstance->currentVelocity_,
            1.0 / objInstance->bandwidthHz_);

    if ((localVel - objInstance->currentVelocity_) < 0.001) {
        objInstance->currentVelocity_ = 0;
    } else {
        objInstance->currentVelocity_ = localVel;
    }
    std::cout << "V = " << objInstance->currentVelocity_ << std::endl;
}

/* Quake's fast inverse square root algorithm */
inline float Adxl345::Q_rsqrt_(float number) {
    const float threehalfs = 1.5F;

    float x2 = number * 0.5F;
    float y = number;
    long i = *(long*)&y;  // evil floating-point bit level hacking
    i = 0x5f3759df - (i >> 1);  // what the fuck?

    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y));  // 1st iteration
    y = y * (threehalfs - (x2 * y * y));  // 2nd iteration, if needed

    return y;
}

inline float Adxl345::FastMagnitude_(float x, float y) {
    return (x * x + y * y) * Q_rsqrt_(x * x + y * y);
}

/* Using rectangular numeric integration */
inline float Adxl345::CalcVelocityRectangular_(float a, float v, float dt) {
    return v + a * dt;
}

Adxl345::Adxl345(
        IntPin_t intPin,
        InputGpio masterIntPin,
        int masterCsPinNum,
        uint8_t spiMode,
        uint8_t commandBitCnt,
        uint8_t addressBitCnt,
        int busSpeedHz,
        Spi comms,
        AccRange_t gRange,
        AccBandwidth_t bandwidth)
    : masterIntPin_(masterIntPin),
      comms_(comms),
      busSpeedHz_(busSpeedHz),
      range_(gRange),
      bandwidth_(bandwidth),
      currentVelocity_(0) {

    comms_.RegisterDevice(
            spiMode, masterCsPinNum, commandBitCnt, addressBitCnt, 0, busSpeedHz, 0, 0, 0);
    SetSensFullResolution_();
    SetSensGRange_();
    SetSensOutDataRate_();
    WakeUp_();
    EnableInterrupt_(intPin);

}

int32_t Adxl345::WakeUp_() {
    auto rc = ESP_OK;

    /* Set device to measure mode */
    rc |= comms_.SetBit(REG::POWER_CTL, POS::D3, CMD::WRITE, CMD::READ);

    /* Turn off LOW POWER */
    rc |= comms_.SetBit(REG::BW_RATE, POS::D3, CMD::WRITE, CMD::READ);
    return rc;
}

int32_t Adxl345::EnableInterrupt_(IntPin_t intPin) {
    auto rc = ESP_OK;

    /* Setup slave interrupt generation */
    switch (intPin) {
        case INT_PIN::INT1: {
            rc |= esp_event_loop_create_default();
            rc |= masterIntPin_.EnableInterrupt(GPIO_INTR_POSEDGE);
            rc |= masterIntPin_.SetEventHandler(
                    &s_DataReadyEventHandler_, this);
            rc |= comms_.SetBit(
                    REG::INT_ENABLE, POS::D7, CMD::WRITE, CMD::READ);
            rc |= comms_.ClrBit(
                    REG::INT_MAP, POS::D7, CMD::WRITE, CMD::READ);
            return rc;
        }

        case INT_PIN::INT2: {
            rc |= esp_event_loop_create_default();
            rc |= masterIntPin_.EnableInterrupt(GPIO_INTR_POSEDGE);
            rc |= masterIntPin_.SetEventHandler(
                    &s_DataReadyEventHandler_, this);
            rc |= comms_.SetBit(
                    REG::INT_ENABLE, POS::D7, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(
                    REG::INT_MAP, POS::D7, CMD::WRITE, CMD::READ);
            return rc;
        }

        default: {
            return ESP_FAIL;
        }
    }
}

int32_t Adxl345::SetSensFullResolution_() {
    return comms_.SetBit(REG::DATA_FORMAT, POS::D3, CMD::WRITE, CMD::READ);
}

int32_t Adxl345::SetSensGRange_() {
    auto rc = ESP_OK;
    switch (range_) {
        case ACC_RANGE::G4: {
            rc |= comms_.SetBit(
                    REG::DATA_FORMAT, POS::D0, CMD::WRITE, CMD::READ);
            rc |= comms_.ClrBit(
                    REG::DATA_FORMAT, POS::D1, CMD::WRITE, CMD::READ);
            return rc;
        }

        case ACC_RANGE::G8: {
            rc |= comms_.ClrBit(
                    REG::DATA_FORMAT, POS::D0, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(
                    REG::DATA_FORMAT, POS::D1, CMD::WRITE, CMD::READ);
            return rc;
        }

        case ACC_RANGE::G16: {
            rc |= comms_.SetBit(
                    REG::DATA_FORMAT, POS::D0, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(
                    REG::DATA_FORMAT, POS::D1, CMD::WRITE, CMD::READ);
            return rc;
        }

        /* Default is 2g */
        default: {
            rc |= comms_.ClrBit(
                    REG::DATA_FORMAT, POS::D0, CMD::WRITE, CMD::READ);
            rc |= comms_.ClrBit(
                    REG::DATA_FORMAT, POS::D1, CMD::WRITE, CMD::READ);
            return rc;
        }
    }
}

int32_t Adxl345::SetSensOutDataRate_() {
    auto rc = ESP_OK;
    switch(bandwidth_) {
                case ACC_BANDW::BW8: {
            rc |= comms_.ClrBit(REG::BW_RATE, POS::D0, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::BW_RATE, POS::D1, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::BW_RATE, POS::D2, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::BW_RATE, POS::D3, CMD::WRITE, CMD::READ);
            bandwidthHz_ = 800;
            return rc;
        }

        /* Use 1600Hz output data bandwidth */
        default: {
            rc |= comms_.SetBit(REG::BW_RATE, POS::D0, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::BW_RATE, POS::D1, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::BW_RATE, POS::D2, CMD::WRITE, CMD::READ);
            rc |= comms_.SetBit(REG::BW_RATE, POS::D3, CMD::WRITE, CMD::READ);
            bandwidthHz_ = 1600;
            return rc;
        }
    }
}

esp_err_t Adxl345::ReadDataIntoBuffer_(void* buffer, uint8_t byteCnt) {
    return comms_.ReadRegisterMultipleBytes(REG::DATA_X0, buffer, byteCnt, CMD::READ_BURST);
}
