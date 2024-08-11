#include <vector>

#include "esp_err.h"
#include "esp_log.h"

#include "Adxl345.h"
#include "InputGpio.h"

void Adxl345::s_DataReadyEventHandler_(
        void* handler_args,
        esp_event_base_t base,
        int32_t id,
        void* event_data) {

    Adxl345* objInstance = (Adxl345*) handler_args;
    objInstance->ReadDataIntoBuffer(&objInstance->rawAccelData_, sizeof(rawAccelData_));

}

/* Quake's fast inverse square root algorithm */
inline float Adxl345::Q_rsqrt(float number) {
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

inline float Adxl345::FastMagnitude(float x, float y) {
    return x * x + y * y * Q_rsqrt(x * x + y * y);
}

/* Using rectangular numeric integration */
inline float Adxl345::CalcVelocityRectangular(float a, float v, float dt) {
    return v + a * dt;
}

inline float Adxl345::CalcVelocityTrapezoidal(
        const std::vector<float>& a,
        double dt) {

    const uint8_t aLen = a.size();
    float v = 0.0;
    v += a[0];

    for (int i = 1; i < aLen - 1; i++) {
        v += 2 * a[i];
    }
    v += a[aLen - 1];
    v *= dt / 2.0;

    return v;
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
      busSpeedHz_(busSpeedHz),
      initialVelocity_(0) {

    accelMagnitudeXY_.reserve(bufferLen_);
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
    rc |= masterIntPin_.SetEventHandler(&s_DataReadyEventHandler_, this);
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
