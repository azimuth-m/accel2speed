#pragma once

#include <cstdint>

#include "BaseGpio.h"
#include "driver/gpio.h"

class OutputGpio : public BaseGpio {
private:
    int32_t level_ = 0;

public:
    OutputGpio(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd);
    esp_err_t Toggle();
    esp_err_t SetLevel(int32_t level);
};
