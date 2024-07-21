#pragma once

#include <cstdint>

#include "driver/gpio.h"

namespace accgpio {

/******************************************************************************/
class GpioBase {
protected:
    gpio_num_t pin_;
};

/******************************************************************************/
/* Set pin as INPUT. Implements READ fucntionality */
class GpioInput : public GpioBase {
public:
    int32_t Read();
    GpioInput(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd);
};

/******************************************************************************/
/* Set pin as OUTPUT. Implements WRITE fucntionality */
class GpioOutput : public GpioBase {
private:
    int32_t level_ = 0;

public:
    GpioOutput(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd);
    esp_err_t Toggle();
    esp_err_t SetLevel(int32_t level);
};

/******************************************************************************/
}
