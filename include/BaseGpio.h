#pragma once

#include "driver/gpio.h"

namespace accgpio {

class BaseGpio {
protected:
    gpio_num_t pin_;
};
} /* namespace accgpio */
