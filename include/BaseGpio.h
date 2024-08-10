#pragma once

#include "driver/gpio.h"

class BaseGpio {
protected:
    gpio_num_t pin_;
};
