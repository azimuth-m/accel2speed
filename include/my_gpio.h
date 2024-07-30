#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_event.h" /* Used for interrupt handling. Event loop Library */

namespace accgpio {

/* Declare event base */
ESP_EVENT_DECLARE_BASE(INPUT_EVENTS);

/******************************************************************************/
class GpioBase {
protected:
    gpio_num_t pin_;
};

/******************************************************************************/
/* Set pin as INPUT. Implements READ fucntionality */
class GpioInput : public GpioBase {
private:
    bool eventHandlerSet_; /* Track wether an event handler has been set */
    static bool s_interruptServiceInstalled; /* Enabled the first time and
                                              * ISR is installed on any pin
                                              */
public:
    int32_t Read();
    GpioInput(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd);

    /* Interrupt related */
    esp_err_t EnableInterrupt(gpio_int_type_t intType);
    esp_err_t SetEventHandler(esp_event_handler_t eventHandler);

    /* Interrupt sub/service routine. Requires being the in the instruction RAM
     * Only one instance in the same memory location.
     */
    static void IRAM_ATTR ISRCallback(void* args);
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
