#pragma once

#include <cstdint>

#include "BaseGpio.h"
#include "driver/gpio.h"
#include "esp_event.h" /* Used for interrupt handling. Event loop Library */

namespace accgpio {

/* DECLARE event base */
ESP_EVENT_DECLARE_BASE(INPUT_EVENTS);

class InputGpio : public BaseGpio {
private:

    /* Enabled the first time and ISR is installed on any pin */
    static bool s_interruptServiceInstalled;
    bool eventHandlerSet_; /* Track wether an event handler has been set */

public:
    int32_t Read();
    InputGpio(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd);

    /* Interrupt related */
    esp_err_t EnableInterrupt(gpio_int_type_t intType);
    esp_err_t SetEventHandler(
            esp_event_handler_t eventHandler,
            void* eventHandlerArgs = 0);

    /* Interrupt sub/service routine. Requires being the in the instruction RAM
     * Only one instance in the same memory location.
     */
    static void IRAM_ATTR ISRCallback(void* args);
};
} /* namespace accgpio */
