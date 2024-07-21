#include <cassert>

#include "my_gpio.h"

/* WARNING:
 * Using pullup and pulldown at the same time is not guarded.
 * May result in unexpected behaviour.
 * Refrein from using pullup and pulldown at the same time.
 */

/* GpioInput */
/******************************************************************************/
accgpio::GpioInput::GpioInput(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd) {

    /* cannot use initializer list due to protected inheritance */
    pin_ = pin;
    gpio_config_t cfg;
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = pu;
    cfg.pull_down_en = pd;
    cfg.intr_type = GPIO_INTR_POSEDGE;

    gpio_config(&cfg);
}

int32_t accgpio::GpioInput::Read() {
    return gpio_get_level(pin_);
}
/******************************************************************************/

/* GpioOutput */
accgpio::GpioOutput::GpioOutput(
        const gpio_num_t pin,
        const gpio_pullup_t pu,
        const gpio_pulldown_t pd) {

        pin_ = pin;

        gpio_config_t cfg;
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode = GPIO_MODE_OUTPUT;
        cfg.pull_up_en = pu;
        cfg.pull_down_en = pd;
        cfg.intr_type = GPIO_INTR_DISABLE;

        gpio_config(&cfg);
}

esp_err_t accgpio::GpioOutput::Toggle() {
    level_ = level_ ? 0 : 1;
    return gpio_set_level(pin_, level_ ? 1 : 0);
}

esp_err_t accgpio::GpioOutput::SetLevel(int32_t level)
{
    return gpio_set_level(pin_, level_);
}
