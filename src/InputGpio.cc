#include "BaseGpio.h"
#include "InputGpio.h"
#include "esp_event.h"
#include "esp_log.h"

/* WARNING:
 * Using pullup and pulldown at the same time is not guarded.
 * May result in unexpected behaviour.
 * Refrein from using pullup and pulldown at the same time.
 */

bool accgpio::InputGpio::s_interruptServiceInstalled = false;

/* DEFINE event base */
ESP_EVENT_DEFINE_BASE(accgpio::INPUT_EVENTS);

/* ISR */
/* Currently takes pin number as an argument */
void IRAM_ATTR accgpio::InputGpio::ISRCallback(void* args) {
        int32_t pin = reinterpret_cast<int32_t>(args);

        /* Pose event to System Event Loop in INPUT_EVENTS group */
        esp_event_isr_post(INPUT_EVENTS, pin, nullptr, 0, nullptr);
}
/******************************************************************************/

accgpio::InputGpio::InputGpio(
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

int32_t accgpio::InputGpio::Read() {
    return gpio_get_level(pin_);
}

/* Interrupt related*/
/******************************************************************************/
esp_err_t accgpio::InputGpio::EnableInterrupt(gpio_int_type_t intType) {
    esp_err_t rc = ESP_OK;

    if (!s_interruptServiceInstalled) {
        rc = gpio_install_isr_service(0);
        if (ESP_OK == rc) {
            s_interruptServiceInstalled = true;
        }
    }

    if (ESP_OK == rc) {
        rc = gpio_set_intr_type(pin_, intType);
    }

    if (ESP_OK == rc) {
        rc = gpio_isr_handler_add(pin_, ISRCallback, (void*) pin_);
    }
    return rc;
}

esp_err_t accgpio::InputGpio::SetEventHandler(
        esp_event_handler_t eventHandler) {
    esp_err_t rc = ESP_OK;
    rc = esp_event_handler_instance_register(
            INPUT_EVENTS, pin_, eventHandler, 0, nullptr);

    if (ESP_OK == rc) {
        eventHandlerSet_ = true;
    } else {
        ESP_LOGE("InputGpio",
                "In function %s: Unable to set event handler. %s", __func__,
                esp_err_to_name(rc));
    }

    return rc;
}
