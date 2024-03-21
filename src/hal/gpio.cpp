/**
 * @file gpio.cpp
 *
 * @brief HAL GPIO class source
 *
 * @date 03/2024
 */

#include "hal/gpio.hpp"

namespace hal {
Gpio::Gpio(const Config& gpio_config) : port{gpio_config.port}, pin{gpio_config.pin} {
}

bool Gpio::read(void) const {
    return HAL_GPIO_ReadPin(this->port, this->pin);
}

void Gpio::write(bool state) {
    HAL_GPIO_WritePin(this->port, this->pin, (GPIO_PinState) state);
}

void Gpio::toggle(void) {
    HAL_GPIO_TogglePin(this->port, this->pin);
}
}
