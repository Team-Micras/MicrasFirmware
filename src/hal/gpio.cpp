/**
 * @file gpio.cpp
 *
 * @brief HAL GPIO class source
 *
 * @date 03/2024
 */

#include "hal/gpio.hpp"

namespace hal {
Gpio::Gpio(Config& config) : port{config.port}, pin{config.pin} {
}

bool Gpio::read(void) const {
    return HAL_GPIO_ReadPin(this->port, this->pin);
}

void Gpio::write(bool state) {
    HAL_GPIO_WritePin(this->port, this->pin, static_cast<GPIO_PinState>(state));
}

void Gpio::toggle(void) {
    HAL_GPIO_TogglePin(this->port, this->pin);
}
}
