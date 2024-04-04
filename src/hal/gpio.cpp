/**
 * @file gpio.cpp
 *
 * @brief HAL GPIO class source
 *
 * @date 03/2024
 */

#include "hal/gpio.hpp"

namespace hal {
Gpio::Gpio(const Config& config) : port{config.port}, pin{config.pin} {
}

bool Gpio::read() const {
    return HAL_GPIO_ReadPin(this->port, this->pin) == GPIO_PIN_SET;
}

void Gpio::write(bool state) {
    HAL_GPIO_WritePin(this->port, this->pin, static_cast<GPIO_PinState>(state));
}

void Gpio::toggle() {
    HAL_GPIO_TogglePin(this->port, this->pin);
}
}  // namespace hal
