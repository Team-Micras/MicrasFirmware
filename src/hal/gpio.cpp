/**
 * @file gpio.cpp
 *
 * @author Comp ThundeRatz <comp@thunderatz.org>
 *
 * @brief HAL GPIO class source.
 *
 * @date 01/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#include "hal/gpio.hpp"

namespace hal {
Gpio::Gpio(const Config& gpio_config) : pin(gpio_config.pin.pin), port(gpio_config.port.port) {
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
