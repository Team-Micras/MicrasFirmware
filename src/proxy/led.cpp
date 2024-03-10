/**
 * @file led.cpp
 *
 * @author Comp ThundeRatz <comp@thunderatz.org>
 *
 * @brief Proxy Led class source.
 *
 * @date 01/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#include "proxy/led.hpp"

namespace proxy {
Led::Led(Config led_config) : led_gpio(led_config.gpio_config) {
}

void Led::turn_on() {
    this->led_gpio.write(true);
}

void Led::turn_off() {
    this->led_gpio.write(false);
}

void Led::toggle() {
    this->led_gpio.toggle();
}
}  // namespace proxy
