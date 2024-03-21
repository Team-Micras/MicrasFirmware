/**
 * @file led.cpp
 *
 * @brief Proxy Led class source
 *
 * @date 03/2024
 */

#include "proxy/led.hpp"

namespace proxy {
Led::Led(const Config& led_config) : gpio{led_config.gpio} {
}

void Led::turn_on() {
    this->gpio.write(true);
}

void Led::turn_off() {
    this->gpio.write(false);
}

void Led::toggle() {
    this->gpio.toggle();
}
}  // namespace proxy
