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
    this->led_gpio.write(true);
}

void Led::turn_off() {
    this->led_gpio.write(false);
}

void Led::toggle() {
    this->led_gpio.toggle();
}
}  // namespace proxy
