/**
 * @file led.cpp
 *
 * @brief Proxy Led class source
 *
 * @date 03/2024
 */

#include "micras/proxy/led.hpp"

namespace micras::proxy {
Led::Led(const Config& config) : gpio{config.gpio} { }

void Led::turn_on() {
    this->gpio.write(true);
}

void Led::turn_off() {
    this->gpio.write(false);
}

void Led::toggle() {
    this->gpio.toggle();
}
}  // namespace micras::proxy
