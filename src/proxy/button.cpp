/**
 * @file button.cpp
 *
 * @brief Proxy Button class source
 *
 * @date 03/2024
 *
 */

#include "proxy/button.hpp"

namespace proxy {
Button::Button(const Config& button_config) : button_gpio(button_config.gpio_config) {
}

bool Button::get_state() {
    return this->button_gpio.read();
}
}  // namespace proxy
