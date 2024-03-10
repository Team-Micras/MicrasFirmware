/**
 * @file button.cpp
 *
 * @author Comp ThundeRatz <comp@thunderatz.org>
 *
 * @brief Proxy Button class source.
 *
 * @date 01/2024
 *
 * @copyright MIT License - Copyright (c) 2024 ThundeRatz
 *
 */

#include "proxy/button.hpp"

namespace proxy {
Button::Button(Config button_config) : button_gpio(button_config.gpio_config) {
}

bool Button::get_state() {
    return this->button_gpio.read();
}
}  // namespace proxy
