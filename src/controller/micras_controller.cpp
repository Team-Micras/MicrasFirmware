/**
 * @file micras_controller.cpp
 *
 * @brief Micras Controller class implementation
 *
 * @date 03/2024
 */

#include "controller/micras_controller.hpp"
#include "target.hpp"

MicrasController::MicrasController() : button(button_config), led(led_config) {
}

void MicrasController::run() {
    if (button.get_status() == proxy::Button::SHORT_PRESS) {
        led.turn_on();
    } else {
        led.turn_off();
    }
}
