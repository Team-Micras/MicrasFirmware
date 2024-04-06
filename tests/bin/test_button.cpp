/**
 * @file test_button.cpp
 *
 * @brief Test for the Button class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

int main() {
    test_core_init();
    proxy::Button button{button_config};
    proxy::Led    led{led_config};

    while (true) {
        if (button.is_pressed()) {
            led.turn_on();
        } else {
            led.turn_off();
        }
    }

    return 0;
}
