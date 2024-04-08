/**
 * @file test_button.cpp
 *
 * @brief Test for the Button class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;

int main() {
    test_core_init();
    proxy::Button button{button_config};
    proxy::Led    led{led_config};

    while (true) {
        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            led.toggle();
        }
    }

    return 0;
}
