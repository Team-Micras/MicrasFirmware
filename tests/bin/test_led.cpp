/**
 * @file test_led.cpp
 *
 * @brief Test for the LED class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

int main() {
    test_core_init();
    proxy::Led led{led_config};

    while (true) {
        led.toggle();

        for (uint32_t i = 0; i < 1000000; i++) { }
    }

    return 0;
}
