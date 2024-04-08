/**
 * @file test_led.cpp
 *
 * @brief Test for the LED class
 *
 * @date 05/2024
 */

#include "target.hpp"
#include "test_core.hpp"

using namespace micras;

int main() {
    test_core_init();
    proxy::Led led{led_config};

    while (true) {
        led.toggle();

        hal::Timer::sleep_ms(500);
    }

    return 0;
}
