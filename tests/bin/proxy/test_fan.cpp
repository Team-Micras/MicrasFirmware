/**
 * @file test_fan.cpp
 *
 * @brief Test for the Fan class
 *
 * @date 05/2024
 */

#include "target.hpp"
#include "test_core.hpp"

using namespace micras;

int main() {
    test_core_init();
    proxy::Fan fan{fan_config};

    while (true) {
        for (float i = 0; i < 80.0F; i++) {
            fan.set_speed(i);
            hal::Timer::sleep_ms(50);
        }

        for (float i = 80.0F; i > -80.0F; i--) {
            fan.set_speed(i);
            hal::Timer::sleep_ms(50);
        }

        for (float i = -80.0F; i < 0.0F; i++) {
            fan.set_speed(i);
            hal::Timer::sleep_ms(50);
        }
    }

    return 0;
}
