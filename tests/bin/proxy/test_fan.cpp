/**
 * @file test_fan.cpp
 *
 * @brief Test for the Fan class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Fan fan{fan_config};

    TestCore::loop([&fan]() {
        for (int8_t i = 0; i < 80; i++) {
            fan.set_speed(i);
            hal::Timer::sleep_ms(50);
        }

        for (int8_t i = 80; i > -80; i--) {
            fan.set_speed(i);
            hal::Timer::sleep_ms(50);
        }

        for (int8_t i = -80; i < 0.0F; i++) {
            fan.set_speed(i);
            hal::Timer::sleep_ms(50);
        }
    });

    return 0;
}
