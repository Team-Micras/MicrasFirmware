/**
 * @file test_led.cpp
 *
 * @brief Test for the LED class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Led led{led_config};

    TestCore::loop([&led]() {
        led.toggle();

        hal::Timer::sleep_ms(500);
    });

    return 0;
}
