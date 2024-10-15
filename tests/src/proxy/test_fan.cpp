/**
 * @file test_fan.cpp
 *
 * @brief Test for the Fan class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_fan_speed{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    proxy::Fan    fan{fan_config};

    TestCore::loop([&button, &fan]() {
        while (button.get_status() == proxy::Button::Status::NO_PRESS) { }

        fan.set_speed(50.0F);

        while (fan.update_speed() < 50.0F) {
            test_fan_speed = fan.update_speed();
        }

        hal::Timer::sleep_ms(3000);

        fan.set_speed(0.0F);

        while (fan.update_speed() > 0.0F) {
            test_fan_speed = fan.update_speed();
        }
    });

    return 0;
}
