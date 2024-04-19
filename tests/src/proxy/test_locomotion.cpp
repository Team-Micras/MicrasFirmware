/**
 * @file test_locomotion.cpp
 *
 * @brief Test for the Locomotion class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button     button{button_config};
    proxy::Locomotion locomotion{locomotion_config};

    TestCore::loop([&locomotion]() {
        while (button.get_status() == proxy::Button::Status::NO_PRESS) { }

        locomotion.set_wheel_command(50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_wheel_command(-50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_wheel_command(0.0F, 50.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_wheel_command(0.0F, -50.0F);
        hal::Timer::sleep_ms(1000);

        locomotion.set_command(50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_command(-50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_command(0.0F, 50.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_command(0.0F, -50.0F);
    });

    return 0;
}
