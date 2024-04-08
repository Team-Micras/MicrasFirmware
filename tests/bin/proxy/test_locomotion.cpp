/**
 * @file test_locomotion.cpp
 *
 * @brief Test for the Locomotion class
 *
 * @date 05/2024
 */

#include "target.hpp"
#include "test_core.hpp"

using namespace micras;

int main() {
    test_core_init();
    proxy::Locomotion locomotion{locomotion_config};

    while (true) {
        locomotion.set_wheel_speed(50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_wheel_speed(-50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_wheel_speed(0.0F, 50.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_wheel_speed(0.0F, -50.0F);
        hal::Timer::sleep_ms(1000);

        locomotion.set_speed(50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_speed(-50.0F, 0.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_speed(0.0F, 50.0F);
        hal::Timer::sleep_ms(1000);
        locomotion.set_speed(0.0F, -50.0F);
    }

    return 0;
}
