/**
 * @file test_torque_sensors.cpp
 *
 * @brief Test for the TorqueSensors class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_torque[2];   // NOLINT(*-avoid-c-arrays)
static volatile float test_current[2];  // NOLINT(*-avoid-c-arrays)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    bool                    running = false;
    proxy::TorqueSensors<2> torque_sensors{torque_sensors_config};
    proxy::Locomotion       locomotion{locomotion_config};
    proxy::Button           button{button_config};

    TestCore::loop([&torque_sensors, &locomotion, &button, &running]() {
        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            running = not running;
        }

        if (running) {
            locomotion.set_command(80.0F, 0.0F);
        } else {
            locomotion.set_command(0.0F, 0.0F);
        }

        for (uint8_t i = 0; i < 2; i++) {
            test_torque[i] = torque_sensors.get_torque(i);
            test_current[i] = torque_sensors.get_current(i);
        }
    });

    return 0;
}
