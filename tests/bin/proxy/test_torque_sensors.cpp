/**
 * @file test_torque_sensors.cpp
 *
 * @brief Test for the TorqueSensors class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float torque[2];   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float current[2];  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::TorqueSensors<2> torque_sensors{torque_sensors_config};

    TestCore::loop([&torque_sensors]() {
        for (uint8_t i = 0; i < 2; i++) {
            torque[i] = torque_sensors.get_torque(i);
            current[i] = torque_sensors.get_current(i);
        }
    });

    return 0;
}
