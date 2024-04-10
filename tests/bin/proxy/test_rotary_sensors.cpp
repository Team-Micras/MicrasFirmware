/**
 * @file test_rotary_sensors.cpp
 *
 * @brief Test for the Rotary Sensor class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float left_position{};   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float right_position{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};

    TestCore::loop([&rotary_sensor_left, &rotary_sensor_right]() {
        left_position = rotary_sensor_left.get_position();
        right_position = rotary_sensor_right.get_position();
    });

    return 0;
}
