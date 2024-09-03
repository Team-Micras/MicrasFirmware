/**
 * @file test_rotary_sensors.cpp
 *
 * @brief Test for the Rotary Sensor class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float
    test_left_position{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables, *-avoid-c-arrays)
static volatile float
    test_right_position{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables, *-avoid-c-arrays)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};

    TestCore::loop([&rotary_sensor_left, &rotary_sensor_right]() {
        test_left_position = rotary_sensor_left.get_position();
        test_right_position = rotary_sensor_right.get_position();
    });

    return 0;
}
