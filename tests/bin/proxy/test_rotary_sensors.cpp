/**
 * @file test_rotary_sensors.cpp
 *
 * @brief Test for the Rotary Sensor class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;

static volatile float left_position{};
static volatile float right_position{};

int main() {
    test_core_init();
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};

    while (true) {
        left_position = rotary_sensor_left.get_position();
        right_position = rotary_sensor_right.get_position();
    }

    return 0;
}
