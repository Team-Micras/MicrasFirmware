/**
 * @file test_distance_sensors.cpp
 *
 * @brief Test for the DistanceSensors class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;

static volatile float distance[8];

int main() {
    test_core_init();
    proxy::DistanceSensors<8> distance_sensors{distance_sensors_config};

    while (true) {
        for (uint8_t i = 0; i < 8; i++) {
            distance[i] = distance_sensors.get_distance(i);
        }
    }

    return 0;
}
