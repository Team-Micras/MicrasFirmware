/**
 * @file test_distance_sensors.cpp
 *
 * @brief Test for the DistanceSensors class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_distance[4];  // NOLINT(*-avoid-c-arrays)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::DistanceSensors<4> distance_sensors{distance_sensors_config};

    TestCore::loop([&distance_sensors]() {
        for (uint8_t i = 0; i < 4; i++) {
            test_distance[i] = distance_sensors.get_distance(i);
        }
    });

    return 0;
}
