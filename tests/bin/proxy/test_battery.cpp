/**
 * @file test_battery.cpp
 *
 * @brief Test for the Battery class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;

int main() {
    test_core_init();
    proxy::Battery battery{battery_config};
    float          battery_voltage{};

    while (true) {
        battery_voltage = battery.get_voltage();
    }

    return 0;
}
