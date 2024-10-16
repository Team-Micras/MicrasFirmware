/**
 * @file test_battery.cpp
 *
 * @brief Test for the Battery class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_battery_voltage{};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Battery battery{battery_config};

    TestCore::loop([&battery]() { test_battery_voltage = battery.get_voltage(); });

    return 0;
}
