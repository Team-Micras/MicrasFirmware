/**
 * @file test_battery.cpp
 *
 * @brief Test for the Battery class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_battery_voltage_raw{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_battery_voltage{};      // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Battery battery{battery_config};

    TestCore::loop([&battery]() {
        battery.update();
        test_battery_voltage_raw = battery.get_voltage_raw();
        test_battery_voltage = battery.get_voltage();
        hal::Timer::sleep_ms(2);
    });

    return 0;
}
