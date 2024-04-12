/**
 * @file test_battery.cpp
 *
 * @brief Test for the Battery class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables, *-avoid-c-arrays)
static volatile float angular_velocity[3]{};
static volatile float linear_acceleration[3]{};
static volatile float orientation[3]{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables, *-avoid-c-arrays)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Imu imu{imu_config};

    while (true) {
        angular_velocity[0] = imu.get_angular_velocity(proxy::Imu::Axis::X);
        angular_velocity[1] = imu.get_angular_velocity(proxy::Imu::Axis::Y);
        angular_velocity[2] = imu.get_angular_velocity(proxy::Imu::Axis::Z);

        linear_acceleration[0] = imu.get_linear_acceleration(proxy::Imu::Axis::X);
        linear_acceleration[1] = imu.get_linear_acceleration(proxy::Imu::Axis::Y);
        linear_acceleration[2] = imu.get_linear_acceleration(proxy::Imu::Axis::Z);

        orientation[0] = imu.get_orientation(proxy::Imu::Axis::X);
        orientation[1] = imu.get_orientation(proxy::Imu::Axis::Y);
        orientation[2] = imu.get_orientation(proxy::Imu::Axis::Z);
    }

    return 0;
}
