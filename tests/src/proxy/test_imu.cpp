/**
 * @file test_battery.cpp
 *
 * @brief Test for the Battery class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(*-avoid-c-arrays)
static volatile float test_angular_velocity[3]{};
static volatile float test_linear_acceleration[3]{};
static volatile float test_orientation[3]{};

// NOLINTEND(*-avoid-c-arrays)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Imu     imu{imu_config};
    proxy::Argb<2> argb{argb_config};

    hal::Timer::sleep_ms(2);

    if (imu.check_whoami()) {
        argb.set_color({0, 255, 0});

    } else {
        argb.set_color({255, 0, 0});

        while (true) { }
    }

    TestCore::loop([&imu]() {
        imu.update_data();

        test_angular_velocity[0] = imu.get_angular_velocity(proxy::Imu::Axis::X);
        test_angular_velocity[1] = imu.get_angular_velocity(proxy::Imu::Axis::Y);
        test_angular_velocity[2] = imu.get_angular_velocity(proxy::Imu::Axis::Z);

        test_linear_acceleration[0] = imu.get_linear_acceleration(proxy::Imu::Axis::X);
        test_linear_acceleration[1] = imu.get_linear_acceleration(proxy::Imu::Axis::Y);
        test_linear_acceleration[2] = imu.get_linear_acceleration(proxy::Imu::Axis::Z);

        test_orientation[0] = imu.get_orientation(proxy::Imu::Axis::X);
        test_orientation[1] = imu.get_orientation(proxy::Imu::Axis::Y);
        test_orientation[2] = imu.get_orientation(proxy::Imu::Axis::Z);
    });

    return 0;
}
