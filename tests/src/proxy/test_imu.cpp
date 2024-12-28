/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_angular_velocity[3]{};
static volatile float test_linear_acceleration[3]{};

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Imu  imu{imu_config};
    proxy::Argb argb{argb_config};

    hal::Timer::sleep_ms(2);

    if (imu.check_whoami()) {
        argb.set_color(proxy::Argb::green);

    } else {
        argb.set_color(proxy::Argb::red);

        while (true) { }
    }

    TestCore::loop([&imu]() {
        imu.update();

        test_angular_velocity[0] = imu.get_angular_velocity(proxy::Imu::Axis::X);
        test_angular_velocity[1] = imu.get_angular_velocity(proxy::Imu::Axis::Y);
        test_angular_velocity[2] = imu.get_angular_velocity(proxy::Imu::Axis::Z);

        test_linear_acceleration[0] = imu.get_linear_acceleration(proxy::Imu::Axis::X);
        test_linear_acceleration[1] = imu.get_linear_acceleration(proxy::Imu::Axis::Y);
        test_linear_acceleration[2] = imu.get_linear_acceleration(proxy::Imu::Axis::Z);
    });

    return 0;
}
