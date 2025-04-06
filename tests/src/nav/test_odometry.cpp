/**
 * @file
 */

#include "constants.hpp"
#include "micras/nav/odometry.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_position_x{};
static volatile float test_position_y{};
static volatile float test_orientation{};
static volatile float test_linear_velocity{};
static volatile float test_angular_velocity{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    auto imu{std::make_shared<proxy::Imu>(imu_config)};

    hal::Timer    timer{timer_config};
    nav::Odometry odometry{
        std::make_shared<proxy::RotarySensor>(rotary_sensor_left_config),
        std::make_shared<proxy::RotarySensor>(rotary_sensor_right_config), imu, odometry_config
    };

    timer.reset_us();

    hal::Timer::sleep_ms(1000);
    imu->calibrate();

    TestCore::loop([&odometry, &imu, &timer]() {
        const float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
        timer.reset_us();

        imu->update();
        odometry.update(elapsed_time);

        const auto& state = odometry.get_state();

        test_position_x = state.pose.position.x;
        test_position_y = state.pose.position.y;
        test_orientation = state.pose.orientation;
        test_linear_velocity = state.velocity.linear;
        test_angular_velocity = state.velocity.angular;

        while (timer.elapsed_time_us() < 1000) { }
    });

    return 0;
}
