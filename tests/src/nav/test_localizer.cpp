/**
 * @file
 */

#include "constants.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/tick.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_position_x{};
static volatile float test_position_y{};
static volatile float test_orientation{};
static volatile float test_linear_velocity{};
static volatile float test_angular_velocity{};
static volatile float test_gyroscope_bias{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

/**
 * @brief Dead reckoning of the pose with the robot moved by hand, to check the encoders and the
 * gyroscope: their signs, the wheel radius and how fast the bias is found while the robot rests.
 */
int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Imu                imu{imu_config};
    const proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    const proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::Tick               tick{tick_config};

    nav::Localizer localizer{localizer_config};

    const auto measure = [&imu, &rotary_sensor_left, &rotary_sensor_right]() {
        return nav::Measurements{
            .left_wheel_angle = rotary_sensor_left.get_position(),
            .right_wheel_angle = rotary_sensor_right.get_position(),
            .angular_rate = imu.get_angular_velocity(proxy::Imu::Axis::Z),
            .acceleration =
                {.x = imu.get_linear_acceleration(proxy::Imu::Axis::X),
                 .y = imu.get_linear_acceleration(proxy::Imu::Axis::Y)},
            .imu_is_new = imu.is_new(),
            .walls = {},
        };
    };

    imu.update();
    localizer.reset({.position = {.x = 0.0F, .y = 0.0F}, .orientation = 0.0F}, measure());

    TestCore::loop([&imu, &tick, &localizer, &measure]() {
        tick.wait();
        imu.update();

        const nav::Measurements measurements = measure();

        localizer.predict(measurements, loop_time);
        localizer.correct_at_rest(measurements, loop_time);

        const nav::State& state = localizer.get_state();

        test_position_x = state.pose.position.x;
        test_position_y = state.pose.position.y;
        test_orientation = state.pose.orientation;
        test_linear_velocity = state.velocity.linear;
        test_angular_velocity = state.velocity.angular;
        test_gyroscope_bias = localizer.get_gyroscope_bias();
    });

    return 0;
}
