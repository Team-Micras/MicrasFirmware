/**
 * @file
 */

#include "constants.hpp"
#include "micras/nav/odometry.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_position_x{};
static volatile float test_position_y{};
static volatile float test_orientation{};
static volatile float test_linear_velocity{};
static volatile float test_angular_velocity{};

static volatile uint32_t test_loop_time;
static volatile uint32_t max_loop_time{0};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    hal::Timer          timer{timer_config};
    proxy::Locomotion   locomotion{locomotion_config};
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::Imu          imu{imu_config};
    nav::Odometry       odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config};

    timer.reset_us();
    imu.calibrate();

    TestCore::loop([&odometry, &imu, &timer]() {
        test_loop_time = timer.elapsed_time_us();
        float elapsed_time = test_loop_time / 1000000.0F;
        timer.reset_us();

        if (test_loop_time > max_loop_time) {
            max_loop_time = test_loop_time;
        }

        imu.update();
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
