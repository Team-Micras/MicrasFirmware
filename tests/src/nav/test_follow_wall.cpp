/**
 * @file
 */

#include <tuple>

#include "constants.hpp"
#include "micras/nav/follow_wall.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/nav/speed_controller.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_position_x{};
static volatile float test_position_y{};
static volatile float test_orientation{};
static volatile float test_linear_velocity{};
static volatile float test_angular_velocity{};
static volatile float test_angular_correction{};
static volatile bool  test_front_wall{};
static volatile float test_left_response{};
static volatile float test_right_response{};
static volatile float test_left_ff{};
static volatile float test_right_ff{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Stopwatch  loop_stopwatch{stopwatch_config};
    proxy::Button     button{button_config};
    proxy::Argb       argb{argb_config};
    proxy::Locomotion locomotion{locomotion_config};
    proxy::Led        led{led_config};

    auto imu{std::make_shared<proxy::Imu>(imu_config)};
    auto wall_sensors{std::make_shared<proxy::WallSensors>(wall_sensors_config)};

    nav::Odometry odometry{
        std::make_shared<proxy::RotarySensor>(rotary_sensor_left_config),
        std::make_shared<proxy::RotarySensor>(rotary_sensor_right_config), imu, odometry_config
    };

    nav::FollowWall      follow_wall{wall_sensors, follow_wall_config};
    nav::SpeedController speed_controller{speed_controller_config};

    if (not imu->was_initialized()) {
        argb.set_color(proxy::Argb::Colors::red);
        return -1;
    }

    bool running = false;

    constexpr float target_linear_speed = 0.4F;

    static proxy::Argb::Color color_left{};
    static proxy::Argb::Color color_right{};

    loop_stopwatch.reset_us();

    wall_sensors->turn_on();

    locomotion.enable();

    TestCore::loop([&]() {
        while (loop_stopwatch.elapsed_time_us() < 1000) { }
        const float elapsed_time = loop_stopwatch.elapsed_time_us() / 1000000.0F;
        loop_stopwatch.reset_us();

        button.update();
        imu->update();
        wall_sensors->update();
        odometry.update(elapsed_time);

        nav::State& state = odometry.get_state();

        test_position_x = state.pose.position.x;
        test_position_y = state.pose.position.y;
        test_orientation = state.pose.orientation;
        test_linear_velocity = state.velocity.linear;
        test_angular_velocity = state.velocity.angular;

        test_front_wall = wall_sensors->get_wall(wall_sensors_index.left_front) and
                          wall_sensors->get_wall(wall_sensors_index.right_front);

        // color_right.blue = wall_sensors->get_wall(wall_sensors_index.right) ? 255 : 0;
        // color_right.red = wall_sensors->get_wall(wall_sensors_index.right_front) ? 255 : 0;
        // argb.set_color(color_right, 0);

        // color_left.blue = wall_sensors->get_wall(wall_sensors_index.left) ? 255 : 0;
        // color_left.red = wall_sensors->get_wall(wall_sensors_index.left_front) ? 255 : 0;
        // argb.set_color(color_left, 1);

        color_left.green = follow_wall.get_is_following_left() ? 255 : 0;
        color_right.green = follow_wall.get_is_following_right() ? 255 : 0;
        argb.set_color(color_left, 1);
        argb.set_color(color_right, 0);

        if (button.get_status() == proxy::Button::Status::SHORT_PRESS) {
            if (not running) {
                running = true;
                imu->calibrate();
                odometry.reset();
                speed_controller.reset();
            } else {
                running = false;
                locomotion.stop();
            }
            return;
        }

        if (not running) {
            return;
        }

        // if (test_front_wall) {
        //     running = false;
        //     locomotion.stop();
        //     return;
        // }

        const float angular_correction = follow_wall.compute_angular_correction(elapsed_time, state);
        test_angular_correction = angular_correction;

        const nav::Twist desired_speeds{
            .linear = target_linear_speed,
            .angular = angular_correction,
        };

        auto [left_response, right_response] =
            speed_controller.compute_control_commands(state.velocity, desired_speeds, elapsed_time);

        auto [left_ff, right_ff] = speed_controller.compute_feed_forward_commands(desired_speeds, elapsed_time);

        test_left_response = left_response;
        test_right_response = right_response;
        test_left_ff = left_ff;
        test_right_ff = right_ff;

        locomotion.set_wheel_command(left_ff + left_response, right_ff + right_response);
    });

    return 0;
}
