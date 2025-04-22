/**
 * @file
 */

#include <array>

#include "constants.hpp"
#include "micras/nav/follow_wall.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr std::array<float, 7>              commands{{5.0F, 10.0F, 15.0F, 20.0F, 25.0F, 30.0F, 35.0F}};
static constexpr std::array<int8_t, 4>             left_multiplier{{1, -1, 1, 0}};
static constexpr std::array<int8_t, 4>             right_multiplier{{1, 1, 0, 1}};
static constexpr std::array<proxy::Argb::Color, 4> colors{
    proxy::Argb::Colors::yellow, proxy::Argb::Colors::red, proxy::Argb::Colors::cyan, proxy::Argb::Colors::magenta
};

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_linear_speeds[commands.size()] = {};
static volatile float test_angular_speeds[commands.size()] = {};

static volatile float test_linear_accelerations[commands.size()] = {};
static volatile float test_angular_accelerations[commands.size()] = {};

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Stopwatch            loop_stopwatch{stopwatch_config};
    proxy::Stopwatch            running_stopwatch{};
    proxy::Button               button{button_config};
    proxy::Locomotion           locomotion{locomotion_config};
    proxy::Argb                 argb{argb_config};
    std::shared_ptr<proxy::Imu> imu{std::make_shared<proxy::Imu>(imu_config)};

    nav::Odometry odometry{
        std::make_shared<proxy::RotarySensor>(rotary_sensor_left_config),
        std::make_shared<proxy::RotarySensor>(rotary_sensor_right_config), imu, odometry_config
    };

    if (not imu->was_initialized()) {
        argb.set_color(proxy::Argb::Colors::red);
        return -1;
    }

    uint8_t iterator = 0;
    uint8_t test_type = 0;

    bool running = false;
    bool waiting = false;

    float last_linear_speed = 0.0F;
    float last_angular_speed = 0.0F;

    argb.set_color(colors[test_type]);
    locomotion.enable();
    loop_stopwatch.reset_us();

    TestCore::loop([&locomotion, &argb, &button, &loop_stopwatch, &running_stopwatch, &waiting, &running, &odometry,
                    &imu, &last_linear_speed, &last_angular_speed, &iterator, &test_type]() {
        while (loop_stopwatch.elapsed_time_us() < 1000) { }

        const float elapsed_time = loop_stopwatch.elapsed_time_us() / 1000000.0F;
        loop_stopwatch.reset_us();

        imu->update();
        odometry.update(elapsed_time);
        const auto& state = odometry.get_state();
        auto        button_status = button.get_status();

        if (button_status == proxy::Button::Status::SHORT_PRESS) {
            waiting = true;
            running_stopwatch.reset_ms();
            argb.set_color(proxy::Argb::Colors::blue);
            return;
        } else if (button_status == proxy::Button::Status::LONG_PRESS and iterator == 0) {
            test_type = (test_type + 1) % 4;
            argb.set_color(colors[test_type]);
        }

        if (waiting and running_stopwatch.elapsed_time_ms() > 2000) {
            imu->calibrate();
            locomotion.set_wheel_command(
                commands[iterator] * left_multiplier[test_type], commands[iterator] * right_multiplier[test_type]
            );

            running_stopwatch.reset_ms();
            argb.set_color(proxy::Argb::Colors::green);

            last_linear_speed = 0.0F;
            last_angular_speed = 0.0F;

            running = true;
            waiting = false;
            return;
        }

        if (running) {
            if (running_stopwatch.elapsed_time_ms() < 250) {
                float current_linear_acceleration = (state.velocity.linear - last_linear_speed) / elapsed_time;
                float current_angular_acceleration = (state.velocity.angular - last_angular_speed) / elapsed_time;

                last_linear_speed = state.velocity.linear;
                last_angular_speed = state.velocity.angular;

                if (current_linear_acceleration > test_linear_accelerations[iterator]) {
                    test_linear_accelerations[iterator] = current_linear_acceleration;
                }

                if (current_angular_acceleration > test_angular_accelerations[iterator]) {
                    test_angular_accelerations[iterator] = current_angular_acceleration;
                }
            } else if (running_stopwatch.elapsed_time_ms() > 1000) {
                running = false;
                locomotion.stop();

                test_linear_speeds[iterator] = state.velocity.linear;
                test_angular_speeds[iterator] = state.velocity.angular;

                iterator++;
                argb.set_color(colors[test_type]);
                return;
            }
        }

        if (iterator >= commands.size()) {
            iterator = 0;
            argb.set_color(proxy::Argb::Colors::white);
        }
    });

    return 0;
}
