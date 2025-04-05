/**
 * @file
 */

#include <array>

#include "constants.hpp"
#include "micras/nav/look_at_point.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr nav::Point goal{0.27F, 0.09F};

static volatile float test_velocity{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    hal::Timer          timer{timer_config};
    hal::Timer          stop_timer;
    proxy::Button       button{button_config};
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::Imu          imu{imu_config};
    proxy::Locomotion   locomotion{locomotion_config};
    proxy::Argb         argb{argb_config};
    nav::Odometry       odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config};
    nav::LookAtPoint    look_at_point{look_at_point_config};

    bool started = false;
    bool finished = false;
    timer.reset_us();

    TestCore::loop([&imu, &odometry, &look_at_point, &finished, &started, &locomotion, &argb, &button, &timer,
                    &stop_timer]() {
        while (timer.elapsed_time_us() < 1000) { }

        if (finished) {
            locomotion.stop();
            return;
        }

        float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
        timer.reset_us();

        imu.update();
        odometry.update(elapsed_time);

        const nav::State& state = odometry.get_state();
        test_velocity = state.velocity.linear;

        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            started = true;
            argb.set_color(proxy::Argb::Colors::cyan);
            locomotion.enable();
            hal::Timer::sleep_ms(3000);
            imu.calibrate();
            stop_timer.reset_ms();
            timer.reset_us();
            odometry.reset();
            look_at_point.reset();
            return;
        }

        if (not started) {
            return;
        }

        if (stop_timer.elapsed_time_ms() > 3000) {
            locomotion.disable();

            if (not finished) {
                argb.set_color(proxy::Argb::Colors::red);
            }

            finished = true;

            return;
        }

        if (look_at_point.finished(state, goal)) {
            finished = true;
            locomotion.disable();
            argb.set_color(proxy::Argb::Colors::green);
        } else {
            finished = false;
            argb.set_color(proxy::Argb::Colors::blue);
        }

        const auto& [linear, angular] = look_at_point.action(state, goal, elapsed_time);
        locomotion.set_command(linear, angular);
    });

    return 0;
}
