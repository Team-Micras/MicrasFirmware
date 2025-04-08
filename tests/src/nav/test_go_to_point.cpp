/**
 * @file
 */

#include "constants.hpp"
#include "micras/nav/go_to_point.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr nav::Point goal{0.09, 0.36};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    auto imu{std::make_shared<proxy::Imu>(imu_config)};

    hal::Timer        timer{timer_config};
    proxy::Button     button{button_config};
    proxy::Locomotion locomotion{locomotion_config};
    proxy::Argb       argb{argb_config};
    nav::Odometry     odometry{
        std::make_shared<proxy::RotarySensor>(rotary_sensor_left_config),
        std::make_shared<proxy::RotarySensor>(rotary_sensor_right_config), imu, odometry_config
    };
    nav::GoToPoint go_to_point{
        std::make_shared<proxy::WallSensors>(wall_sensors_config), go_to_point_config, follow_wall_config
    };

    bool started = false;
    bool finished = false;
    timer.reset_us();

    TestCore::loop([&imu, &odometry, &go_to_point, &finished, &started, &locomotion, &argb, &button, &timer]() {
        while (timer.elapsed_time_us() < 1000) { }

        if (finished) {
            locomotion.stop();
            argb.set_color(proxy::Argb::Colors::green);
            return;
        }

        const float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
        timer.reset_us();

        imu->update();
        odometry.update(elapsed_time);

        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            started = true;
            argb.set_color(proxy::Argb::Colors::red);
            locomotion.enable();
            hal::Timer::sleep_ms(3000);
            imu->calibrate();
            argb.set_color(proxy::Argb::Colors::blue);
            timer.reset_us();
            odometry.reset();
            go_to_point.reset();
            return;
        }

        if (not started) {
            return;
        }

        if (go_to_point.finished(odometry.get_state(), goal)) {
            finished = true;
            locomotion.disable();
            return;
        }

        if (not finished) {
            const auto& [linear, angular] =
                go_to_point.action(odometry.get_state(), goal, core::FollowWallType::NONE, elapsed_time);
            locomotion.set_command(linear, angular);
        }
    });

    return 0;
}
