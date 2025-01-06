/**
 * @file
 */

#include "constants.hpp"
#include "micras/nav/go_to_point.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

nav::Point goal{0.09, 0.36};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    hal::Timer            timer{timer_config};
    proxy::Button         button{button_config};
    proxy::Battery        battery{battery_config};
    proxy::RotarySensor   rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor   rotary_sensor_right{rotary_sensor_right_config};
    proxy::Imu            imu{imu_config};
    proxy::Locomotion     locomotion{locomotion_config};
    proxy::Argb<2>        argb{argb_config};
    proxy::WallSensors<4> wall_sensors{wall_sensors_config};
    nav::Odometry         odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config};
    nav::GoToPoint        go_to_point{wall_sensors, go_to_point_config, follow_wall_config};

    bool started = false;
    bool finished = false;
    wall_sensors.turn_on();
    timer.reset_us();

    TestCore::loop([&imu, &odometry, &go_to_point, &finished, &started, &locomotion, &argb, &button, &timer,
                    &wall_sensors]() {
        if (finished) {
            locomotion.set_command(0.0F, 0.0F);
            argb.set_color({0, 255, 0});
            return;
        }

        float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
        timer.reset_us();

        wall_sensors.update();
        imu.update();
        odometry.update(elapsed_time);

        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            started = true;
            argb.set_color({255, 0, 0});
            locomotion.enable();
            hal::Timer::sleep_ms(5000);
            argb.set_color({0, 0, 255});
            // go_to_point.calibrate();
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
            wall_sensors.turn_off();
            return;
        }

        if (not finished) {
            const auto& [linear, angular] =
                go_to_point.action(odometry.get_state(), goal, core::FollowWallType::NONE, elapsed_time);
            locomotion.set_command(linear, angular);
        }

        while (timer.elapsed_time_us() < 1000) { }
    });

    return 0;
}
