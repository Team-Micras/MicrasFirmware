/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    hal::Timer            wait_timer;
    proxy::Button         button{button_config};
    proxy::Argb<2>        argb{argb_config};
    proxy::WallSensors<4> wall_sensors{wall_sensors_config};

    TestCore::loop([&wall_sensors, &button, &wait_timer, &argb]() {
        static bool lateral_calibration{};
        static bool waiting{};
        static bool free_space_calibration{};

        wall_sensors.update();
        auto button_status = button.get_status();

        if (button_status == proxy::Button::Status::SHORT_PRESS) {
            wait_timer.reset_ms();
            waiting = true;
            free_space_calibration = false;
            argb.set_color({0, 0, 255});
        } else if (button_status == proxy::Button::Status::LONG_PRESS) {
            wait_timer.reset_ms();
            waiting = true;
            free_space_calibration = true;
            argb.set_color({255, 0, 0});
        }

        if (waiting and wait_timer.elapsed_time_ms() > 3000) {
            waiting = false;
            argb.set_color({0, 255, 0});

            if (lateral_calibration) {
                if (free_space_calibration) {
                    wall_sensors.calibrate_free_space(1);
                    wall_sensors.calibrate_free_space(2);
                } else {
                    wall_sensors.calibrate_wall(1);
                    wall_sensors.calibrate_wall(2);
                }
            } else {
                if (free_space_calibration) {
                    wall_sensors.calibrate_free_space(0);
                    wall_sensors.calibrate_free_space(3);
                } else {
                    wall_sensors.calibrate_wall(0);
                    wall_sensors.calibrate_wall(3);
                }
            }
        }

        hal::Timer::sleep_ms(2);
    });

    return 0;
}
