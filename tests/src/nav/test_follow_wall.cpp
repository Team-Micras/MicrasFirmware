/**
 * @file
 */

#include <array>

#include "constants.hpp"
#include "micras/nav/follow_wall.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr float linear = 20.0F;

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    hal::Timer            timer{timer_config};
    proxy::Button         button{button_config};
    proxy::WallSensors<4> wall_sensors{wall_sensors_config};
    proxy::Locomotion     locomotion{locomotion_config};
    proxy::Argb<2>        argb{argb_config};
    nav::FollowWall       follow_wall{wall_sensors, follow_wall_config};

    timer.reset_us();

    TestCore::loop([&wall_sensors, &locomotion, &argb, &button, &timer, &follow_wall]() {
        static core::FollowWallType follow_wall_type = core::FollowWallType::NONE;
        static bool                 started = false;

        float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
        timer.reset_us();
        wall_sensors.update();
        auto button_status = button.get_status();

        if (button_status == proxy::Button::Status::SHORT_PRESS) {
            started = not started;

            if (started) {
                wall_sensors.turn_on();
                locomotion.enable();
                follow_wall.reset();
            } else {
                wall_sensors.turn_off();
                locomotion.disable();
            }
        }

        if (button_status == proxy::Button::Status::LONG_PRESS) {
            follow_wall_type = static_cast<core::FollowWallType>(follow_wall_type + 1 % 5);

            switch (follow_wall_type) {
                case core::FollowWallType::NONE:
                    argb.set_color({0, 0, 0});
                    break;
                case core::FollowWallType::FRONT:
                    argb.set_color({0, 255, 0});
                    break;
                case core::FollowWallType::LEFT:
                    argb.set_color({0, 0, 255});
                    break;
                case core::FollowWallType::RIGHT:
                    argb.set_color({255, 0, 0});
                    break;
                case core::FollowWallType::PARALLEL:
                    argb.set_color({255, 0, 255});
            }
        }

        if (not started) {
            return;
        }

        float angular = follow_wall.action(follow_wall_type, elapsed_time);
        locomotion.set_command(linear, angular);

        while (timer.elapsed_time_us() < 1000) { }
    });

    return 0;
}
