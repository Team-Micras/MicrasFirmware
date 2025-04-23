/**
 * @file
 */

#include <array>

#include "constants.hpp"
#include "micras/nav/follow_wall.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr float linear_command{20.0F};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    auto wall_sensors{std::make_shared<proxy::WallSensors>(wall_sensors_config)};

    proxy::Stopwatch  stopwatch{stopwatch_config};
    proxy::Button     button{button_config};
    proxy::Locomotion locomotion{locomotion_config};
    proxy::Argb       argb{argb_config};
    nav::FollowWall   follow_wall{wall_sensors, follow_wall_config};

    bool                 started = false;
    core::FollowWallType follow_wall_type = core::FollowWallType::NONE;
    stopwatch.reset_us();

    TestCore::loop([&wall_sensors, &locomotion, &argb, &button, &stopwatch, &follow_wall, &started,
                    &follow_wall_type]() {
        while (stopwatch.elapsed_time_us() < 1000) { }

        const float elapsed_time = stopwatch.elapsed_time_us() / 1000000.0F;
        stopwatch.reset_us();

        wall_sensors->update();
        button.update();

        if (button.get_status() == proxy::Button::Status::SHORT_PRESS) {
            started = not started;

            if (started) {
                wall_sensors->turn_on();
                locomotion.enable();
                follow_wall.reset();
                proxy::Stopwatch::sleep_ms(3000);
                return;
            }

            wall_sensors->turn_off();
            locomotion.disable();
        }

        if (button.get_status() == proxy::Button::Status::LONG_PRESS) {
            follow_wall_type = static_cast<core::FollowWallType>((follow_wall_type + 1) % 5);

            switch (follow_wall_type) {
                case core::FollowWallType::NONE:
                    argb.turn_off();
                    break;
                case core::FollowWallType::FRONT:
                    argb.set_color(proxy::Argb::Colors::green);
                    break;
                case core::FollowWallType::LEFT:
                    argb.set_color(proxy::Argb::Colors::blue);
                    break;
                case core::FollowWallType::RIGHT:
                    argb.set_color(proxy::Argb::Colors::red);
                    break;
                case core::FollowWallType::PARALLEL:
                    argb.set_color(proxy::Argb::Colors::magenta);
                default:
                    break;
            }
        }

        if (not started) {
            return;
        }

        const float angular_command = follow_wall.action(follow_wall_type, elapsed_time);
        locomotion.set_command(linear_command, angular_command);
    });

    return 0;
}
