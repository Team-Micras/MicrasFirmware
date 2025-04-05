/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_left_position{};   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_right_position{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    hal::Timer          stop_timer;
    proxy::Button       button{button_config};
    proxy::Locomotion   locomotion{locomotion_config};
    proxy::RotarySensor left_rotary_sensor{rotary_sensor_left_config};
    proxy::RotarySensor right_rotary_sensor{rotary_sensor_right_config};

    bool running = false;
    stop_timer.reset_ms();

    TestCore::loop([&button, &locomotion, &left_rotary_sensor, &right_rotary_sensor, &running, &stop_timer]() {
        test_left_position = left_rotary_sensor.get_position();
        test_right_position = right_rotary_sensor.get_position();

        if (running and stop_timer.elapsed_time_ms() > 5000) {
            running = false;
            stop_timer.reset_ms();
        } else if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            running = not running;
            stop_timer.reset_ms();
        }

        if (running) {
            locomotion.enable();
            locomotion.set_command(50.0F, 0.0F);
        } else {
            locomotion.disable();
            locomotion.stop();
        }
    });

    return 0;
}
