/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float left_position;
static volatile float right_position;

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    hal::Timer          timer{timer_config};
    hal::Timer          stop_timer;
    proxy::Button       button{button_config};
    proxy::Locomotion   locomotion{locomotion_config};
    proxy::RotarySensor left_rotary_sensor{rotary_sensor_left_config};
    proxy::RotarySensor right_rotary_sensor{rotary_sensor_right_config};

    left_position = left_rotary_sensor.get_position();
    right_position = right_rotary_sensor.get_position();

    bool running = false;
    stop_timer.reset_ms();

    TestCore::loop([&button, &locomotion, &left_rotary_sensor, &right_rotary_sensor, &timer, &running, &stop_timer]() {
        float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
        timer.reset_us();

        if (stop_timer.elapsed_time_ms() > 5000) {
            locomotion.stop();
            running = false;
            stop_timer.reset_ms();
        }

        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            running = not running;
            hal::Timer::sleep_ms(3000);
            stop_timer.reset_ms();
            return;
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
