/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr float    test_speed{50.0F};
static constexpr uint32_t time_interval{1000};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button     button{button_config};
    proxy::Locomotion locomotion{locomotion_config};

    TestCore::loop([&button, &locomotion]() {
        auto button_status = button.get_status();

        switch (button_status) {
            case proxy::Button::Status::SHORT_PRESS:
                locomotion.set_command(test_speed, 0.0F);
                hal::Timer::sleep_ms(time_interval);
                locomotion.stop();
                hal::Timer::sleep_ms(time_interval);
                locomotion.set_command(-test_speed, 0.0F);
                hal::Timer::sleep_ms(time_interval);
                locomotion.stop();
                hal::Timer::sleep_ms(time_interval);

                locomotion.set_command(0.0F, test_speed);
                hal::Timer::sleep_ms(time_interval);
                locomotion.stop();
                hal::Timer::sleep_ms(time_interval);
                locomotion.set_command(0.0F, -test_speed);
                hal::Timer::sleep_ms(time_interval);
                locomotion.stop();
                break;

            case proxy::Button::Status::LONG_PRESS:
                for (int8_t i = 1; i < 100; i++) {
                    locomotion.set_wheel_command(i, i);
                    hal::Timer::sleep_ms(20);
                }

                for (int8_t i = 100; i > -100; i--) {
                    locomotion.set_wheel_command(i, i);
                    hal::Timer::sleep_ms(20);
                }

                for (int8_t i = -100; i <= 0; i++) {
                    locomotion.set_wheel_command(i, i);
                    hal::Timer::sleep_ms(20);
                }

                break;

            default:
                break;
        }
    });

    return 0;
}
