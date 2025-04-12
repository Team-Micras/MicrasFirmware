/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr float max_speed{50.0F};

static volatile float test_fan_speed{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    proxy::Fan    fan{fan_config};

    TestCore::loop([&button, &fan]() {
        while (button.get_status() == proxy::Button::Status::NO_PRESS) { }

        fan.set_speed(max_speed);

        while (fan.update() < max_speed) {
            test_fan_speed = fan.update();
        }

        proxy::Stopwatch::sleep_ms(3000);

        fan.set_speed(0.0F);

        while (fan.update() > 0.0F) {
            test_fan_speed = fan.update();
        }
    });

    return 0;
}
