/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    proxy::Led    led{led_config};
    proxy::Argb   argb{argb_config};

    proxy::Argb::Color color{};

    TestCore::loop([&button, &led, &argb, &color]() {
        button.update();
        button.is_pressed() ? led.turn_on() : led.turn_off();

        if (button.get_status() == proxy::Button::Status::SHORT_PRESS) {
            color = proxy::Argb::Colors::red;
        } else if (button.get_status() == proxy::Button::Status::LONG_PRESS) {
            color = proxy::Argb::Colors::green;
        } else if (button.get_status() == proxy::Button::Status::EXTRA_LONG_PRESS) {
            color = proxy::Argb::Colors::blue;
        }

        argb.set_color(color);
        proxy::Stopwatch::sleep_ms(2);
    });
}
