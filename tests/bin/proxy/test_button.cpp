/**
 * @file test_button.cpp
 *
 * @brief Test for the Button class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    proxy::Led    led{led_config};

    TestCore::loop([&button, &led]() {
        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            led.toggle();
        }
    });

    return 0;
}
