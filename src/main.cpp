/**
 * @file main.cpp
 *
 * @brief Main function
 */

#include "mcu.hpp"
#include "target.hpp"

/*****************************************
 * Private Constant Definitions
 *****************************************/

// static constexpr uint16_t led_toggle_delay_ms = 1500;

/*****************************************
 * Main Function
 *****************************************/

int main(void) {
    hal::mcu::init();

    proxy::Led led(led_config);
    proxy::Button button(button_config);

    for (;;) {
        // hal::mcu::led_toggle();
        micras_controller_test_loop(led, button);

        // hal::mcu::sleep(led_toggle_delay_ms);
    }
}
