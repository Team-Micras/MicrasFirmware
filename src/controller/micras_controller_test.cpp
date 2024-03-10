#include "controller/micras_controller_test.hpp"

void micras_controller_test_loop(proxy::Led& led, proxy::Button& button) {
    if (button.get_state()) {
        led.turn_on();
    } else {
        led.turn_off();
    }
}
