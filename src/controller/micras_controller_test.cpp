#include "controller/micras_controller_test.hpp"

void micras_controller_test_loop(proxy::Button& button, proxy::Led& led) {
    if (button.get_state()) {
        led.turn_on();
    } else {
        led.turn_off();
    }
}
