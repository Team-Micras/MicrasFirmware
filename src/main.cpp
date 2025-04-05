/**
 * @file
 */

#include "micras/micras.hpp"
#include "micras/hal/mcu.hpp"

/*****************************************
 * Main Function
 *****************************************/

int main() {
    micras::hal::Mcu::init();
    micras::Micras micras_controller;

    while (true) {
        micras_controller.update();
    }

    return 0;
}
