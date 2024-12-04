/**
 * @file
 */

#include "micras/micras_controller.hpp"
#include "micras/hal/mcu.hpp"

/*****************************************
 * Main Function
 *****************************************/

int main() {
    micras::hal::Mcu::init();
    micras::MicrasController micras_controller;

    while (true) {
        micras_controller.run();
    }

    return 0;
}
