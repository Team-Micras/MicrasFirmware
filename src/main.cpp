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
    micras::Micras micras;

    while (true) {
        micras.update();
    }

    return 0;
}
