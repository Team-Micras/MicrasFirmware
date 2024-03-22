/**
 * @file main.cpp
 *
 * @brief Main function
 *
 * @date 03/2024
 */

#include "controller/micras_controller.hpp"
#include "hal/mcu.hpp"

/*****************************************
 * Main Function
 *****************************************/

int main(void) {
    hal::Mcu::init();
    MicrasController micras_controller;

    while (true) {
        micras_controller.run();
    }
}
