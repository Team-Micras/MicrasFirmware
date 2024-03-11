/**
 * @file micras_controller_test.hpp
 *
 * @brief Micras Controller Test class header
 *
 * @date 03/2024
 */

#ifndef __MICRAS_CONTROLLER_HPP__
#define __MICRAS_CONTROLLER_HPP__

#include "proxy/button.hpp"
#include "proxy/led.hpp"

class MicrasController {
    public:
        /**
         * @brief Constructor for the MicrasController class
         */
        MicrasController();

        /**
         * @brief Destroy the Micras Controller object
         */
        ~MicrasController() = default;

        /**
         * @brief Runs the controller loop once
         */
        void run();

    private:
        proxy::Button button;
        proxy::Led led;
};

#endif // __MICRAS_CONTROLLER_HPP__
