/**
 * @file button.hpp
 *
 * @brief Proxy Button class header
 *
 * @date 03/2024
 */

#ifndef __BUTTON_HPP__
#define __BUTTON_HPP__

#include "hal/gpio.hpp"

namespace proxy {
/**
 * @brief Class for controlling Button
 */
class Button {
    public:
        /**
         * @brief configuration structure for button
         */
        struct Config {
            hal::Gpio::Config gpio_config;
        };

        /**
         * @brief Constructor for the Button class
         *
         * @param button_config Configuration for the button
         */
        Button(const Config& button_config);

        /**
         * @brief Get the button state
         *
         * @return true if the button is pressed, false otherwise
         */
        bool get_state();

    private:
        hal::Gpio button_gpio;
};
}  // namespace proxy

#endif // __BUTTON_HPP__
