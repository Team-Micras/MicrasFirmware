/**
 * @file led.hpp
 *
 * @brief Proxy Led class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_LED_HPP
#define MICRAS_PROXY_LED_HPP

#include "hal/gpio.hpp"

namespace proxy {
/**
 * @brief Class for controlling an LED
 */
class Led {
    public:
        /**
         * @brief Configuration structure for LED
         */
        struct Config {
            hal::Gpio::Config gpio;
        };

        /**
         * @brief Constructor for the Led class
         *
         * @param config Configuration for the LED
         */
        Led(Config& config);

        /**
         * @brief Turn the LED on
         */
        void turn_on();

        /**
         * @brief Turn the LED off
         */
        void turn_off();

        /**
         * @brief Toggle the LED
         */
        void toggle();

    private:
        /**
         * @brief Gpio object for the LED
         */
        hal::Gpio gpio;
};
}  // namespace proxy

#endif // MICRAS_PROXY_LED_HPP
