/**
 * @file led.hpp
 *
 * @brief Proxy Led class header
 *
 * @date 03/2024
 */

#ifndef __LED_HPP__
#define __LED_HPP__

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
         * @param led_config Configuration for the LED
         */
        Led(const Config& led_config);

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

#endif // __LED_HPP__
