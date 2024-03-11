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
 * @brief Class for controlling Led.
 */
class Led {
    public:
        /**
         * @brief configuration structure for led
         */
        struct Config {
            hal::Gpio::Config gpio_config;
        };

        /**
         * @brief Constructor for the Led class
         *
         * @param led_config Configuration for the led
         */
        Led(const Config& led_config);

        /**
         * @brief Turn the led on
         */
        void turn_on();

        /**
         * @brief Turn the led off
         */
        void turn_off();

        /**
         * @brief Toggle led
         */
        void toggle();

    private:
        hal::Gpio led_gpio;
};
}  // namespace proxy

#endif // __LED_HPP__
