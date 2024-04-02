/**
 * @file gpio.hpp
 *
 * @brief HAL GPIO class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_HAL_GPIO_HPP
#define MICRAS_HAL_GPIO_HPP

#include <cstdint>
#include <gpio.h>

namespace hal {
/**
 * @brief Class for controlling GPIO pins on STM32 microcontrollers
 */
class Gpio {
    public:
        /**
         * @brief Configuration structure for GPIO pin
         */
        struct Config {
            GPIO_TypeDef* port;
            uint16_t      pin;
        };

        /**
         * @brief Constructor for the Gpio class
         *
         * @param config Configuration for the GPIO pin
         */
        Gpio(const Config& config);

        /**
         * @brief Read the current state of the GPIO pin
         *
         * @return The current state of the GPIO pin (true for high, false for low)
         */
        bool read() const;

        /**
         * @brief Write a new state to the GPIO pin
         *
         * @param pin_state The state to be written (true for high, false for low)
         */
        void write(bool state);

        /**
         * @brief Toggle the state of the GPIO pin
         */
        void toggle();

    private:
        /**
         * @brief The port of the GPIO
         */
        GPIO_TypeDef* port;

        /**
         * @brief The pin number of the GPIO
         */
        uint16_t pin;
};
}  // namespace hal

#endif // MICRAS_HAL_GPIO_HPP
