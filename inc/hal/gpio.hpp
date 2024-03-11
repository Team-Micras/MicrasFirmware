/**
 * @file gpio.hpp
 *
 * @brief HAL GPIO class header
 *
 * @date 03/2024
 */

#ifndef __GPIO_HPP__
#define __GPIO_HPP__

#include "gpio.h"

namespace hal {
/**
 * @brief Class for controlling GPIO pins on STM32 microcontrollers
 */
class Gpio {
    private:
        /**
         * @brief Structure representing a GPIO port
         */
        struct GpioPort {
            GPIO_TypeDef* port;
        };

        /**
         * @brief Structure representing a GPIO pin
         */
        struct GpioPin {
            uint16_t pin;
        };

    public:
        /**
         * @brief Configuration structure for GPIO pin
         */
        struct Config {
            GpioPort port;
            GpioPin  pin;
        };

        /**
         * @brief Structure representing GPIO ports
         */
        struct Port {
#ifdef GPIOA
            static constexpr GpioPort A = {GPIOA};
#endif
#ifdef GPIOB
            static constexpr GpioPort B = {GPIOB};
#endif
#ifdef GPIOC
            static constexpr GpioPort C = {GPIOC};
#endif
#ifdef GPIOD
            static constexpr GpioPort D = {GPIOD};
#endif
#ifdef GPIOE
            static constexpr GpioPort E = {GPIOE};
#endif
#ifdef GPIOF
            static constexpr GpioPort F = {GPIOF};
#endif
#ifdef GPIOG
            static constexpr GpioPort G = {GPIOG};
#endif
#ifdef GPIOH
            static constexpr GpioPort H = {GPIOH};
#endif
#ifdef GPIOI
            static constexpr GpioPort I = {GPIOI};
#endif
#ifdef GPIOJ
            static constexpr GpioPort J = {GPIOJ};
#endif
#ifdef GPIOK
            static constexpr GpioPort K = {GPIOK};
#endif
#ifdef GPIOZ
            static constexpr GpioPort Z = {GPIOZ};
#endif
        };

        /**
         * @brief Structure representing GPIO pins
         */
        struct Pin {
#ifdef GPIO_PIN_0
            static constexpr GpioPin P0 = {GPIO_PIN_0};
#endif
#ifdef GPIO_PIN_1
            static constexpr GpioPin P1 = {GPIO_PIN_1};
#endif
#ifdef GPIO_PIN_2
            static constexpr GpioPin P2 = {GPIO_PIN_2};
#endif
#ifdef GPIO_PIN_3
            static constexpr GpioPin P3 = {GPIO_PIN_3};
#endif
#ifdef GPIO_PIN_4
            static constexpr GpioPin P4 = {GPIO_PIN_4};
#endif
#ifdef GPIO_PIN_5
            static constexpr GpioPin P5 = {GPIO_PIN_5};
#endif
#ifdef GPIO_PIN_6
            static constexpr GpioPin P6 = {GPIO_PIN_6};
#endif
#ifdef GPIO_PIN_7
            static constexpr GpioPin P7 = {GPIO_PIN_7};
#endif
#ifdef GPIO_PIN_8
            static constexpr GpioPin P8 = {GPIO_PIN_8};
#endif
#ifdef GPIO_PIN_9
            static constexpr GpioPin P9 = {GPIO_PIN_9};
#endif
#ifdef GPIO_PIN_10
            static constexpr GpioPin P10 = {GPIO_PIN_10};
#endif
#ifdef GPIO_PIN_11
            static constexpr GpioPin P11 = {GPIO_PIN_11};
#endif
#ifdef GPIO_PIN_12
            static constexpr GpioPin P12 = {GPIO_PIN_12};
#endif
#ifdef GPIO_PIN_13
            static constexpr GpioPin P13 = {GPIO_PIN_13};
#endif
#ifdef GPIO_PIN_14
            static constexpr GpioPin P14 = {GPIO_PIN_14};
#endif
#ifdef GPIO_PIN_15
            static constexpr GpioPin P15 = {GPIO_PIN_15};
#endif
        };

        /**
         * @brief Constructor for the Gpio class
         *
         * @param gpio_config Configuration for the GPIO pin
         */
        Gpio(const Config& gpio_config);

        /**
         * @brief Destroy the Gpio object
         */
        ~Gpio() = default;

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
        uint16_t pin;
        GPIO_TypeDef* port;
};
}  // namespace hal

#endif // __GPIO_HPP__
