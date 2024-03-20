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
#include <stdint.h>

namespace proxy {
/**
 * @brief Class for controlling buttons.
 */
class Button {
    public:
        /**
         * @brief Enum for button status.
         */
        enum Status {
            NO_PRESS,
            SHORT_PRESS,
            LONG_PRESS,
            EXTRA_LONG_PRESS
        };

        /**
         * @brief Enum for button pull resistor.
         */
        enum PullResistor {
            PULL_UP,
            PULL_DOWN,
        };

        /**
         * @brief Configuration structure for button.
         */
        struct Config {
            hal::Gpio::Config gpio_config;
            PullResistor      pull_resistor;
            uint16_t          debounce_delay_ms = 10;
            uint16_t          long_press_delay_ms = 1000;
            uint16_t          extra_long_press_delay_ms = 5000;
        };

        /**
         * @brief Constructor for Button class.
         *
         * @param button_config Button configuration.
         */
        Button(const Config& button_config);

        /**
         * @brief Get button status.
         *
         * @return Button status.
         */
        Status get_status();

        /**
         * @brief Check if button is pressed.
         *
         * @return True if button is pressed, false otherwise.
         */
        bool is_pressed();

        /**
         * @brief Check if button is released.
         *
         * @return True if button is released, false otherwise.
         */
        bool is_released();

        /**
         * @brief Check if button was just pressed.
         *
         * @return True if button was just pressed, false otherwise.
         */
        bool is_rising_edge() const;

        /**
         * @brief Check if button was just released.
         *
         * @return True if button was just released, false otherwise.
         */
        bool is_falling_edge() const;

    private:
        /**
         * @brief Update button state.
         */
        void update_state();

        /**
         * @brief Get raw button reading.
         *
         * @return Button reading.
         */
        bool get_raw_reading() const;

        /**
         * @brief Button pressing delays.
         */
        const uint16_t debounce_delay_ms;
        const uint16_t long_press_delay_ms;
        const uint16_t extra_long_press_delay_ms;

        /**
         * @brief Gpio object for button.
         */
        hal::Gpio gpio;

        /**
         * @brief pull resistor configuration
         */
        PullResistor pull_resistor;

        /**
         * @brief timer to check if button is debouncing
         */
        uint32_t debounce_timer = 0;

        /**
         * @brief timer to determine type of button press
         */
        uint32_t status_timer = 0;

        /**
         * @brief flag to know when button is debouncing
         */
        bool is_debouncing = false;

        /**
         * @brief flag to know if button was being pressed
         */
        bool previous_state = false;

        /**
         * @brief flag to know if button is being pressed
         */
        bool current_state = false;
};  // class Button
}  // namespace proxy

#endif // __BUTTON_HPP__
