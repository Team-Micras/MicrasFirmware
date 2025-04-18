/**
 * @file
 */

#ifndef MICRAS_PROXY_BUTTON_HPP
#define MICRAS_PROXY_BUTTON_HPP

#include <cstdint>

#include "micras/hal/gpio.hpp"
#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring button data.
 */
class Button {
public:
    /**
     * @brief Enum for button status.
     */
    enum Status : uint8_t {
        NO_PRESS,
        SHORT_PRESS,
        LONG_PRESS,
        EXTRA_LONG_PRESS
    };

    /**
     * @brief Enum for button pull resistor.
     */
    enum PullResistor : uint8_t {
        PULL_UP,
        PULL_DOWN,
    };

    /**
     * @brief Configuration struct for the button.
     */
    struct Config {
        hal::Gpio::Config gpio{};
        PullResistor      pull_resistor{};
        uint16_t          debounce_delay{10};
        uint16_t          long_press_delay{500};
        uint16_t          extra_long_press_delay{2000};
    };

    /**
     * @brief Construct a new Button object.
     *
     * @param config Button configuration.
     */
    explicit Button(const Config& config);

    /**
     * @brief Check if button is pressed.
     *
     * @return True if button is pressed, false otherwise.
     */
    bool is_pressed();

    /**
     * @brief Get button status.
     *
     * @return Current button status.
     */
    Status get_status();

private:
    /**
     * @brief Get raw button reading.
     *
     * @return Button reading without debounce.
     */
    bool get_raw_reading() const;

    /**
     * @brief Update button state.
     *
     * @return Current state of the button.
     */
    bool update_state();

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

    /**
     * @brief Button pressing delays in ms.
     */
    ///@{
    uint16_t debounce_delay;
    uint16_t long_press_delay;
    uint16_t extra_long_press_delay;
    ///@}

    /**
     * @brief Gpio object for button.
     */
    hal::Gpio gpio;

    /**
     * @brief Pull resistor configuration.
     */
    PullResistor pull_resistor;

    /**
     * @brief Stopwatch to check if button is debouncing.
     */
    proxy::Stopwatch debounce_stopwatch;

    /**
     * @brief Stopwatch to determine type of button press.
     */
    proxy::Stopwatch status_stopwatch;

    /**
     * @brief Flag to know when button is debouncing.
     */
    bool is_debouncing{false};

    /**
     * @brief Flag to know if button was being pressed.
     */
    bool previous_state{false};

    /**
     * @brief Flag to know if button is being pressed.
     */
    bool current_state{false};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUTTON_HPP
