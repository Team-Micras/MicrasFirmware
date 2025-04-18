/**
 * @file
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_HPP
#define MICRAS_PROXY_DIP_SWITCH_HPP

#include <array>
#include <cstdint>

#include "micras/hal/gpio.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring dip switch data.
 */
template <uint8_t num_of_switches>
class TDipSwitch {
public:
    /**
     * @brief Configuration struct for the Dip Switch.
     */
    struct Config {
        std::array<hal::Gpio::Config, num_of_switches> gpio_array;
    };

    /**
     * @brief Construct a new Dip Switch object.
     *
     * @param config Configuration struct for the DipSwitch.
     */
    explicit TDipSwitch(const Config& config);

    /**
     * @brief Get the state of a switch.
     *
     * @param switch_index Index of the switch.
     * @return True if the switch is on, false otherwise.
     */
    bool get_switch_state(uint8_t switch_index) const;

    /**
     * @brief Get the value of all switches.
     *
     * @return Value of all switches.
     */
    uint8_t get_switches_value() const;

private:
    /**
     * @brief Array of GPIOs for the switches.
     */
    std::array<hal::Gpio, num_of_switches> gpio_array;
};
}  // namespace micras::proxy

#include "../src/dip_switch.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_DIP_SWITCH_HPP
