/**
 * @file dip_switch.hpp
 *
 * @brief Proxy Dip Switch class header
 *
 * @date 03/2024
 */

#ifndef __DIP_SWITCH_HPP__
#define __DIP_SWITCH_HPP__

#include <array>
#include <cstdint>

#include "hal/gpio.hpp"

namespace proxy {
/**
 * @brief Class for controlling a dip switch
 */
template <uint8_t num_of_switches>
class DipSwitch {
    public:
        /**
         * @brief Configuration struct for DipSwitch
         */
        struct Config {
            std::array<hal::Gpio::Config, num_of_switches> gpio_array;
        };

        /**
         * @brief Construct a new Dip Switch object
         *
         * @param dip_switch_config Configuration struct for DipSwitch
         */
        DipSwitch(Config& dip_switch_config);

        /**
         * @brief Get the state of a switch
         *
         * @param switch_index Index of the switch
         * @return true if the switch is on
         * @return false if the switch is off
         */
        bool get_switch_state(uint8_t switch_index) const;

        /**
         * @brief Get the value of all switches
         *
         * @return uint8_t Value of all switches
         */
        uint8_t get_switches_value() const;

    private:
        /**
         * @brief Array of GPIOs for the switches
         */
        std::array<hal::Gpio, num_of_switches> gpio_array;
};
}  // namespace proxy

#endif // __DIP_SWITCH_HPP__
