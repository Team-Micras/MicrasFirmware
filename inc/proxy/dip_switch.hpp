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
        struct Config {
            std::array<hal::Gpio::Config, num_of_switches> gpio_array;
        };

        DipSwitch(const Config& dip_switch_config);

        bool get_switch_state(uint8_t switch_index);

        uint8_t get_switches_value();

    private:
        std::array<hal::Gpio, num_of_switches> gpio_array;
};
}  // namespace proxy

#endif // __DIP_SWITCH_HPP__
