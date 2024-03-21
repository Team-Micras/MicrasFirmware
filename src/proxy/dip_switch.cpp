/**
 * @file dip_switch.cpp
 *
 * @brief Proxy DIP Switch class source
 *
 * @date 03/2024
 */

#ifndef __DIP_SWITCH_CPP__
#define __DIP_SWITCH_CPP__

#include "hal/mcu.hpp"
#include "proxy/dip_switch.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
DipSwitch<num_of_sensors>::DipSwitch(Config& dip_switch_config) :
    gpio_array([&]<std::size_t... I> (std::index_sequence<I...>) -> std::array<hal::Gpio, num_of_sensors> {
        return {hal::Gpio{dip_switch_config.gpio_array[I]}...};
    }(std::make_index_sequence<num_of_sensors>())) {
}

template <uint8_t num_of_sensors>
bool DipSwitch<num_of_sensors>::get_switch_state(uint8_t switch_index) const {
    return gpio_array.at(switch_index).read();
}

template <uint8_t num_of_sensors>
uint8_t DipSwitch<num_of_sensors>::get_switches_value() const {
    uint8_t switches_value = 0;

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        switches_value |= (gpio_array[i].read() << i);
    }

    return switches_value;
}
}  // namespace proxy

#endif // __DIP_SWITCH_CPP__
