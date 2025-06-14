/**
 * @file
 */

#ifndef MICRAS_PROXY_DIP_SWITCH_CPP
#define MICRAS_PROXY_DIP_SWITCH_CPP

#include "micras/core/utils.hpp"
#include "micras/proxy/dip_switch.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TDipSwitch<num_of_sensors>::TDipSwitch(const Config& config) :
    gpio_array{core::make_array<hal::Gpio>(config.gpio_array)} { }

template <uint8_t num_of_sensors>
bool TDipSwitch<num_of_sensors>::get_switch_state(uint8_t switch_index) const {
    return gpio_array.at(switch_index).read();
}

template <uint8_t num_of_sensors>
uint8_t TDipSwitch<num_of_sensors>::get_switches_value() const {
    uint8_t switches_value = 0;

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        switches_value |= (gpio_array[i].read() << i);
    }

    return switches_value;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_DIP_SWITCH_CPP
