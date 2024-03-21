/**
 * @file dip_switch.cpp
 *
 * @brief Proxy DIP Switch class source
 *
 * @date 03/2024
 */

#include "hal/mcu.hpp"
#include "proxy/dip_switch.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
DipSwitch<num_of_sensors>::DipSwitch(Config& dip_switch_config) {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->gpio_array[i] = dip_switch_config.gpio_array[i];
    }
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
