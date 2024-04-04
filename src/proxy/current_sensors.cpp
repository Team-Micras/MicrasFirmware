/**
 * @file current_sensors.cpp
 *
 * @brief Proxy CurrentSensors class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_CURRENT_SENSORS_CPP
#define MICRAS_PROXY_CURRENT_SENSORS_CPP

#include "proxy/current_sensors.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
CurrentSensors<num_of_sensors>::CurrentSensors(const Config& config) :
    adc{config.adc}, shunt_resistor{config.shunt_resistor} {
    this->adc.start_dma(this->buffer.data(), num_of_sensors);
}

template <uint8_t num_of_sensors>
float CurrentSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->adc.reference_voltage * this->buffer.at(sensor_index) /
           (this->adc.max_reading * this->shunt_resistor);
}

template <uint8_t num_of_sensors>
uint32_t CurrentSensors<num_of_sensors>::get_current_raw(uint8_t sensor_index) const {
    return this->buffer.at(sensor_index);
}
}  // namespace proxy

#endif // MICRAS_PROXY_CURRENT_SENSORS_CPP
