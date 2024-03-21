/**
 * @file current_sensors.cpp
 *
 * @brief Proxy CurrentSensors class implementation
 *
 * @date 03/2024
 */

#ifndef __CURRENT_SENSORS_CPP__
#define __CURRENT_SENSORS_CPP__

#include "proxy/current_sensors.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
CurrentSensors<num_of_sensors>::CurrentSensors(Config& current_sensors_config) :
    adc{current_sensors_config.adc}, shunt_resistor{current_sensors_config.shunt_resistor} {
    this->adc.start_dma(this->buffer.data(), num_of_sensors);
}

template <uint8_t num_of_sensors>
float CurrentSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->reference_voltage * this->buffer.at(sensor_index) / (this->max_adc_reading * this->shunt_resistor);
}

template <uint8_t num_of_sensors>
uint32_t CurrentSensors<num_of_sensors>::get_current_raw(uint8_t sensor_index) const {
    return this->buffer.at(sensor_index);
}
}  // namespace proxy

#endif // __CURRENT_SENSORS_CPP__
