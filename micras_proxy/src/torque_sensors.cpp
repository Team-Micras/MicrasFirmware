/**
 * @file torque_sensors.cpp
 *
 * @brief Proxy TorqueSensors class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_CPP
#define MICRAS_PROXY_TORQUE_SENSORS_CPP

#include "micras/proxy/torque_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TorqueSensors<num_of_sensors>::TorqueSensors(const Config& config) :
    adc{config.adc}, shunt_resistor{config.shunt_resistor}, max_torque{config.max_torque} {
    this->adc.start_dma(this->buffer.data(), num_of_sensors);
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_torque(uint8_t sensor_index) const {
    return this->buffer.at(sensor_index) * TorqueSensors::max_torque / this->adc.max_reading;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->adc.reference_voltage * this->buffer.at(sensor_index) / (this->adc.max_reading * this->shunt_resistor);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_TORQUE_SENSORS_CPP
