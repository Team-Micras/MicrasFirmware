/**
 * @file torque_sensors.cpp
 *
 * @brief Proxy TorqueSensors class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_CPP
#define MICRAS_PROXY_TORQUE_SENSORS_CPP

#include "proxy/torque_sensors.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
TorqueSensors<num_of_sensors>::TorqueSensors(const Config& config) :
    current_sensors{config.current_sensors}, max_torque{config.max_torque} {
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_torque(uint8_t sensor_index) const {
    return this->current_sensors.get_current_raw(sensor_index) * TorqueSensors::max_torque / this->adc.max_reading;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->current_sensors.get_current(sensor_index);
}
}  // namespace proxy

#endif // MICRAS_PROXY_TORQUE_SENSORS_CPP
