/**
 * @file torque_sensors.cpp
 *
 * @brief Proxy TorqueSensors class implementation
 *
 * @date 03/2024
 */

#ifndef __TORQUE_SENSORS_CPP__
#define __TORQUE_SENSORS_CPP__

#include "proxy/torque_sensors.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
TorqueSensors<num_of_sensors>::TorqueSensors(Config& torque_sensors_config) :
    current_sensors{torque_sensors_config.current_sensors} {
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_torque(uint8_t sensor_index) const {
    return this->current_sensors.get_current_raw(sensor_index) * TorqueSensors::max_torque / hal::AdcDma::max_reading;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->current_sensors.get_current(sensor_index);
}
}  // namespace proxy

#endif // __TORQUE_SENSORS_CPP__
