/**
 * @file current_sensor_array.cpp
 *
 * @brief Proxy CurrentSensorArray class implementation
 *
 * @date 03/2024
 */

#include "proxy/current_sensor_array.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
CurrentSensorArray<num_of_sensors>::CurrentSensorArray(const Config& current_sensor_config) : current_sensor_adc(
        current_sensor_config.adc_config) {
    this->current_sensor_adc.start_dma(this->adc_buffer, num_of_sensors);
}

template <uint8_t num_of_sensors>
uint16_t CurrentSensorArray<num_of_sensors>::get_current(uint8_t sensor_index) {
    return this->adc_buffer[sensor_index];
}
}  // namespace proxy
