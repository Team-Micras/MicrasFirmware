/**
 * @file current_sensors.cpp
 *
 * @brief Proxy CurrentSensors class implementation
 *
 * @date 03/2024
 */

#include "proxy/current_sensors.hpp"

namespace proxy {
static constexpr float reference_voltage{3.3f};

static constexpr uint32_t max_adc_reading{4095};

template <uint8_t num_of_sensors>
CurrentSensors<num_of_sensors>::CurrentSensors(Config& current_sensors_config) :
    adc{current_sensors_config.adc_config}, shunt_resistor{current_sensors_config.shunt_resistor} {
    this->adc.start_dma(this->adc_buffer.data(), num_of_sensors);
}

template <uint8_t num_of_sensors>
float CurrentSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return reference_voltage * this->adc_buffer.at(sensor_index) / (max_adc_reading * this->shunt_resistor);
}

template <uint8_t num_of_sensors>
uint32_t CurrentSensors<num_of_sensors>::get_current_raw(uint8_t sensor_index) const {
    return this->adc_buffer.at(sensor_index);
}
}  // namespace proxy
