/**
 * @file torque_sensors.cpp
 *
 * @brief Proxy TorqueSensors class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_CPP
#define MICRAS_PROXY_TORQUE_SENSORS_CPP

#include "micras/core/utils.hpp"
#include "micras/hal/timer.hpp"
#include "micras/proxy/torque_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TorqueSensors<num_of_sensors>::TorqueSensors(const Config& config) :
    adc{config.adc},
    max_current{hal::AdcDma::reference_voltage / config.shunt_resistor},
    max_torque{config.max_torque},
    filters{core::make_array<core::ButterworthFilter<>, num_of_sensors>(config.filter_cutoff)} {
    this->adc.start_dma(this->buffer);

    for (uint8_t i = 0; i < 50; i++) {
        hal::Timer::sleep_ms(2);
        this->update();
    }

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->base_reading[i] = this->filters[i].get_last();
    }
}

template <uint8_t num_of_sensors>
void TorqueSensors<num_of_sensors>::update() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters[i].update(this->get_adc_reading(i));
    }
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_torque(uint8_t sensor_index) const {
    return this->filters.at(sensor_index).get_last() * this->max_torque;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_torque_raw(uint8_t sensor_index) const {
    return this->get_adc_reading(sensor_index) * this->max_torque;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_current(uint8_t sensor_index) const {
    return this->filters.at(sensor_index).get_last() * this->max_current;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_current_raw(uint8_t sensor_index) const {
    return this->get_adc_reading(sensor_index) * this->max_current;
    ;
}

template <uint8_t num_of_sensors>
float TorqueSensors<num_of_sensors>::get_adc_reading(uint8_t sensor_index) const {
    return static_cast<float>(this->buffer.at(sensor_index)) / this->adc.get_max_reading() -
           this->base_reading.at(sensor_index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_TORQUE_SENSORS_CPP
