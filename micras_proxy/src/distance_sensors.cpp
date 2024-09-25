/**
 * @file distance_sensor.cpp
 *
 * @brief Proxy DistanceSensors class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_DISTANCE_SENSORS_CPP
#define MICRAS_PROXY_DISTANCE_SENSORS_CPP

#include "micras/core/utils.hpp"
#include "micras/proxy/distance_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
DistanceSensors<num_of_sensors>::DistanceSensors(const Config& config) :
    adc{config.adc},
    led_0_pwm{config.led_0_pwm},
    led_1_pwm{config.led_1_pwm},
    max_distance{config.max_distance},
    filters{core::make_array<core::ButterworthFilter<>, num_of_sensors>(config.filter_cutoff)} {
    this->adc.start_dma(this->buffer);
    this->led_0_pwm.set_duty_cycle(100.0F);
    this->led_1_pwm.set_duty_cycle(100.0F);
}

template <uint8_t num_of_sensors>
void DistanceSensors<num_of_sensors>::set_led_intensity(float intensity) {
    this->led_0_pwm.set_duty_cycle(intensity);
    this->led_1_pwm.set_duty_cycle(intensity);
}

template <uint8_t num_of_sensors>
void DistanceSensors<num_of_sensors>::update() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filter[i].update(this->buffer[i]);
    }
}

template <uint8_t num_of_sensors>
float DistanceSensors<num_of_sensors>::get_distance(uint8_t sensor_index) const {
    return 30.0F * this->max_distance * this->filters.at(sensor_index).get_last() / this->adc.get_max_reading();
}

template <uint8_t num_of_sensors>
uint32_t DistanceSensors<num_of_sensors>::get_distance_raw(uint8_t sensor_index) const {
    return this->buffer.at(sensor_index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_DISTANCE_SENSORS_CPP
