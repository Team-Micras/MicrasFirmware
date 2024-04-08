/**
 * @file distance_sensor.cpp
 *
 * @brief Proxy DistanceSensors class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_DISTANCE_SENSORS_CPP
#define MICRAS_PROXY_DISTANCE_SENSORS_CPP

#include "micras/proxy/distance_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
DistanceSensors<num_of_sensors>::DistanceSensors(const Config& config) :
    adc{config.adc}, led0_pwm{config.led0_pwm}, led1_pwm{config.led1_pwm}, max_distance{config.max_distance} {
    this->adc.start_dma(this->buffer.data(), num_of_sensors);
    this->led0_pwm.set_duty_cycle(50.0F);
    this->led1_pwm.set_duty_cycle(50.0F);
}

template <uint8_t num_of_sensors>
void DistanceSensors<num_of_sensors>::set_led_intensity(float intensity) {
    this->led0_pwm.set_duty_cycle(intensity);
    this->led1_pwm.set_duty_cycle(intensity);
}

template <uint8_t num_of_sensors>
float DistanceSensors<num_of_sensors>::get_distance(uint8_t sensor_index) const {
    return this->max_distance * this->buffer.at(sensor_index) / this->adc.max_reading;
}

template <uint8_t num_of_sensors>
uint32_t DistanceSensors<num_of_sensors>::get_distance_raw(uint8_t sensor_index) const {
    return this->buffer.at(sensor_index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_DISTANCE_SENSORS_CPP
