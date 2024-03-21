/**
 * @file distance_sensor.cpp
 *
 * @brief Proxy DistanceSensors class implementation
 *
 * @date 03/2024
 */

#include "proxy/distance_sensors.hpp"

namespace proxy {
static constexpr float max_distance{0.3f};

static constexpr uint32_t max_adc_reading{4095};

template <uint8_t num_of_sensors>
DistanceSensors<num_of_sensors>::DistanceSensors(const Config& distance_sensor_config) :
    distance_sensor_adc{distance_sensor_config.sensor_adc_config},
    infrared_pwm{distance_sensor_config.infrared_pwm} {
    this->distance_sensor_adc.start_dma(this->adc_buffer.data(), num_of_sensors);
    this->infrared_pwm.set_duty_cycle(100.0f);
}

template <uint8_t num_of_sensors>
void DistanceSensors<num_of_sensors>::set_infrared_led_intensity(float intensity) {
    this->infrared_pwm.set_duty_cycle(intensity);
}

template <uint8_t num_of_sensors>
float DistanceSensors<num_of_sensors>::get_distance(uint8_t sensor_index) {
    return max_distance * this->adc_buffer.at(sensor_index) / max_adc_reading;
}

template <uint8_t num_of_sensors>
uint32_t DistanceSensors<num_of_sensors>::get_distance_raw(uint8_t sensor_index) {
    return this->adc_buffer.at(sensor_index);
}
}  // namespace proxy
