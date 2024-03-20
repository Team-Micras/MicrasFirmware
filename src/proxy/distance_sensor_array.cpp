/**
 * @file distance_sensor_array.cpp
 *
 * @brief Proxy DistanceSensorArray class implementation
 *
 * @date 03/2024
 */

#include "proxy/distance_sensor_array.hpp"

namespace proxy {
template <uint8_t num_of_sensors>
DistanceSensorArray<num_of_sensors>::DistanceSensorArray(const Config& distance_sensor_config) :
    distance_sensor_adc(distance_sensor_config.sensor_adc_config),
    infrared_led_tim(distance_sensor_config.infrared_tim_config),
    infrared_led_channel(distance_sensor_config.infrared_led_channel) {
    this->distance_sensor_adc.start_dma(this->adc_buffer, num_of_sensors);
    this->infrared_led_tim.pwm_start(this->infrared_led_channel);
    this->infrared_led_tim.set_compare(this->infrared_led_tim.get_autoreload());
}

template <uint8_t num_of_sensors>
void DistanceSensorArray<num_of_sensors>::set_infrared_led_intensity(uint8_t intensity) {
    uint32_t compare = (intensity * this->infrared_led_tim.get_autoreload()) / 255;
    this->infrared_led_tim.set_compare(this->infrared_led_channel, compare);
}

template <uint8_t num_of_sensors>
uint16_t DistanceSensorArray<num_of_sensors>::get_distance(uint8_t sensor_index) {
    return this->adc_buffer[sensor_index];
}
}  // namespace proxy
