/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_CPP
#define MICRAS_PROXY_WALL_SENSORS_CPP

#include "micras/core/utils.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    adc{config.adc},
    led_0_pwm{config.led_0_pwm},
    led_1_pwm{config.led_1_pwm},
    filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.filter_cutoff)},
    base_readings{config.base_readings},
    uncertainty{config.uncertainty} {
    this->adc.start_dma(this->buffer);
    this->turn_off();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_on() {
    this->led_0_pwm.set_duty_cycle(50.0F);
    this->led_1_pwm.set_duty_cycle(50.0F);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_off() {
    this->led_0_pwm.set_duty_cycle(0.0F);
    this->led_1_pwm.set_duty_cycle(0.0F);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::update() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters[i].update(this->get_adc_reading(i));
    }
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::get_wall(uint8_t sensor_index, bool disturbed) const {
    return this->filters.at(sensor_index).get_last() >
           this->base_readings.at(sensor_index) * this->uncertainty * (disturbed ? 1.2F : 1.0F);
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_reading(uint8_t sensor_index) const {
    return this->filters.at(sensor_index).get_last();
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_adc_reading(uint8_t sensor_index) const {
    return static_cast<float>(std::abs(this->buffer.at(sensor_index) - this->buffer.at(sensor_index + num_of_sensors))
           ) /
           this->adc.get_max_reading();
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_sensor_error(uint8_t sensor_index) const {
    return this->get_reading(sensor_index) - this->base_readings.at(sensor_index);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_sensor(uint8_t sensor_index) {
    this->base_readings.at(sensor_index) = this->get_reading(sensor_index);
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_CPP
