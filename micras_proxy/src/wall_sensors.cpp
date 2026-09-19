/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_CPP
#define MICRAS_PROXY_WALL_SENSORS_CPP

#include <cstdint>
#include <cstdlib>

#include "micras/core/butterworth_filter.hpp"
#include "micras/core/utils.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    adc{config.adc},
    led_pwms{core::make_array<hal::Pwm>(config.led_pwms)},
    filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.filter)},
    base_readings{config.base_readings},
    uncertainty{config.uncertainty},
    initialized{
        this->adc.start_dma(this->buffer) and this->adc.was_initialized() and
        config.adc.handle->Init.NbrOfConversion == num_of_sensors
    } {
    this->turn_off();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_on() {
    for (auto& led_pwm : this->led_pwms) {
        led_pwm.set_duty_cycle(50.0F);
    }
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_off() {
    for (auto& led_pwm : this->led_pwms) {
        led_pwm.set_duty_cycle(0.0F);
    }
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::update() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters.at(i).update(this->get_adc_reading(i));
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
    return static_cast<float>(
               std::abs(this->buffer.at(sensor_index) - this->buffer.at(sensor_index + num_of_sensors))
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

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_CPP
