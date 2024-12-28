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
    uncertainty{config.uncertainty},
    wall_threshold{config.wall_threshold},
    free_space_threshold{config.free_threshold} {
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
core::Observation TWallSensors<num_of_sensors>::get_observation(uint8_t sensor_index) const {
    float reading = this->filters.at(sensor_index).get_last();

    if (reading > this->wall_threshold.at(sensor_index)) {
        return core::Observation::WALL;
    }

    if (reading < this->free_space_threshold.at(sensor_index)) {
        return core::Observation::FREE_SPACE;
    }

    return core::Observation::UNKNOWN;
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
void TWallSensors<num_of_sensors>::calibrate_front_wall() {
    this->wall_calibration_measure[0] = this->get_reading(0);
    this->wall_calibration_measure[3] = this->get_reading(3);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_left_wall() {
    this->wall_calibration_measure[1] = this->get_reading(1);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_right_wall() {
    this->wall_calibration_measure[2] = this->get_reading(2);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_front_free_space() {
    this->free_space_calibration_measure[0] = this->get_reading(0);
    this->free_space_calibration_measure[3] = this->get_reading(3);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_left_free_space() {
    this->free_space_calibration_measure[1] = this->get_reading(1);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_right_free_space() {
    this->free_space_calibration_measure[2] = this->get_reading(2);
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::update_thresholds() {
    for (uint8_t i = 0; i < num_of_sensors; i++) {
        float mean_value = (this->wall_calibration_measure.at(i) + this->free_space_calibration_measure.at(i)) / 2.0F;

        float unknown_delta = this->uncertainty *
                              (this->wall_calibration_measure.at(i) - this->free_space_calibration_measure.at(i)) /
                              2.0F;

        this->wall_threshold.at(i) = mean_value + unknown_delta;
        this->free_space_threshold.at(i) = mean_value - unknown_delta;
    }
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_CPP
