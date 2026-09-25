/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_TPP
#define MICRAS_PROXY_WALL_SENSORS_TPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iterator>

#include "micras/core/butterworth_filter.hpp"
#include "micras/core/utils.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    adc{config.adc},
    led_pwms{core::make_array<hal::Pwm>(config.led_pwms)},
    emitter_duty_cycle{config.emitter_duty_cycle},
    fast_filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.fast_filter)},
    slow_filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.slow_filter)},
    reference_readings{config.reference_readings},
    reference_distances{config.reference_distances},
    receiver_offset{config.receiver_offset},
    receiver_half_angle{config.receiver_half_angle},
    noise_floor{config.noise_floor},
    max_distance{config.max_distance},
    max_reading{config.max_reading},
    wall_distance{config.wall_distance},
    wall_hysteresis{config.wall_hysteresis},
    calibration_samples{config.calibration_samples},
    initialized{
        this->adc.start_dma(this->buffer, this->snapshot) and this->adc.was_initialized() and
        config.adc.handle->Init.NbrOfConversion == num_of_sensors and
        std::ranges::all_of(this->led_pwms, [&config](const hal::Pwm& led_pwm) {
            return led_pwm.was_initialized() and
                   std::abs(led_pwm.get_frequency() - config.fast_filter.sampling_frequency) <
                       frequency_tolerance * config.fast_filter.sampling_frequency;
        })
    } {
    if (this->receiver_offset > 0.0F) {
        const float step = this->max_distance / static_cast<float>(4 * shape_points);
        float       peak = step;

        for (float distance = step; distance < this->max_distance; distance += step) {
            if (this->shape(distance) > this->shape(peak)) {
                peak = distance;
            }
        }

        const float ratio = std::pow(this->max_distance / peak, 1.0F / static_cast<float>(shape_points - 1));
        float       distance = peak;

        for (uint8_t i = 0; i < shape_points; i++) {
            this->shape_distances.at(i) = distance;
            this->shape_scales.at(i) = 1.0F / std::sqrt(this->shape(distance));
            distance *= ratio;
        }
    }

    this->turn_off();
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::shape(float distance) const {
    const float angle = std::atan(this->receiver_offset / distance) / this->receiver_half_angle;
    return std::exp2(-angle * angle) / (distance * distance);
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::to_distance(uint8_t sensor_index, float intensity) const {
    const float reference_distance = this->reference_distances.at(sensor_index);
    const float ratio = this->reference_readings.at(sensor_index) / intensity;

    if (this->receiver_offset <= 0.0F) {
        return reference_distance * std::sqrt(ratio);
    }

    const float scale = std::sqrt(ratio) / std::sqrt(this->shape(reference_distance));

    if (scale <= this->shape_scales.front()) {
        return this->shape_distances.front();
    }

    if (scale >= this->shape_scales.back()) {
        return this->max_distance;
    }

    const auto  upper = std::ranges::upper_bound(this->shape_scales, scale);
    const auto  index = static_cast<uint8_t>(std::distance(this->shape_scales.begin(), upper));
    const float low = this->shape_scales.at(index - 1);
    const float fraction = (scale - low) / (this->shape_scales.at(index) - low);

    return this->shape_distances.at(index - 1) +
           fraction * (this->shape_distances.at(index) - this->shape_distances.at(index - 1));
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::turn_on() {
    for (auto& led_pwm : this->led_pwms) {
        led_pwm.set_duty_cycle(this->emitter_duty_cycle);
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
    const uint32_t current_sequence = this->adc.read_snapshot(this->scans);
    const bool     is_new = current_sequence != this->sequence;

    this->sequence = current_sequence;

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        Reading& reading = this->readings.at(i);
        reading.is_new = is_new;

        if (not is_new) {
            continue;
        }

        const float intensity = this->get_intensity(i);

        const float distance = intensity >= this->noise_floor ?
                                   this->to_distance(i, std::min(intensity, this->max_reading)) :
                                   this->max_distance;

        reading.valid = distance < this->max_distance;
        reading.distance = this->fast_filters.at(i).update(std::min(distance, this->max_distance));
        reading.slow_distance = this->slow_filters.at(i).update(std::min(distance, this->max_distance));

        if (not reading.valid or reading.slow_distance > this->wall_distance + this->wall_hysteresis) {
            this->walls.at(i) = false;
        } else if (reading.slow_distance < this->wall_distance) {
            this->walls.at(i) = true;
        }

        Calibration& calibration = this->calibrations.at(i);

        if (calibration.samples_left == 0) {
            continue;
        }

        calibration.sum += intensity;
        calibration.squared_sum += intensity * intensity;
        calibration.samples_left--;

        if (calibration.samples_left == 0) {
            const float mean = calibration.sum / static_cast<float>(this->calibration_samples);
            const float variance =
                calibration.squared_sum / static_cast<float>(this->calibration_samples) - mean * mean;

            calibration.spread = mean > 0.0F ? std::sqrt(std::max(variance, 0.0F)) / mean : 0.0F;
            this->reference_readings.at(i) = std::max(mean, this->noise_floor);
        }
    }
}

template <uint8_t num_of_sensors>
const typename TWallSensors<num_of_sensors>::Reading&
    TWallSensors<num_of_sensors>::get_reading(uint8_t sensor_index) const {
    return this->readings.at(sensor_index);
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::get_wall(uint8_t sensor_index) const {
    return this->walls.at(sensor_index);
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_intensity(uint8_t sensor_index) const {
    return static_cast<float>(std::abs(this->scans.at(sensor_index) - this->scans.at(sensor_index + num_of_sensors))) /
           this->adc.get_max_reading();
}

template <uint8_t num_of_sensors>
void TWallSensors<num_of_sensors>::calibrate_sensor(uint8_t sensor_index) {
    this->calibrations.at(sensor_index) = {
        .sum = 0.0F,
        .squared_sum = 0.0F,
        .samples_left = this->calibration_samples,
        .spread = 0.0F,
    };
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::is_calibrating() const {
    return std::ranges::any_of(this->calibrations, [](const Calibration& calibration) {
        return calibration.samples_left > 0;
    });
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_reference_reading(uint8_t sensor_index) const {
    return this->reference_readings.at(sensor_index);
}

template <uint8_t num_of_sensors>
float TWallSensors<num_of_sensors>::get_calibration_spread(uint8_t sensor_index) const {
    return this->calibrations.at(sensor_index).spread;
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_TPP
