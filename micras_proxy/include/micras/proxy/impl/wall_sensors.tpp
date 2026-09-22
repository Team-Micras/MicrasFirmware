/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_TPP
#define MICRAS_PROXY_WALL_SENSORS_TPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string_view>

#include "micras/core/butterworth_filter.hpp"
#include "micras/core/utils.hpp"
#include "micras/core/variable_pool.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::proxy {
template <uint8_t num_of_sensors>
TWallSensors<num_of_sensors>::TWallSensors(const Config& config) :
    adc{config.adc},
    led_pwms{core::make_array<hal::Pwm>(config.led_pwms)},
    emitter_duty_cycle{config.emitter_duty_cycle},
    filters{core::make_array<core::ButterworthFilter, num_of_sensors>(config.filter)},
    base_readings{config.base_readings},
    uncertainty{config.uncertainty},
    initialized{
        this->adc.start_dma(this->buffer, this->snapshot) and this->adc.was_initialized() and
        config.adc.handle->Init.NbrOfConversion == num_of_sensors and
        std::ranges::all_of(this->led_pwms, [&config](const hal::Pwm& led_pwm) {
            return led_pwm.was_initialized() and std::abs(led_pwm.get_frequency() - config.filter.sampling_frequency) <
                                                     frequency_tolerance * config.filter.sampling_frequency;
        })
    } {
    this->turn_off();
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

    this->fresh = current_sequence != this->sequence;
    this->sequence = current_sequence;

    if (not this->fresh) {
        return;
    }

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters.at(i).update(this->get_adc_reading(i));
    }
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::is_new() const {
    return this->fresh;
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
    return static_cast<float>(std::abs(this->scans.at(sensor_index) - this->scans.at(sensor_index + num_of_sensors))) /
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
void TWallSensors<num_of_sensors>::register_variables(core::VariablePool& pool, std::string_view prefix) {
    static constexpr std::array<std::string_view, 8> names{"0", "1", "2", "3", "4", "5", "6", "7"};
    static_assert(num_of_sensors <= names.size(), "Every sensor needs a name literal to register under");

    for (uint8_t i = 0; i < num_of_sensors; i++) {
        this->filters.at(i).register_variables(pool, prefix, names.at(i));
    }
}

template <uint8_t num_of_sensors>
bool TWallSensors<num_of_sensors>::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WALL_SENSORS_TPP
