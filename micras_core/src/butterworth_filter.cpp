/**
 * @file
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "micras/core/butterworth_filter.hpp"

namespace micras::core {
ButterworthFilter::ButterworthFilter(float cutoff_frequency, float sampling_frequency) {
    const float relative_frequency = cutoff_frequency / sampling_frequency;
    const float relative_frequency_2 = relative_frequency * relative_frequency;

    const float b0 = 1;
    const float b1 = 2;
    const float b2 = 1;

    // Butterworth filter coefficients
    const float a0 = 1 + 2 * std::numbers::sqrt2_v<float> / relative_frequency + 4 / relative_frequency_2;
    const float a1 = 2 - 8 / relative_frequency_2;
    const float a2 = 1 - 2 * std::numbers::sqrt2_v<float> / relative_frequency + 4 / relative_frequency_2;

    std::get<0>(this->a_array) = a2 / a0;
    std::get<1>(this->a_array) = a1 / a0;

    std::get<0>(this->b_array) = b2 / a0;
    std::get<1>(this->b_array) = b1 / a0;
    std::get<2>(this->b_array) = b0 / a0;
}

float ButterworthFilter::update(float x0) {
    std::get<0>(this->x_array) = std::get<1>(this->x_array);
    std::get<1>(this->x_array) = std::get<2>(this->x_array);
    std::get<2>(this->x_array) = x0;

    float x_b_dot = 0;

    for (uint8_t i = 0; i < filter_order + 1; i++) {
        x_b_dot += this->x_array.at(i) * this->b_array.at(i);
    }

    float y_a_dot = 0;

    for (uint8_t i = 0; i < filter_order; i++) {
        y_a_dot += this->y_array.at(i) * this->a_array.at(i);
    }

    const float y0 = x_b_dot - y_a_dot;

    std::get<0>(this->y_array) = std::get<1>(this->y_array);
    std::get<1>(this->y_array) = y0;

    return y0;
}

float ButterworthFilter::get_last() const {
    return std::get<1>(this->y_array);
}
}  // namespace micras::core
