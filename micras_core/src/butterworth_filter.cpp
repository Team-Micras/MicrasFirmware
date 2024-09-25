#ifndef MICRAS_CORE_BUTTERWORTH_FILTER_CPP
#define MICRAS_CORE_BUTTERWORTH_FILTER_CPP

#include <cmath>
#include <numbers>

#include "micras/core/butterworth_filter.hpp"

namespace micras::core {
template <uint8_t filter_order>
ButterworthFilter<filter_order>::ButterworthFilter(float cutoff_frequency, float sampling_frequency) {
    float relative_frequency = cutoff_frequency / sampling_frequency;
    float relative_frequency_2 = relative_frequency * relative_frequency;

    float b0 = 1;
    float b1 = 2;
    float b2 = 1;

    // Butterworth filter coefficients
    float a0 = 1 + 2 * std::numbers::sqrt2_v<float> / relative_frequency + 4 / relative_frequency_2;
    float a1 = 2 - 8 / relative_frequency_2;
    float a2 = 1 - 2 * std::numbers::sqrt2_v<float> / relative_frequency + 4 / relative_frequency_2;

    this->a_array[0] = a2 / a0;
    this->a_array[1] = a1 / a0;

    this->b_array[0] = b2 / a0;
    this->b_array[1] = b1 / a0;
    this->b_array[2] = b0 / a0;
}

template <uint8_t filter_order>
float ButterworthFilter<filter_order>::update(float x0) {
    this->x_array[0] = this->x_array[1];
    this->x_array[1] = this->x_array[2];
    this->x_array[2] = x0;

    float x_b_dot = 0;

    for (int i = 0; i < filter_order + 1; i++) {
        x_b_dot += this->x_array[i] * this->b_array[i];  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    float y_a_dot = 0;

    for (int i = 0; i < filter_order; i++) {
        y_a_dot += this->y_array[i] * this->a_array[i];  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    float y0 = x_b_dot - y_a_dot;

    this->y_array[0] = this->y_array[1];
    this->y_array[1] = y0;

    return y0;
}

template <uint8_t filter_order>
float ButterworthFilter<filter_order>::get_last() const {
    return this->y_array[1];
}

}  // namespace micras::core

#endif  // MICRAS_CORE_BUTTERWORTH_FILTER_CPP
