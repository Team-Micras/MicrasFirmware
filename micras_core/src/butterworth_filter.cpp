/**
 * @file
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "micras/core/butterworth_filter.hpp"

namespace micras::core {
ButterworthFilter::Coefficients ButterworthFilter::compute_coefficients(const Config& config) {
    const float warp = 1.0F / std::tan(std::numbers::pi_v<float> * config.cutoff_frequency / config.sampling_frequency);
    const float warp_2 = warp * warp;

    const float b0 = 1;
    const float b1 = 2;
    const float b2 = 1;

    const float a0 = warp_2 + std::numbers::sqrt2_v<float> * warp + 1;
    const float a1 = 2 - 2 * warp_2;
    const float a2 = warp_2 - std::numbers::sqrt2_v<float> * warp + 1;

    return {
        .feed_forward = {b0 / a0, b1 / a0, b2 / a0},
        .feedback = {a1 / a0, a2 / a0},
    };
}

ButterworthFilter::ButterworthFilter(const Config& config) {
    const Coefficients coefficients = compute_coefficients(config);

    std::get<0>(this->a_array) = std::get<1>(coefficients.feedback);
    std::get<1>(this->a_array) = std::get<0>(coefficients.feedback);

    std::get<0>(this->b_array) = std::get<2>(coefficients.feed_forward);
    std::get<1>(this->b_array) = std::get<1>(coefficients.feed_forward);
    std::get<2>(this->b_array) = std::get<0>(coefficients.feed_forward);
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
