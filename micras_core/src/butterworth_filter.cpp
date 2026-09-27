/**
 * @file
 */

#include <cmath>
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

    this->damping = std::get<1>(coefficients.feedback);
    this->gain = std::get<0>(coefficients.feed_forward);
}

float ButterworthFilter::update(float x0) {
    const float input_sum = x0 + 2.0F * std::get<0>(this->inputs) + std::get<1>(this->inputs);

    this->rate = this->damping * this->rate + this->gain * (input_sum - 4.0F * this->output);
    this->output += this->rate;

    std::get<1>(this->inputs) = std::get<0>(this->inputs);
    std::get<0>(this->inputs) = x0;

    return this->output;
}

float ButterworthFilter::get_last() const {
    return this->output;
}
}  // namespace micras::core
