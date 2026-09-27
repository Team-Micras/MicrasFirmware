/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/proxy/fmac_filter.hpp"

namespace micras::proxy {
FmacFilter::FmacFilter(const Config& config) : fmac{config.fmac} {
    const core::ButterworthFilter::Coefficients coefficients =
        core::ButterworthFilter::compute_coefficients(config.filter);

    bool representable = true;

    for (std::size_t i = 0; i < coefficients.feed_forward.size(); i++) {
        representable &= to_fixed_point(coefficients.feed_forward.at(i), this->feed_forward.at(i));
    }

    for (std::size_t i = 0; i < coefficients.feedback.size(); i++) {
        representable &= to_fixed_point(-coefficients.feedback.at(i), this->feedback.at(i));
    }

    if (not representable) {
        return;
    }

    this->initialized = this->fmac.configure_iir(this->feed_forward, this->feedback);
}

float FmacFilter::update(float x0) {
    if (not this->initialized) {
        return 0.0F;
    }

    const float clamped = std::clamp(x0, -1.0F, 1.0F - 1.0F / fixed_point_scale);
    const auto  sample = static_cast<int16_t>(std::lround(clamped * fixed_point_scale));

    this->last = static_cast<float>(this->fmac.update(sample)) / fixed_point_scale;

    return this->last;
}

float FmacFilter::get_last() const {
    return this->last;
}

bool FmacFilter::was_initialized() const {
    return this->initialized;
}

bool FmacFilter::to_fixed_point(float value, int16_t& result) {
    const float scaled = value * fixed_point_scale;

    if (std::abs(scaled) >= fixed_point_scale) {
        result = 0;
        return false;
    }

    result = static_cast<int16_t>(std::lround(scaled));

    return result != 0 or value == 0.0F;
}
}  // namespace micras::proxy
