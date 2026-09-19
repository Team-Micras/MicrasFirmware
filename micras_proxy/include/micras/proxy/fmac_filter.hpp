/**
 * @file
 */

#ifndef MICRAS_PROXY_FMAC_FILTER_HPP
#define MICRAS_PROXY_FMAC_FILTER_HPP

#include <array>
#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/hal/fmac.hpp"

namespace micras::proxy {
/**
 * @brief Second order Butterworth low-pass filter running on the filter math accelerator.
 *
 * @note Interface compatible with core::ButterworthFilter, so the two are interchangeable at the
 * point of use. Three properties of the accelerator decide whether that substitution is a good
 * idea, and none of them is a property of this class:
 *
 * - The accelerator works in q1.15, so every coefficient has to fall inside [-1, 1) and be large
 *   enough not to quantize to zero. A second order Butterworth has a feed forward coefficient of
 *   roughly (fc / fs)^2 / 4, so at a cutoff far below the sampling rate the coefficients underflow.
 *   At a 960 Hz sampling rate the useful range starts around a 100 Hz cutoff, well above the 5 Hz
 *   and 10 Hz cutoffs the sensors of this robot use. was_initialized reports that refusal instead
 *   of filtering with silently truncated coefficients.
 * - Samples and results are q1.15 too, so the input has to be normalized to [-1, 1). That suits a
 *   ratiometric reading such as an ADC fraction, and not a physical quantity such as a rate in
 *   radians per second.
 * - The accelerator holds one filter configuration at a time. Several instances cannot run
 *   concurrently without reprogramming the coefficient buffer between samples, which costs far
 *   more than the handful of multiply accumulates it would save.
 */
class FmacFilter {
public:
    /**
     * @brief Configuration struct for the filter.
     */
    struct Config {
        hal::Fmac::Config               fmac;
        core::ButterworthFilter::Config filter;
    };

    /**
     * @brief Construct a new FmacFilter object.
     *
     * @param config Configuration for the filter.
     */
    explicit FmacFilter(const Config& config);

    /**
     * @brief Produce a new value from measured data.
     *
     * @param x0 Last measure, normalized to the range [-1, 1).
     * @return Filtered value, or zero if the coefficients could not be represented.
     */
    float update(float x0);

    /**
     * @brief Get the last filtered value.
     *
     * @return Last filtered value.
     */
    float get_last() const;

    /**
     * @brief Check if the coefficients fit the fixed point format and the filter is running.
     *
     * @return True if the filter is usable, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Number of fractional bits of the q1.15 format of the accelerator.
     */
    static constexpr uint8_t fractional_bits{15};

    /**
     * @brief Scale between a float in [-1, 1) and its q1.15 representation.
     */
    static constexpr float fixed_point_scale{static_cast<float>(1U << fractional_bits)};

    /**
     * @brief Convert a coefficient to q1.15, reporting whether it survives the conversion.
     *
     * @param value Coefficient to convert.
     * @param result Converted coefficient.
     * @return True if the coefficient is inside the representable range and does not quantize to
     * zero, false otherwise.
     */
    static bool to_fixed_point(float value, int16_t& result);

    /**
     * @brief Filter math accelerator running the filter.
     */
    hal::Fmac fmac;

    /**
     * @brief Coefficients applied to the input samples, most recent first.
     */
    std::array<int16_t, core::ButterworthFilter::filter_order + 1> feed_forward{};

    /**
     * @brief Coefficients applied to the previous outputs, most recent first.
     */
    std::array<int16_t, core::ButterworthFilter::filter_order> feedback{};

    /**
     * @brief Last filtered value.
     */
    float last{};

    /**
     * @brief Flag to check if the filter is running.
     */
    bool initialized{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_FMAC_FILTER_HPP
