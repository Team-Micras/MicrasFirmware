/**
 * @file butterworth_filter.hpp
 *
 * @brief Core Butterworth filter class header
 *
 * @date 09/2024
 */

#ifndef MICRAS_CORE_BUTTERWORTH_FILTER_HPP
#define MICRAS_CORE_BUTTERWORTH_FILTER_HPP

#include <array>
#include <cstdint>

namespace micras::core {
/**
 * @brief Implementation of Butterworth second order low-pass filter
 *        A generic digital filter follows the relation
 *          a0 * y[k] = sum(bi * x[k - i]) - sum(aj * y[k - j])
 *        Where
 *          x[k] - measurement at instant k
 *          y[k] - filtered signal at instant k
 *        The Butterworth filter have the special property of being a
 *        maximally flat magnitude filter, in other words, is the best
 *        filter that doesn't present distortions around the cutoff
 *        frequency
 *        The formula for the continuos coefficients of the Butterworth
 *        filter is available here:
 *        https://en.wikipedia.org/wiki/Butterworth_filter
 *        The discrete version were computed with the Tustin method:
 *        https://en.wikipedia.org/wiki/Bilinear_transform
 */
template <uint8_t filter_order = 2>
class ButterworthFilter {
public:
    /**
     * @brief Construct a new Butterworth Second Order filter object
     *
     * @param cutoff_frequency Low-pass cutoff frequency in Hz
     * @param sampling_frequency Sampling frequency in Hz.
     */
    explicit ButterworthFilter(float cutoff_frequency, float sampling_frequency = 1.0F);

    /**
     * @brief Produces a new value from measured data
     *
     * @param x0 Last measure
     *
     * @return Filtered value
     */
    float update(float x0);

    /**
     * @brief Get the last filtered value
     *
     * @return Last filtered value
     */
    float get_last() const;

private:
    /**
     * @brief Last input values of the filter
     */
    std::array<float, filter_order + 1> x_array{};

    /**
     * @brief Last output values of the filter
     */
    std::array<float, filter_order> y_array{};

    /**
     * @brief Coefficients of the filter related to the output value
     */
    std::array<float, filter_order> a_array{};

    /**
     * @brief Coefficients of the filter related to the input value
     */
    std::array<float, filter_order + 1> b_array{};
};
}  // namespace micras::core

#include "../src/butterworth_filter.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_CORE_BUTTERWORTH_FILTER_HPP
