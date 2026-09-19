/**
 * @file
 */

#ifndef MICRAS_CORE_BUTTERWORTH_FILTER_HPP
#define MICRAS_CORE_BUTTERWORTH_FILTER_HPP

#include <array>
#include <cstdint>

namespace micras::core {
/**
 * @brief Implementation of Butterworth second order low-pass filter.
 *
 * @details
 * A generic digital filter follows the relation:
 *   a0 * y[k] = sum(bi * x[k - i]) - sum(aj * y[k - j])
 * Where:
 *   x[k] - measurement at instant k
 *   y[k] - filtered signal at instant k
 * The Butterworth filter have the special property of being a
 * maximally flat magnitude filter, in other words, is the best
 * filter that doesn't present distortions around the cutoff
 * frequency.
 *
 * @see
 * The formula for the continuos coefficients of the Butterworth
 * filter is available here:
 * https://en.wikipedia.org/wiki/Butterworth_filter
 * The discrete version were computed with the Tustin method:
 * https://en.wikipedia.org/wiki/Bilinear_transform
 */
class ButterworthFilter {
public:
    /**
     * @brief Order of the filter.
     */
    static constexpr uint8_t filter_order{2};

    /**
     * @brief Configuration struct for the filter.
     *
     * @note Both frequencies are in hertz, and the cutoff is the -3 dB point of the resulting
     * discrete filter, exactly, at any ratio up to Nyquist.
     *
     * @note The sampling frequency has no default on purpose: the discrete coefficients depend on
     * the ratio of the cutoff to the sampling frequency, so a filter that assumes the wrong rate is
     * silently a filter with the wrong cutoff.
     */
    struct Config {
        float cutoff_frequency;
        float sampling_frequency;
    };

    /**
     * @brief Coefficients of the discrete transfer function.
     *
     * @note Written in the natural order, most recent sample first, for the relation
     * y[k] = sum(feed_forward[i] * x[k - i]) - sum(feedback[j] * y[k - 1 - j]).
     */
    struct Coefficients {
        std::array<float, filter_order + 1> feed_forward;
        std::array<float, filter_order>     feedback;
    };

    /**
     * @brief Compute the discrete coefficients of the filter.
     *
     * @param config Cutoff and sampling frequencies in Hz.
     * @return Coefficients of the discrete transfer function.
     */
    static Coefficients compute_coefficients(const Config& config);

    /**
     * @brief Construct a new Butterworth Second Order filter object.
     *
     * @param config Cutoff and sampling frequencies in Hz.
     */
    explicit ButterworthFilter(const Config& config);

    /**
     * @brief Produce a new value from measured data.
     *
     * @param x0 Last measure.
     * @return Filtered value.
     */
    float update(float x0);

    /**
     * @brief Get the last filtered value.
     *
     * @return Last filtered value.
     */
    float get_last() const;

private:
    /**
     * @brief Last input values of the filter.
     */
    std::array<float, filter_order + 1> x_array{};

    /**
     * @brief Last output values of the filter.
     */
    std::array<float, filter_order> y_array{};

    /**
     * @brief Coefficients of the filter related to the output value, oldest first.
     */
    std::array<float, filter_order> a_array{};

    /**
     * @brief Coefficients of the filter related to the input value, oldest first.
     */
    std::array<float, filter_order + 1> b_array{};
};
}  // namespace micras::core

#endif  // MICRAS_CORE_BUTTERWORTH_FILTER_HPP
