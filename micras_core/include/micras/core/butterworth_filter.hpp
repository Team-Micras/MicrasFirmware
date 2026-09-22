/**
 * @file
 */

#ifndef MICRAS_CORE_BUTTERWORTH_FILTER_HPP
#define MICRAS_CORE_BUTTERWORTH_FILTER_HPP

#include <array>
#include <cstdint>
#include <numbers>

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
     * @brief Get the delay the filter adds to a signal that changes slowly compared with the cutoff.
     *
     * @note This is the group delay at low frequency, `sqrt(2) / (2 * pi * cutoff)`. Whoever turns a
     * filtered reading into a position has to account for the distance traveled during it.
     *
     * @param cutoff_frequency Cutoff frequency in Hz.
     * @return The delay in seconds.
     */
    static constexpr float get_delay(float cutoff_frequency) {
        return std::numbers::sqrt2_v<float> / (2.0F * std::numbers::pi_v<float> * cutoff_frequency);
    }

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
     * @details The recursion is written for the change of the output instead of the output,
     * `y[k] = y[k-1] + r[k]` with `r[k] = a2 * r[k-1] + b0 * (x[k] + 2 * x[k-1] + x[k-2] - 4 * y[k-1])`,
     * which is the same transfer function. The direct form computes the output as the small
     * difference of two large terms, and gets its unity gain from three rounded coefficients
     * adding up exactly, which single precision stops doing when the cutoff is a thousand times
     * below the sampling rate: a 7.64 Hz filter sampled at 10 kHz comes out with a gain error of
     * 0.35 % and blind to changes of 0.04 %. In this form the gain is one by construction and the
     * same filter is good to 0.001 %.
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
     * @brief Last two input values of the filter, most recent first.
     */
    std::array<float, filter_order> inputs{};

    /**
     * @brief Last output value of the filter.
     */
    float output{};

    /**
     * @brief Change of the output in the last update.
     */
    float rate{};

    /**
     * @brief Share of the rate that is kept from one update to the next, which is the feedback
     * coefficient of the oldest output.
     */
    float damping;

    /**
     * @brief Gain from the inputs to the rate, which is the first feed forward coefficient.
     */
    float gain;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_BUTTERWORTH_FILTER_HPP
