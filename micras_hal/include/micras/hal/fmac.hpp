/**
 * @file
 */

#ifndef MICRAS_HAL_FMAC_HPP
#define MICRAS_HAL_FMAC_HPP

#include <array>
#include <cstdint>
#include <fmac.h>
#include <span>

namespace micras::hal {
/**
 * @brief Class to handle the filter math accelerator on STM32 microcontrollers.
 *
 * @note The accelerator works in q1.15 fixed point and holds one filter configuration at a time,
 * so a second filter cannot run without reprogramming the coefficient buffer. Both limits are
 * properties of the peripheral, not of this wrapper.
 */
class Fmac {
public:
    /**
     * @brief Configuration struct for the filter math accelerator.
     */
    struct Config {
        void (*init_function)();
        FMAC_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new Fmac object.
     *
     * @param config Configuration for the filter math accelerator.
     */
    explicit Fmac(const Config& config);

    /**
     * @brief Configure the accelerator as an infinite impulse response filter in direct form 1.
     *
     * @param feed_forward Coefficients applied to the input samples, most recent first, in q1.15.
     * @param feedback Coefficients applied to the previous outputs, most recent first, in q1.15.
     * @return True if the filter was configured and started, false otherwise.
     */
    bool configure_iir(std::span<const int16_t> feed_forward, std::span<const int16_t> feedback);

    /**
     * @brief Push one sample through the configured filter and read the matching output.
     *
     * @note The data registers are accessed directly rather than through the streaming API of the
     * vendor HAL, whose per call bookkeeping costs more than the filter it is driving.
     *
     * @param sample Input sample in q1.15.
     * @return Filtered value in q1.15.
     */
    int16_t update(int16_t sample);

    /**
     * @brief Check if the accelerator holds a running filter.
     *
     * @return True if a filter was configured and started, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Number of 16 bit words of the internal memory allocated to each buffer.
     */
    static constexpr uint8_t buffer_size{8};

    /**
     * @brief Base addresses of the three buffers inside the internal memory.
     */
    ///@{
    static constexpr uint8_t input_base_address{0};
    static constexpr uint8_t coefficient_base_address{buffer_size};
    static constexpr uint8_t output_base_address{2 * buffer_size};
    ///@}

    /**
     * @brief Filter math accelerator handle.
     */
    FMAC_HandleTypeDef* handle;

    /**
     * @brief Output buffer registered with the vendor HAL when the filter is started.
     *
     * @note Never read, since update accesses the data register directly, but the HAL refuses to
     * start a filter without one.
     */
    std::array<int16_t, 1> output_buffer{};

    /**
     * @brief Size of the output buffer, as the vendor HAL expects it by reference.
     */
    uint16_t output_buffer_size{1};

    /**
     * @brief Flag to check if a filter is running.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_FMAC_HPP
