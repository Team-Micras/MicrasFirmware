/**
 * @file
 */

#ifndef MICRAS_HAL_ADC_DMA_HPP
#define MICRAS_HAL_ADC_DMA_HPP

#include <cstdint>
#include <main.h>
#include <span>

namespace micras::hal {
/**
 * @brief Class to handle ADC peripheral on STM32 microcontrollers using DMA.
 */
class AdcDma {
public:
    /**
     * @brief Configuration struct for ADC DMA.
     *
     * @note The reference voltage is a property of the board, not of the silicon, so it is
     * configured per instance next to the resolution instead of being fixed by this class.
     */
    struct Config {
        void (*init_function)();
        ADC_HandleTypeDef* handle;
        uint16_t           max_reading;
        float              reference_voltage;
    };

    /**
     * @brief Construct a new AdcDma object.
     *
     * @param config ADC DMA configuration struct.
     */
    explicit AdcDma(const Config& config);

    /**
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA.
     *
     * @param buffer 32 bit destination buffer address.
     * @return True if the conversion was started, false otherwise.
     */
    bool start_dma(std::span<uint32_t> buffer);

    /**
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA.
     *
     * @param buffer 16 bit destination buffer address.
     * @return True if the conversion was started, false otherwise.
     */
    bool start_dma(std::span<uint16_t> buffer);

    /**
     * @brief Stop ADC conversion of regular group (and injected group in case of auto_injection mode).
     */
    void stop_dma();

    /**
     * @brief Get the maximum reading of the ADC.
     *
     * @return Maximum reading of the ADC.
     */
    uint16_t get_max_reading() const;

    /**
     * @brief Get the reference voltage of the ADC measurement.
     *
     * @return Reference voltage in volts.
     */
    float get_reference_voltage() const;

    /**
     * @brief Check if the ADC was calibrated and started successfully.
     *
     * @note A failed calibration or start leaves the DMA buffer at zero, which reads exactly like
     * a real measurement, so this is the only way to tell one from the other.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Maximum ADC reading.
     */
    uint16_t max_reading;

    /**
     * @brief Reference voltage for the ADC measurement.
     */
    float reference_voltage;

    /**
     * @brief ADC handle.
     */
    ADC_HandleTypeDef* handle;

    /**
     * @brief Flag to check if the ADC was calibrated and started.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ADC_DMA_HPP
