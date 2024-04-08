/**
 * @file adc_dma.hpp
 *
 * @brief ADC DMA HAL header
 *
 * @date 03/2024
 */

#ifndef MICRAS_HAL_ADC_DMA_HPP
#define MICRAS_HAL_ADC_DMA_HPP

#include <adc.h>
#include <cstdint>

namespace micras::hal {
/**
 * @brief Class to handle ADC peripheral on STM32 microcontrollers using DMA
 */
class AdcDma {
public:
    /**
     * @brief Configuration structure for ADC DMA
     */
    struct Config {
        void (*init_function)();
        ADC_HandleTypeDef* handle;
        uint32_t           max_reading;
        float              reference_voltage;
    };

    /**
     * @brief Construct a new AdcDma object
     *
     * @param config ADC DMA configuration struct
     */
    explicit AdcDma(const Config& config);

    /**
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA
     *
     * @param buffer Destination Buffer address
     * @param size Number of data to be transferred from ADC DMA peripheral to memory
     */
    void start_dma(uint32_t buffer[], uint32_t size);  // NOLINT(*-avoid-c-arrays)

    /**
     * @brief Stop ADC conversion of regular group (and injected group in case of auto_injection mode)
     */
    void stop_dma();

    /**
     * @brief Maximum ADC reading
     */
    const uint32_t max_reading;  // NOLINT(*-non-private-member-variables-in-classes)

    /**
     * @brief Reference voltage for the ADC measurement
     */
    const float reference_voltage;  // NOLINT(*-non-private-member-variables-in-classes)

private:
    /**
     * @brief ADC handle
     */
    ADC_HandleTypeDef* handle;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ADC_DMA_HPP
