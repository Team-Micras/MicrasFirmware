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
        uint16_t           max_reading;
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
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA
     *
     * @param buffer Destination Buffer address
     * @param size Number of data to be transferred from ADC DMA peripheral to memory
     */
    void start_dma(uint16_t buffer[], uint32_t size);  // NOLINT(*-avoid-c-arrays)

    /**
     * @brief Stop ADC conversion of regular group (and injected group in case of auto_injection mode)
     */
    void stop_dma();

    /**
     * @brief Get the maximum reading of the ADC
     *
     * @return uint16_t Maximum reading of the ADC
     */
    uint16_t get_max_reading() const;

    /**
     * @brief Get the reference voltage for the ADC measurement
     *
     * @return float Reference voltage for the ADC measurement
     */
    float get_reference_voltage() const;

private:
    /**
     * @brief Maximum ADC reading
     */
    uint16_t max_reading;

    /**
     * @brief Reference voltage for the ADC measurement
     */
    float reference_voltage;

    /**
     * @brief ADC handle
     */
    ADC_HandleTypeDef* handle;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ADC_DMA_HPP
