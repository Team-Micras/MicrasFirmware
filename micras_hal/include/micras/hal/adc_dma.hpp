/**
 * @file
 */

#ifndef MICRAS_HAL_ADC_DMA_HPP
#define MICRAS_HAL_ADC_DMA_HPP

#include <adc.h>
#include <cstdint>
#include <span>

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
     */
    void start_dma(std::span<uint32_t> buffer);

    /**
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA
     *
     * @param buffer Destination Buffer address
     */
    void start_dma(std::span<uint16_t> buffer);

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
     * @brief Reference voltage for the ADC measurement
     */
    static constexpr float reference_voltage{3.3F};

private:
    /**
     * @brief Maximum ADC reading
     */
    uint16_t max_reading;

    /**
     * @brief ADC handle
     */
    ADC_HandleTypeDef* handle;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ADC_DMA_HPP
