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

namespace hal {
/**
 * @brief Class to handle ADC peripheral on STM32 microcontrollers using DMA
 */
class AdcDma {
    public:
        /**
         * @brief Configuration structure for ADC DMA
         */
        struct Config {
            ADC_HandleTypeDef* handle;
            void               (* init_function)(void);
            uint32_t           max_reading;
            float              reference_voltage;
        };

        /**
         * @brief Construct a new AdcDma object
         *
         * @param config ADC DMA configuration struct
         */
        AdcDma(Config& config);

        /**
         * @brief Enable ADC, start conversion of regular group and transfer result through DMA
         *
         * @param buffer Destination Buffer address
         * @param size Number of data to be transferred from ADC DMA peripheral to memory
         */
        void start_dma(uint32_t buffer[], uint32_t size);

        /**
         * @brief Stop ADC conversion of regular group (and injected group in case of auto_injection mode)
         */
        void stop_dma();

        /**
         * @brief Maximum ADC reading
         */
        const uint32_t max_reading;

        /**
         * @brief Reference voltage for the ADC measurement
         */
        const float reference_voltage;

    private:
        /**
         * @brief ADC handle
         */
        ADC_HandleTypeDef* handle;
};
}

#endif // MICRAS_HAL_ADC_DMA_HPP
