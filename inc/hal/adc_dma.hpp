/**
 * @file adc_dma.hpp
 *
 * @brief STM32 ADC DMA HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __HAL_ADC_DMA_HPP__
#define __HAL_ADC_DMA_HPP__

#include <cstdint>
#include <functional>

#include "adc.h"

namespace hal {
/**
 * @brief Class to handle ADC DMA peripheral on STM32 microcontrollers
 */
class AdcDma {
    public:
        /**
         * @brief ADC DMA configuration struct
         */
        struct Config {
            ADC_HandleTypeDef*        handle;
            std::function<void(void)> init_function;
        };

        /**
         * @brief Construct a new AdcDma object
         *
         * @param adc_config ADC DMA configuration struct
         */
        AdcDma(Config& adc_config);

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

    private:
        /**
         * @brief ADC DMA handle
         */
        ADC_HandleTypeDef* handle;
};
}

#endif // __HAL_ADC_DMA_HPP__
