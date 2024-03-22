/**
 * @file adc_dma.hpp
 *
 * @brief ADC DMA HAL header
 *
 * @date 03/2024
 */

#ifndef __HAL_ADC_DMA_HPP__
#define __HAL_ADC_DMA_HPP__

#include <adc.h>
#include <cstdint>

namespace hal {
/**
 * @brief Class to handle ADC DMA peripheral on STM32 microcontrollers
 */
class AdcDma {
    public:
        /**
         * @brief Configuration structure for ADC DMA
         */
        struct Config {
            ADC_HandleTypeDef* handle;
            void               (* init_function)(void);
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

        /**
         * @brief Reference voltage for the ADC measurement
         */
        static constexpr float reference_voltage{3.3f};

        /**
         * @brief Maximum ADC reading
         */
        static constexpr uint32_t max_reading{4095};
};
}

#endif // __HAL_ADC_DMA_HPP__
