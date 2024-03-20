/**
 * @file adc_dma.cpp
 *
 * @brief STM32 ADC DMA HAL wrapper.
 *
 * @date 03/2024
 */

#include "dma.h"
#include "hal/adc_dma.hpp"

namespace hal {
/*****************************************
 * Public Methods Bodies Definitions.
 *****************************************/

AdcDma::AdcDma(Config& adc_config) : handle(adc_config.handle) {
    MX_DMA_Init();
    adc_config.init_function();
}

void AdcDma::start_dma(uint32_t* buffer, uint32_t size) {
    HAL_ADC_Start_DMA(this->handle, buffer, size);
}

void AdcDma::stop_dma() {
    HAL_ADC_Stop_DMA(this->handle);
}
}  // namespace hal
