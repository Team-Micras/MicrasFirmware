/**
 * @file adc_dma.cpp
 *
 * @brief STM32 ADC DMA HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/adc_dma.hpp"

namespace hal {
AdcDma::AdcDma(const Config& config) :
    max_reading{config.max_reading}, reference_voltage{config.reference_voltage}, handle{config.handle} {
    config.init_function();
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void AdcDma::start_dma(uint32_t buffer[], uint32_t size) {
    HAL_ADC_Start_DMA(this->handle, buffer, size);
}

void AdcDma::stop_dma() {
    HAL_ADC_Stop_DMA(this->handle);
}
}  // namespace hal
