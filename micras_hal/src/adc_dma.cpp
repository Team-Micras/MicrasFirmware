/**
 * @file adc_dma.cpp
 *
 * @brief STM32 ADC DMA HAL wrapper
 *
 * @date 03/2024
 */

#include <bit>

#include "micras/hal/adc_dma.hpp"

namespace micras::hal {
AdcDma::AdcDma(const Config& config) :
    max_reading{config.max_reading}, reference_voltage{config.reference_voltage}, handle{config.handle} {
    config.init_function();
    HAL_ADCEx_Calibration_Start(this->handle, ADC_SINGLE_ENDED);
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void AdcDma::start_dma(uint32_t buffer[], uint32_t size) {
    HAL_ADC_Start_DMA(this->handle, buffer, size);
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void AdcDma::start_dma(uint16_t buffer[], uint32_t size) {
    this->start_dma(std::bit_cast<uint32_t*>(buffer), size);
}

void AdcDma::stop_dma() {
    HAL_ADC_Stop_DMA(this->handle);
}

uint16_t AdcDma::get_max_reading() const {
    return this->max_reading;
}

float AdcDma::get_reference_voltage() const {
    return this->reference_voltage;
}
}  // namespace micras::hal
