/**
 * @file
 */

#include <bit>

#include "micras/hal/adc_dma.hpp"

namespace micras::hal {
AdcDma::AdcDma(const Config& config) : max_reading{config.max_reading}, handle{config.handle}
{
    config.init_function();
    HAL_ADCEx_Calibration_Start(this->handle, ADC_SINGLE_ENDED);
}

void AdcDma::start_dma( std::span<uint32_t> buffer) {
    HAL_ADC_Start_DMA(this->handle, buffer.data(), buffer.size());
}

void AdcDma::start_dma(std::span<uint16_t> buffer) {
    this->start_dma({std::bit_cast<uint32_t*>(buffer.data()), buffer.size()});
}

void AdcDma::stop_dma() {
    HAL_ADC_Stop_DMA(this->handle);
}

uint16_t AdcDma::get_max_reading() const {
    return this->max_reading;
}
}  // namespace micras::hal
