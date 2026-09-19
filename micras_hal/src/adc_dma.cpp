/**
 * @file
 */

#include <bit>
#include <cstdint>
#include <span>

#include "micras/hal/adc_dma.hpp"

namespace micras::hal {
AdcDma::AdcDma(const Config& config) :
    max_reading{config.max_reading}, reference_voltage{config.reference_voltage}, handle{config.handle} {
    if (this->handle->State == HAL_ADC_STATE_RESET) {
        config.init_function();
    }

#ifdef STM32H7
    const HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(this->handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
#elifdef STM32G4
    const HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(this->handle, ADC_SINGLE_ENDED);
#else
    #error "ADC calibration is only supported for STM32H7 and STM32G4 platforms."
#endif

    this->initialized = status == HAL_OK;
}

bool AdcDma::start_dma(std::span<uint32_t> buffer) {
    if (HAL_ADC_Start_DMA(this->handle, buffer.data(), buffer.size()) != HAL_OK) {
        this->initialized = false;
        return false;
    }

    return true;
}

bool AdcDma::start_dma(std::span<uint16_t> buffer) {
    return this->start_dma({std::bit_cast<uint32_t*>(buffer.data()), buffer.size()});
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

bool AdcDma::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
