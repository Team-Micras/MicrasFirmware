/**
 * @file
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstdint>
#include <span>

#include "micras/hal/adc_dma.hpp"

extern "C" {
/**
 * @brief Callback of the vendor HAL for the end of a DMA transfer of a converter.
 *
 * @param hadc Handle of the converter.
 */
// NOLINTNEXTLINE(readability-identifier-naming) the name is fixed by the vendor HAL
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    micras::hal::AdcDma::on_sequence_complete(hadc);
}

/**
 * @brief Callback of the vendor HAL for an overrun of a converter or a transfer error of its DMA.
 *
 * @param hadc Handle of the converter.
 */
// NOLINTNEXTLINE(readability-identifier-naming) the name is fixed by the vendor HAL
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
    micras::hal::AdcDma::on_error(hadc);
}
}

namespace micras::hal {
std::array<AdcDma*, AdcDma::max_instances> AdcDma::instances{};
uint32_t                                   AdcDma::restarts{};

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

    auto* const slot = std::ranges::find(instances, nullptr);

    if (slot == instances.end()) {
        return;
    }

    *slot = this;
    this->initialized = status == HAL_OK;
}

AdcDma::~AdcDma() {
    std::ranges::replace(instances, this, static_cast<AdcDma*>(nullptr));
}

bool AdcDma::start_dma(std::span<uint32_t> buffer) {
    this->transfer = buffer;

    if (HAL_ADC_Start_DMA(this->handle, buffer.data(), buffer.size()) != HAL_OK) {
        this->initialized = false;
        return false;
    }

    return true;
}

bool AdcDma::start_dma(std::span<uint16_t> buffer) {
    return this->start_dma({std::bit_cast<uint32_t*>(buffer.data()), buffer.size()});
}

bool AdcDma::start_dma(std::span<uint16_t> buffer, std::span<uint16_t> snapshot) {
    if (snapshot.size() != buffer.size()) {
        this->initialized = false;
        return false;
    }

    this->buffer = buffer;
    this->snapshot = snapshot;

    return this->start_dma(buffer);
}

uint32_t AdcDma::read_snapshot(std::span<uint16_t> destination) const {
    const std::span<const uint16_t> source = this->snapshot.first(std::min(this->snapshot.size(), destination.size()));
    uint32_t                        before = this->sequence;

    while (true) {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        std::ranges::copy(source, destination.begin());
        std::atomic_signal_fence(std::memory_order_seq_cst);

        const uint32_t after = this->sequence;

        if (after == before) {
            return after;
        }

        before = after;
    }
}

void AdcDma::on_sequence_complete(const ADC_HandleTypeDef* handle) {
    AdcDma* const instance = find(handle);

    if (instance == nullptr) {
        return;
    }

    std::ranges::copy(instance->buffer, instance->snapshot.begin());
    instance->sequence = instance->sequence + 1;
}

void AdcDma::on_error(const ADC_HandleTypeDef* handle) {
    AdcDma* const instance = find(handle);

    if (instance != nullptr) {
        instance->stopped = true;
    }
}

AdcDma* AdcDma::find(const ADC_HandleTypeDef* handle) {
    const auto* const found = std::ranges::find_if(instances, [handle](const AdcDma* instance) {
        return instance != nullptr and instance->handle == handle;
    });

    return found == instances.end() ? nullptr : *found;
}

void AdcDma::stop_dma() {
    HAL_ADC_Stop_DMA(this->handle);
}

void AdcDma::recover() {
    if (not this->stopped) {
        return;
    }

    HAL_ADC_Stop_DMA(this->handle);
    this->handle->State = this->handle->State & ~(HAL_ADC_STATE_ERROR_DMA | HAL_ADC_STATE_ERROR_INTERNAL);
    this->stopped = HAL_ADC_Start_DMA(this->handle, this->transfer.data(), this->transfer.size()) != HAL_OK;
    restarts++;
}

uint32_t AdcDma::get_restarts() {
    return restarts;
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
