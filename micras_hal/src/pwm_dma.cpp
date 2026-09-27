/**
 * @file
 */

#include <bit>
#include <cstdint>
#include <span>

#include "micras/hal/pwm_dma.hpp"

namespace micras::hal {
PwmDma::PwmDma(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    if (this->handle->State == HAL_TIM_STATE_RESET) {
        config.init_function();
    }

    this->initialized = this->handle->State == HAL_TIM_STATE_READY;
}

void PwmDma::start_dma(std::span<uint32_t> buffer) {
    if (this->is_busy()) {
        return;
    }

    HAL_TIM_PWM_Start_DMA(this->handle, this->channel, buffer.data(), buffer.size());
}

void PwmDma::start_dma(std::span<uint16_t> buffer) {
    start_dma({std::bit_cast<uint32_t*>(buffer.data()), buffer.size()});
}

void PwmDma::stop_dma() {
    HAL_TIM_PWM_Stop_DMA(this->handle, this->channel);
}

uint32_t PwmDma::get_compare(float duty_cycle) const {
    const float scaled = duty_cycle * static_cast<float>(__HAL_TIM_GET_AUTORELOAD(this->handle) + 1) * 0.01F;

    // NOLINTNEXTLINE(bugprone-incorrect-roundings)
    return static_cast<uint32_t>(scaled + 0.5F);
}

bool PwmDma::is_busy() {
    return TIM_CHANNEL_STATE_GET(this->handle, this->channel) == HAL_TIM_CHANNEL_STATE_BUSY;
}

bool PwmDma::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
