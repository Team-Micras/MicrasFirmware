/**
 * @file pwm_dma.cpp
 *
 * @brief STM32 PWM DMA HAL wrapper
 *
 * @date 03/2024
 */

#include <bit>
#include <cmath>

#include "micras/hal/pwm_dma.hpp"

namespace micras::hal {
PwmDma::PwmDma(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    config.init_function();
}

void PwmDma::start_dma(std::span<uint32_t> buffer) {
    if (TIM_CHANNEL_STATE_GET(this->handle, this->channel) == HAL_TIM_CHANNEL_STATE_BUSY) {
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
    return std::lround((duty_cycle * (__HAL_TIM_GET_AUTORELOAD(this->handle) + 1) / 100) - 1);
}
}  // namespace micras::hal
