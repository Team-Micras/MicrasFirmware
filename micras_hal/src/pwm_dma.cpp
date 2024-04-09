/**
 * @file pwm_dma.cpp
 *
 * @brief STM32 PWM DMA HAL wrapper
 *
 * @date 03/2024
 */

#include <cmath>

#include "micras/hal/pwm_dma.hpp"

namespace micras::hal {
PwmDma::PwmDma(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    config.init_function();
    HAL_TIM_RegisterCallback(this->handle, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, pulse_finished_callback);
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void PwmDma::start_dma(uint32_t buffer[], uint32_t size) {
    if (TIM_CHANNEL_STATE_GET(this->handle, this->channel) == HAL_TIM_CHANNEL_STATE_BUSY) {
        HAL_TIM_PWM_Stop_DMA(this->handle, this->channel);
    }

    HAL_TIM_PWM_Start_DMA(this->handle, this->channel, buffer, size);
}

void PwmDma::stop_dma() {
    HAL_TIM_PWM_Stop_DMA(this->handle, this->channel);
}

uint32_t PwmDma::get_compare(float duty_cycle) const {
    return std::lround((duty_cycle * (__HAL_TIM_GET_AUTORELOAD(this->handle) + 1)) / 100 - 1);
}

void PwmDma::pulse_finished_callback(TIM_HandleTypeDef* htim) {
    switch (HAL_TIM_GetActiveChannel(htim)) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
            HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
            break;
        default:
            break;
    }
}
}  // namespace micras::hal
