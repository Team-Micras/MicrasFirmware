/**
 * @file pwm_dma.cpp
 *
 * @brief STM32 PWM DMA HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/pwm_dma.hpp"

namespace hal {
PwmDma::PwmDma(Config& pwm_config) : handle{pwm_config.handle}, channel{pwm_config.timer_channel} {
    pwm_config.init_function();
}

void PwmDma::start_dma(uint32_t buffer[], uint32_t size) {
    HAL_TIM_PWM_Start_DMA(this->handle, this->channel, buffer, size);
}

void PwmDma::stop_dma() {
    HAL_TIM_PWM_Stop_DMA(this->handle, this->channel);
}
}  // namespace hal
