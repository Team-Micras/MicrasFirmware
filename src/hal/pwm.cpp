/**
 * @file pwm.cpp
 *
 * @brief STM32 PWM HAL wrapper
 *
 * @date 03/2024
 */

#include <cmath>

#include "hal/pwm.hpp"

namespace hal {
Pwm::Pwm(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    config.init_function();
    HAL_TIM_PWM_Start(this->handle, this->channel);
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, 0);
}

void Pwm::set_duty_cycle(float duty_cycle) {
    uint32_t compare = std::round((duty_cycle * (__HAL_TIM_GET_AUTORELOAD(this->handle) + 1)) / 100 - 1);
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, compare);
}

void Pwm::set_frequency(uint32_t frequency) {
    uint32_t base_freq = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = this->handle->Instance->PSC;

    uint32_t autoreload = base_freq / ((prescaler + 1) * frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(this->handle, autoreload);
}
}  // namespace hal
