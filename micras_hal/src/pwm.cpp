/**
 * @file
 */

#include <cmath>

#include "micras/hal/pwm.hpp"

namespace micras::hal {
Pwm::Pwm(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    config.init_function();
    HAL_TIM_PWM_Start(this->handle, this->channel);
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, 0);
}

void Pwm::set_duty_cycle(float duty_cycle) {
    const uint32_t compare = std::lround((duty_cycle * (__HAL_TIM_GET_AUTORELOAD(this->handle) + 1) / 100.0F));
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, compare);
}

void Pwm::set_frequency(uint32_t frequency) {
    const uint32_t base_freq = HAL_RCC_GetPCLK1Freq();
    const uint32_t prescaler = this->handle->Instance->PSC;

    const uint32_t autoreload = base_freq / ((prescaler + 1) * frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(this->handle, autoreload);
    __HAL_TIM_SET_COUNTER(this->handle, 0);
}
}  // namespace micras::hal
