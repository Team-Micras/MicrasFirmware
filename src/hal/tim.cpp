/**
 * @file tim.cpp
 *
 * @brief STM32 TIM HAL wrapper.
 *
 * @date 03/2024
 */

#include "hal/tim.hpp"

namespace hal {
Tim::Tim(Config& tim_config) : handle(tim_config.handle) {
    tim_config.init_function();
}

void Tim::base_start() {
    HAL_TIM_Base_Start(this->handle);
}

void Tim::base_stop() {
    HAL_TIM_Base_Stop(this->handle);
}

void Tim::set_counter(uint32_t counter) {
    __HAL_TIM_SET_COUNTER(this->handle, counter);
}

uint32_t Tim::get_counter() {
    return __HAL_TIM_GET_COUNTER(this->handle);
}

void Tim::pwm_start(uint32_t channel) {
    HAL_TIM_PWM_Start(this->handle, channel);
}

void Tim::pwm_stop(uint32_t channel) {
    HAL_TIM_PWM_Stop(this->handle, channel);
}

void Tim::set_prescaler(uint32_t prescaler) {
    __HAL_TIM_SET_PRESCALER(this->handle, prescaler);
}

void Tim::set_compare(uint32_t channel, uint32_t compare) {
    __HAL_TIM_SET_COMPARE(this->handle, channel, compare);
}

uint32_t Tim::get_autoreload() {
    return __HAL_TIM_GET_AUTORELOAD(this->handle);
}
}  // namespace hal
