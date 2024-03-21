/**
 * @file timer.cpp
 *
 * @brief STM32 TIM HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/timer.hpp"

namespace hal {
Timer::Timer(Config& tim_config) : handle(tim_config.handle) {
    tim_config.init_function();
}

void Timer::base_start() {
    HAL_TIM_Base_Start(this->handle);
}

void Timer::base_stop() {
    HAL_TIM_Base_Stop(this->handle);
}

void Timer::set_counter(uint32_t counter) {
    __HAL_TIM_SET_COUNTER(this->handle, counter);
}

uint32_t Timer::get_counter() {
    return __HAL_TIM_GET_COUNTER(this->handle);
}

void Timer::pwm_start(uint32_t channel) {
    HAL_TIM_PWM_Start(this->handle, channel);
}

void Timer::pwm_stop(uint32_t channel) {
    HAL_TIM_PWM_Stop(this->handle, channel);
}

void Timer::set_prescaler(uint32_t prescaler) {
    __HAL_TIM_SET_PRESCALER(this->handle, prescaler);
}

uint32_t Timer::get_prescaler() {
    return this->handle->Instance->PSC;
}

void Timer::set_compare(uint32_t channel, uint32_t compare) {
    __HAL_TIM_SET_COMPARE(this->handle, channel, compare);
}

uint32_t Timer::get_autoreload() {
    return __HAL_TIM_GET_AUTORELOAD(this->handle);
}

void Timer::set_autoreload(uint32_t autoreload) {
    __HAL_TIM_SET_AUTORELOAD(this->handle, autoreload);
}

uint32_t Timer::get_timer_clock_freq() {
    return HAL_RCC_GetPCLK1Freq();
}
}  // namespace hal
