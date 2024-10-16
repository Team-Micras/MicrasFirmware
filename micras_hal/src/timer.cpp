/**
 * @file timer.cpp
 *
 * @brief STM32 TIM HAL wrapper
 *
 * @date 03/2024
 */

#include "micras/hal/timer.hpp"

namespace micras::hal {
Timer::Timer() {
    this->reset_ms();
}

Timer::Timer(const Config& config) : handle{config.handle} {
    config.init_function();

    uint32_t base_freq = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = this->handle->Instance->PSC;

    if (base_freq / (prescaler + 1) == 1000000 and __HAL_TIM_GetAutoreload(this->handle) == 999) {
        this->enable_microseconds = true;
    }

    this->reset_us();
}

void Timer::reset_ms() {
    this->counter = get_counter_ms();
}

void Timer::reset_us() {
    this->counter = this->get_counter_us();
}

uint32_t Timer::elapsed_time_ms() const {
    return get_counter_ms() - this->counter;
}

uint32_t Timer::elapsed_time_us() const {
    return this->get_counter_us() - this->counter;
}

void Timer::sleep_ms(uint32_t time) {
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < time) { }
}

void Timer::sleep_us(uint32_t time) const {
    uint32_t start = this->get_counter_us();

    while (this->get_counter_us() - start < time) { }
}

uint32_t Timer::get_counter_ms() {
    return HAL_GetTick();
}

uint32_t Timer::get_counter_us() const {
    if (this->enable_microseconds) {
        return 1000 * HAL_GetTick() + __HAL_TIM_GET_COUNTER(this->handle);
    }

    return 1000 * HAL_GetTick();
}
}  // namespace micras::hal
