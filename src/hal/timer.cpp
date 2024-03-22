/**
 * @file timer.cpp
 *
 * @brief STM32 TIM HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/timer.hpp"

namespace hal {
Timer::Timer(Config& timer_config) : handle{timer_config.handle} {
    timer_config.init_function();

    uint32_t base_freq = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = this->handle->Instance->PSC;

    if (base_freq / (prescaler + 1) == 1000000 and __HAL_TIM_GetAutoreload(this->handle) == 1000) {
        this->enable_microseconds = true;
    }
}

void Timer::reset_ms() {
    this->counter = this->get_counter_ms();
}

void Timer::reset_us() {
    this->counter = this->get_counter_us();
}

uint32_t Timer::elapsed_time_ms() const {
    return this->get_counter_ms() - this->counter;
}

uint32_t Timer::elapsed_time_us() const {
    return this->get_counter_us() - this->counter;
}

void Timer::sleep_ms(uint32_t time) {
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < time) {
        continue;
    }
}

void Timer::sleep_us(uint32_t time) const {
    uint32_t start = this->get_counter_us();

    while (this->get_counter_us() - start < time) {
        continue;
    }
}

uint32_t Timer::get_counter_ms() const {
    return HAL_GetTick();
}

uint32_t Timer::get_counter_us() const {
    if (this->enable_microseconds) {
        return 1000 * HAL_GetTick() + __HAL_TIM_GET_COUNTER(this->handle);
    }

    return 1000 * HAL_GetTick();
}
}  // namespace hal
