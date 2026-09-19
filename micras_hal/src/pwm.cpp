/**
 * @file
 */

#include <bit>
#include <cmath>
#include <cstdint>

#include "micras/hal/pwm.hpp"

namespace micras::hal {
/**
 * @brief Get the frequency of the clock that feeds a timer.
 *
 * @note Timers run at twice the frequency of their APB bus when the APB prescaler is not 1.
 *
 * @param instance Timer peripheral.
 * @return Timer clock frequency in Hz.
 */
static uint32_t get_timer_clock_frequency(const TIM_TypeDef* instance) {
    RCC_ClkInitTypeDef clock_config{};
    uint32_t           flash_latency{};
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    if (std::bit_cast<uintptr_t>(instance) >= APB2PERIPH_BASE) {
        const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        return clock_config.APB2CLKDivider == RCC_APB2_DIV1 ? pclk2 : 2 * pclk2;
    }

    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return clock_config.APB1CLKDivider == RCC_APB1_DIV1 ? pclk1 : 2 * pclk1;
}

Pwm::Pwm(const Config& config) : handle{config.handle}, channel{config.timer_channel} {
    config.init_function();
    HAL_TIM_PWM_Start(this->handle, this->channel);
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, 0);
}

void Pwm::set_duty_cycle(float duty_cycle) {
    const auto compare =
        static_cast<uint32_t>(std::lround(duty_cycle * (__HAL_TIM_GET_AUTORELOAD(this->handle) + 1) / 100.0F));
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, compare);
}

void Pwm::set_frequency(uint32_t frequency) {
    const uint32_t base_freq = get_timer_clock_frequency(this->handle->Instance);
    const uint32_t prescaler = this->handle->Instance->PSC;

    const uint32_t autoreload = base_freq / ((prescaler + 1) * frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(this->handle, autoreload);
    __HAL_TIM_SET_COUNTER(this->handle, 0);
}
}  // namespace micras::hal
