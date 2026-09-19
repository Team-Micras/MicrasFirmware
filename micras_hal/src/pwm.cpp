/**
 * @file
 */

#include <bit>
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
    // Several channels of the same timer are separate Pwm objects sharing one init function, and
    // re-running it would regenerate an update event on a timer that is already counting
    if (this->handle->State == HAL_TIM_STATE_RESET) {
        config.init_function();
    }

    this->initialized = HAL_TIM_PWM_Start(this->handle, this->channel) == HAL_OK;
    __HAL_TIM_SET_COMPARE(this->handle, this->channel, 0);
}

void Pwm::set_duty_cycle(float duty_cycle) {
    // Adding a half and truncating is exactly round half up for a duty cycle that is never
    // negative, and it avoids the libm call std::lround would make in a path that runs five times
    // per control loop
    const float scaled = duty_cycle * static_cast<float>(__HAL_TIM_GET_AUTORELOAD(this->handle) + 1) * 0.01F;

    // NOLINTNEXTLINE(bugprone-incorrect-roundings)
    const auto compare = static_cast<uint32_t>(scaled + 0.5F);

    __HAL_TIM_SET_COMPARE(this->handle, this->channel, compare);
}

void Pwm::set_frequency(uint32_t frequency) {
    const uint32_t base_freq = get_timer_clock_frequency(this->handle->Instance);
    const uint32_t prescaler = this->handle->Instance->PSC;

    const uint32_t autoreload = base_freq / ((prescaler + 1) * frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(this->handle, autoreload);
    __HAL_TIM_SET_COUNTER(this->handle, 0);
}

bool Pwm::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
