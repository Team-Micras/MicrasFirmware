/**
 * @file
 */

#include <algorithm>
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

/**
 * @brief Mask that turns the identifier of one of the first four channels into the position of its
 * bits in the capture and compare enable register.
 */
static constexpr uint32_t channel_shift_mask{0x1F};

/**
 * @brief Make a value written to the compare register of a channel take effect at once.
 *
 * @param handle Timer handle.
 * @param channel Timer channel.
 */
static void disable_compare_preload(TIM_HandleTypeDef* handle, uint32_t channel) {
    __HAL_TIM_DISABLE_OCxPRELOAD(handle, channel);
}

/**
 * @brief Make a value written to the compare register of a channel wait for the next update.
 *
 * @param handle Timer handle.
 * @param channel Timer channel.
 */
static void enable_compare_preload(TIM_HandleTypeDef* handle, uint32_t channel) {
    __HAL_TIM_ENABLE_OCxPRELOAD(handle, channel);
}

Pwm::Pwm(const Config& config) : handle{config.handle}, channel{config.timer_channel}, inverted{config.inverted} {
    if (this->handle->State == HAL_TIM_STATE_RESET) {
        config.init_function();
    }

    if (this->inverted) {
        this->handle->Instance->CCER |= TIM_CCER_CC1P << (this->channel & channel_shift_mask);
    }

    disable_compare_preload(this->handle, this->channel);
    this->set_duty_cycle(0.0F);
    enable_compare_preload(this->handle, this->channel);

    this->initialized = HAL_TIM_PWM_Start(this->handle, this->channel) == HAL_OK;
}

void Pwm::set_duty_cycle(float duty_cycle) {
    duty_cycle = std::clamp(duty_cycle, 0.0F, 100.0F);

    if (this->inverted) {
        duty_cycle = 100.0F - duty_cycle;
    }

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
