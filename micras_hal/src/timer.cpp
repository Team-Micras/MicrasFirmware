/**
 * @file
 */

#include "micras/hal/timer.hpp"

namespace micras::hal {
Timer::Timer(const Config& config) : handle{config.handle} {
    config.init_function();

    const uint32_t base_freq = HAL_RCC_GetPCLK1Freq();
    const uint32_t prescaler = this->handle->Instance->PSC;

    if (base_freq / (prescaler + 1) == 1000000) {
        this->enable_microseconds = true;
        HAL_TIM_Base_Start(this->handle);
    }
}

uint32_t Timer::get_counter_ms() {
    return HAL_GetTick();
}

uint32_t Timer::get_counter_us() const {
    if (this->enable_microseconds) {
        return __HAL_TIM_GET_COUNTER(this->handle);
    }

    return 1000 * HAL_GetTick();
}
}  // namespace micras::hal
