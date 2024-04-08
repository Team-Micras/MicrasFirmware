/**
 * @file encoder.cpp
 *
 * @brief STM32 encoder HAL wrapper
 *
 * @date 03/2024
 */

#include "micras/hal/encoder.hpp"

namespace micras::hal {
Encoder::Encoder(const Config& config) : handle{config.handle} {
    config.init_function();
    HAL_TIM_Encoder_Start(this->handle, config.timer_channel);
}

int32_t Encoder::get_counter() const {
    return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(this->handle));
}
}  // namespace micras::hal