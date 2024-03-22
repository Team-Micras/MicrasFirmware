/**
 * @file encoder.cpp
 *
 * @brief STM32 encoder HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/encoder.hpp"

namespace hal {
Encoder::Encoder(Config& encoder_config) : handle{encoder_config.handle} {
    encoder_config.init_function();
    HAL_TIM_Encoder_Start(this->handle, encoder_config.timer_channel);
}

int32_t Encoder::get_counter() {
    return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(this->handle));
}
}  // namespace hal
