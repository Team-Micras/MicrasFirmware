/**
 * @file
 */

#include <bit>

#include "micras/hal/encoder.hpp"

namespace micras::hal {
Encoder::Encoder(const Config& config) : handle{config.handle} {
    config.init_function();
    HAL_TIM_Encoder_Start(this->handle, config.timer_channel);

    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
    this->start_count = __HAL_TIM_GET_AUTORELOAD(this->handle) / 2;
    __HAL_TIM_SET_COUNTER(this->handle, this->start_count);
}

int32_t Encoder::get_counter() const {
    return std::bit_cast<int32_t>(__HAL_TIM_GET_COUNTER(this->handle) - this->start_count);
}
}  // namespace micras::hal
