/**
 * @file crc.cpp
 *
 * @brief STM32 CRC HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/crc.hpp"

namespace hal {
Crc::Crc(const Config& config) : handle{config.handle} {
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
uint32_t Crc::calculate(uint32_t data[], uint32_t size) {
    return HAL_CRC_Calculate(this->handle, data, size);
}
}  // namespace hal
