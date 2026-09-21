/**
 * @file
 */

#include <bit>
#include <cstdint>
#include <span>

#include "micras/hal/crc.hpp"

namespace micras::hal {
Crc::Crc(const Config& config) : handle{config.handle} { }

uint32_t Crc::calculate(std::span<const uint8_t> data) {
    return HAL_CRC_Calculate(this->handle, std::bit_cast<const uint32_t*>(data.data()), data.size());
}
}  // namespace micras::hal
