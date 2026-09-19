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
    // With CRC_INPUTDATA_FORMAT_BYTES the HAL reads the buffer byte by byte, so the word pointer in
    // its signature describes the declaration rather than the access
    return HAL_CRC_Calculate(this->handle, std::bit_cast<const uint32_t*>(data.data()), data.size());
}
}  // namespace micras::hal
