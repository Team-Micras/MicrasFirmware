/**
 * @file
 */

#include <array>
#include <cstdint>
#include <span>

#include "micras/core/crc.hpp"

namespace micras::core {
static constexpr std::array<uint16_t, 16> crc16_table{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
};

uint16_t crc16(std::span<const uint8_t> data, uint16_t seed) {
    uint16_t crc = seed;

    for (const uint8_t byte : data) {
        crc = static_cast<uint16_t>(crc << 4) ^ crc16_table.at((crc >> 12) ^ (byte >> 4));
        crc = static_cast<uint16_t>(crc << 4) ^ crc16_table.at((crc >> 12) ^ (byte & 0x0F));
    }

    return crc;
}

uint32_t crc32(std::span<const uint8_t> data, uint32_t seed) {
    uint32_t crc = seed;

    for (const uint8_t byte : data) {
        crc ^= static_cast<uint32_t>(byte) << 24;

        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80000000) != 0 ? (crc << 1) ^ 0x04C11DB7 : crc << 1;
        }
    }

    return crc;
}
}  // namespace micras::core
