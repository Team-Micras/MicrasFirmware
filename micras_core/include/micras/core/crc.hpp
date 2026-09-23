/**
 * @file
 */

#ifndef MICRAS_CORE_CRC_HPP
#define MICRAS_CORE_CRC_HPP

#include <cstdint>
#include <span>

namespace micras::core {
/**
 * @brief Compute the CRC-16/CCITT-FALSE of a buffer.
 *
 * @note Computed a bit at a time, from the polynomial itself, rather than through a table or the
 * CRC peripheral. The peripheral is configured for the polynomial of the rotary sensors and is
 * stateful, so sharing it would couple the 8 kHz sensor path to this one; a table would buy speed
 * on a path that runs a hundred times a second and carries a few hundred bytes each time.
 *
 * @param data Buffer to compute the CRC of.
 * @param seed Initial value of the register, so that a CRC can be computed in several calls.
 * @return CRC of the buffer.
 */
constexpr uint16_t crc16(std::span<const uint8_t> data, uint16_t seed = 0xFFFF) {
    constexpr uint16_t polynomial{0x1021};

    uint16_t crc = seed;

    for (const uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte << 8U);

        for (uint8_t bit = 0; bit < 8; bit++) {
            const bool overflow = (crc & 0x8000U) != 0;
            crc = static_cast<uint16_t>(crc << 1U);

            if (overflow) {
                crc ^= polynomial;
            }
        }
    }

    return crc;
}
}  // namespace micras::core

#endif  // MICRAS_CORE_CRC_HPP
