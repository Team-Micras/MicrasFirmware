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
 * @note Implemented in software, over a nibble table, instead of through the CRC peripheral. The
 * peripheral is configured for the polynomial of the rotary sensors and is stateful, so sharing it
 * with the 8 kHz sensor path would couple two unrelated things to save a few cycles on a path that
 * runs a hundred times a second.
 *
 * @param data Buffer to compute the CRC of.
 * @param seed Initial value of the register, so that a CRC can be computed in several calls.
 * @return CRC of the buffer.
 */
uint16_t crc16(std::span<const uint8_t> data, uint16_t seed = 0xFFFF);
}  // namespace micras::core

#endif  // MICRAS_CORE_CRC_HPP
