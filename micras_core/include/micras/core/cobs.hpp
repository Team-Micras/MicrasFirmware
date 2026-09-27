/**
 * @file
 */

#ifndef MICRAS_CORE_COBS_HPP
#define MICRAS_CORE_COBS_HPP

#include <cstddef>
#include <cstdint>
#include <span>

namespace micras::core {
/**
 * @brief Byte reserved to delimit frames, which encoding removes from the data.
 */
constexpr uint8_t cobs_delimiter{0x00};

/**
 * @brief Compute the largest size the encoding of a buffer can have.
 *
 * @param size Number of bytes of the data.
 * @return Number of bytes the encoded data can take at most.
 */
constexpr std::size_t cobs_encoded_size(std::size_t size) {
    return size + size / 254 + 1;
}

/**
 * @brief Encode a buffer so that it contains no delimiter byte.
 *
 * @note Consistent Overhead Byte Stuffing costs one byte every 254, against up to one byte per byte
 * for escaping, and leaves the delimiter impossible to produce by construction. Resynchronizing
 * after the module silently drops a run of bytes is then just a matter of scanning to the next
 * delimiter.
 *
 * @param from Data to encode.
 * @param into Buffer to encode into, which should hold cobs_encoded_size bytes.
 * @return Number of bytes written, or zero if the buffer is too small.
 */
std::size_t cobs_encode(std::span<const uint8_t> from, std::span<uint8_t> into);

/**
 * @brief Decode a frame that was encoded by cobs_encode.
 *
 * @param from Encoded frame, without any delimiter.
 * @param into Buffer to decode into.
 * @return Number of bytes written, or zero if the frame is malformed or the buffer is too small.
 */
std::size_t cobs_decode(std::span<const uint8_t> from, std::span<uint8_t> into);
}  // namespace micras::core

#endif  // MICRAS_CORE_COBS_HPP
