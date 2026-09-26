/**
 * @file
 */

#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/core/cobs.hpp"

namespace micras::core {
// Both functions index buffers the caller sized, and every access is bounded by a check just
// above it, which is the whole content of the algorithm.
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
std::size_t cobs_encode(std::span<const uint8_t> from, std::span<uint8_t> into) {
    if (into.size() < cobs_encoded_size(from.size())) {
        return 0;
    }

    std::size_t code_index = 0;
    std::size_t write_index = 1;
    uint8_t     code = 1;

    for (const uint8_t byte : from) {
        if (byte != cobs_delimiter) {
            into[write_index++] = byte;

            if (++code != 0xFF) {
                continue;
            }
        }

        into[code_index] = code;
        code_index = write_index++;
        code = 1;
    }

    if (not from.empty() and from.back() != cobs_delimiter and code == 1) {
        return write_index - 1;
    }

    into[code_index] = code;
    return write_index;
}

std::size_t cobs_decode(std::span<const uint8_t> from, std::span<uint8_t> into) {
    std::size_t read_index = 0;
    std::size_t write_index = 0;

    while (read_index < from.size()) {
        const uint8_t code = from[read_index];

        if (code == cobs_delimiter or read_index + code > from.size()) {
            return 0;
        }

        read_index++;

        for (uint8_t i = 1; i < code; i++) {
            if (write_index >= into.size()) {
                return 0;
            }

            into[write_index++] = from[read_index++];
        }

        if (code != 0xFF and read_index < from.size()) {
            if (write_index >= into.size()) {
                return 0;
            }

            into[write_index++] = cobs_delimiter;
        }
    }

    return write_index;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
}  // namespace micras::core
