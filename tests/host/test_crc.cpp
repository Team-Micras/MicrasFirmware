/**
 * @file
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

#include "micras/core/crc.hpp"
#include "test_host.hpp"

using namespace micras::core;

int main() {
    const std::string_view         text{"123456789"};
    const std::span<const uint8_t> bytes{std::bit_cast<const uint8_t*>(text.data()), text.size()};

    // The published check value of CRC-16/CCITT-FALSE, so this is pinned to the standard rather
    // than to whatever the other implementation of it happens to agree with
    CHECK(crc16(bytes) == 0x29B1);

    // Computing it in two calls has to give the same answer as computing it in one
    CHECK(crc16(bytes.subspan(4), crc16(bytes.first(4))) == crc16(bytes));

    // A single flipped bit anywhere has to change it
    for (std::size_t index = 0; index < text.size(); index++) {
        for (uint8_t bit = 0; bit < 8; bit++) {
            std::array<uint8_t, 9> flipped{};
            std::ranges::copy(bytes, flipped.begin());
            flipped.at(index) ^= static_cast<uint8_t>(1U << bit);

            CHECK(crc16(flipped) != crc16(bytes));
        }
    }

    std::puts("crc ok");
}
