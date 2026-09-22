/**
 * @file
 */

#include <bit>
#include <cstdio>
#include <string_view>

#include "micras/core/crc.hpp"
#include "test_host.hpp"

using namespace micras::core;

int main() {
    std::string_view         s = "123456789";
    std::span<const uint8_t> b{std::bit_cast<const uint8_t*>(s.data()), s.size()};
    printf("crc16=%04X crc32=%08X\n", crc16(b), crc32(b));
    CHECK(crc16(b) == 0x29B1);
    CHECK(crc32(b) == 0x0376E6E7);
    // incremental must equal one shot
    CHECK(crc16(b.subspan(4), crc16(b.first(4))) == crc16(b));
    puts("crc ok");
}
