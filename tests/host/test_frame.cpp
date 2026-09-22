/**
 * @file
 */

#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>
#include <vector>

#include "micras/comm/frame.hpp"
#include "micras/comm/protocol.hpp"
#include "test_host.hpp"

using namespace micras::comm;

namespace {
struct Vector {
    std::string_view     name;
    MessageType          type;
    std::vector<uint8_t> payload;
    std::vector<uint8_t> frame;
};

// The same table lives in micras-monitor, as src/lib/comm/__tests__/frameVectors.ts. Every writer
// and reader disagreement this project has had was invisible until something was on the wire, so
// the two implementations are pinned to the same bytes rather than to each other's good intentions.
const std::vector<Vector> vectors{
    {"hello", MessageType{0x01}, {}, {4, 1, 209, 241, 0}},
    {"hello_ack",
     MessageType{0x81},
     {1, 133, 114, 115, 247, 4, 0, 125, 0, 0, 0, 0, 1},
     {8, 129, 1, 133, 114, 115, 247, 4, 2, 125, 1, 1, 1, 4, 1, 40, 160, 0}},
    {"schema_request", MessageType{0x02}, {0, 0}, {2, 2, 1, 3, 252, 162, 0}},
    {"group_define", MessageType{0x03}, {0, 80, 0, 2, 0, 0, 1, 0}, {2, 3, 2, 80, 2, 2, 1, 2, 1, 3, 19, 168, 0}},
    {"credit", MessageType{0x05}, {0, 1}, {2, 5, 4, 1, 77, 55, 0}},
    {"write", MessageType{0x06}, {2, 0, 0, 0, 128, 63}, {3, 6, 2, 1, 1, 5, 128, 63, 143, 7, 0}},
    {"zeros", MessageType{0x85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {2, 133, 1, 1, 1,   1,   1,
                                                                                    1, 1,   1, 1, 1,   1,   1,
                                                                                    1, 1,   1, 3, 142, 100, 0}},
    {"ramp",
     MessageType{0x8B},
     {0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,
      23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,
      46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,
      69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,
      92,  93,  94,  95,  96,  97,  98,  99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114,
      115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137,
      138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
      161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183,
      184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199},
     {2,   139, 202, 1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20,
      21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,
      44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,
      67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
      90,  91,  92,  93,  94,  95,  96,  97,  98,  99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
      113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135,
      136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158,
      159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
      182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 115, 21,  0}},
    {"high",
     MessageType{0x89},
     {200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
      220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239},
     {44,  137, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
      221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 42,  80,  0}},
};
}  // namespace

int main() {
    for (const Vector& vector : vectors) {
        std::vector<uint8_t> built(max_frame_size);
        built.resize(encode_frame(vector.type, vector.payload, built));

        if (built != vector.frame) {
            std::printf("%.*s: built", int(vector.name.size()), vector.name.data());

            for (uint8_t byte : built) {
                std::printf(" %02X", byte);
            }

            std::puts("");
            CHECK(false);
        }

        FrameReader reader;
        bool        complete = false;

        for (uint8_t byte : vector.frame) {
            complete = reader.push(byte);
        }

        CHECK(complete);
        CHECK(reader.type() == vector.type);
        CHECK(std::vector<uint8_t>(reader.payload().begin(), reader.payload().end()) == vector.payload);
        CHECK(reader.discarded() == 0);
    }

    // A frame that lost a byte is discarded on its own, and the next one still arrives
    {
        const std::vector<uint8_t>& good = vectors.at(3).frame;
        FrameReader                 reader;
        bool                        complete = false;

        for (std::size_t i = 0; i + 3 < good.size(); i++) {
            CHECK(not reader.push(good[i]));
        }

        CHECK(not reader.push(0x00));
        CHECK(reader.discarded() == 1);

        for (uint8_t byte : good) {
            complete = reader.push(byte);
        }

        CHECK(complete);
        CHECK(reader.discarded() == 1);
    }

    std::printf("frame ok: %zu vectors\n", vectors.size());
}
