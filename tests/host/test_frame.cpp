/**
 * @file
 */

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <span>
#include <sstream>
#include <string>
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
    std::string_view     fields;
};

// The same table is written to vectors/v2.json, which micras-monitor reads, so that both
// implementations are pinned to the same bytes. The fields name what a message of the protocol
// carries, and are empty for the vectors that only exercise the framing.
const std::vector<Vector> vectors{
    {"hello", MessageType{0x01}, {}, {4, 1, 1, 1, 0}, R"json({})json"},
    {"hello_ack",
     MessageType{0x81},
     {2, 133, 114, 115, 247, 4, 0, 125, 0, 0, 0, 0, 1, 120, 86, 52, 18, 6, 109, 105, 99, 114, 97, 115},
     {8,   129, 2,  133, 114, 115, 247, 4,  2,   125, 1,   1, 1, 15, 1,
      120, 86,  52, 18,  6,   109, 105, 99, 114, 97,  115, 6, 8, 0},
     R"json({"protocol_version": 2, "schema_hash": 4151538309, "variable_count": 4, "loop_time_us": 125, "credit_window": 256, "boot_id": 305419896, "robot_name": "micras"})json"},
    {"schema_request", MessageType{0x02}, {0, 0}, {2, 2, 1, 3, 2, 6, 0}, R"json({"first": 0})json"},
    {"schema_page",
     MessageType{0x82},
     {133, 114, 115, 247, 0,  0,   2,   0, 2,   1,  1,   5,   115, 116, 97,  116, 101,
      11,  8,   4,   109, 97, 122, 101, 9, 109, 97, 122, 101, 45,  103, 114, 105, 100},
     {6, 130, 133, 114, 115, 247, 1, 2,   2,  29,  2,   1,  1,   5,   115, 116, 97,  116, 101, 11,
      8, 4,   109, 97,  122, 101, 9, 109, 97, 122, 101, 45, 103, 114, 105, 100, 102, 59,  0},
     R"json({"schema_hash": 4151538309, "first": 0, "variable_count": 2, "entries": [{"type": 1, "access": 1, "name": "state"}, {"type": 11, "access": 8, "name": "maze", "type_tag": "maze-grid"}]})json"},
    {"group_define",
     MessageType{0x03},
     {0, 80, 0, 2, 0, 0, 1, 0},
     {2, 3, 2, 80, 2, 2, 1, 2, 1, 3, 86, 89, 0},
     R"json({"group": 0, "period": 80, "ids": [0, 1]})json"},
    {"credit",
     MessageType{0x05},
     {69, 35, 1, 0},
     {5, 5, 69, 35, 1, 3, 110, 153, 0},
     R"json({"consumed_total": 74565})json"},
    {"credit_wrapped",
     MessageType{0x05},
     {16, 255, 255, 255},
     {8, 5, 16, 255, 255, 255, 21, 89, 0},
     R"json({"consumed_total": 4294967056})json"},
    {"write",
     MessageType{0x06},
     {2, 0, 0, 0, 128, 63},
     {3, 6, 2, 1, 1, 5, 128, 63, 199, 118, 0},
     R"json({"id": 2, "value_f32": 1.0})json"},
    {"command",
     MessageType{0x08},
     {5, 0, 0, 0, 0},
     {3, 8, 5, 1, 1, 1, 3, 13, 73, 0},
     R"json({"code": 5, "argument": 0})json"},
    {"command_ack",
     MessageType{0x88},
     {5, 3, 2},
     {7, 136, 5, 3, 2, 146, 57, 0},
     R"json({"code": 5, "result": 3, "reason": 2})json"},
    {"command_ack_refused",
     MessageType{0x88},
     {0, 2, 1},
     {2, 136, 5, 2, 1, 139, 39, 0},
     R"json({"code": 0, "result": 2, "reason": 1})json"},
    {"log",
     MessageType{0x8A},
     {1, 64, 226, 1, 0, 115, 116, 97, 116, 101, 32, 82, 85, 78},
     {6, 138, 1, 64, 226, 1, 12, 115, 116, 97, 116, 101, 32, 82, 85, 78, 232, 159, 0},
     R"json({"severity": 1, "timestamp_us": 123456, "text": "state RUN"})json"},
    {"zeros",
     MessageType{0x85},
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {2, 133, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 133, 221, 0},
     R"json()json"},
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
      182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 149, 49,  0},
     R"json()json"},
    {"high",
     MessageType{0x89},
     {200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
      220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239},
     {44,  137, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
      221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 247, 247, 0},
     R"json()json"},
};

std::vector<uint8_t> build(auto&& fill) {
    std::vector<uint8_t> buffer(max_payload_size);
    Writer               writer{buffer};
    fill(writer);
    const auto written = writer.done();
    return {written.begin(), written.end()};
}

const Vector& find(std::string_view name) {
    for (const Vector& vector : vectors) {
        if (vector.name == name) {
            return vector;
        }
    }

    CHECK(false);
    return vectors.front();
}

std::string to_json() {
    std::ostringstream out;
    out << "{\n  \"protocol_version\": " << int(protocol_version) << ",\n  \"vectors\": [\n";

    for (std::size_t i = 0; i < vectors.size(); i++) {
        const Vector& vector = vectors[i];
        out << "    {\"name\": \"" << vector.name << "\", \"type\": " << int(vector.type) << ", \"payload\": [";

        for (std::size_t k = 0; k < vector.payload.size(); k++) {
            out << (k > 0 ? ", " : "") << int(vector.payload[k]);
        }

        out << "], \"frame\": [";

        for (std::size_t k = 0; k < vector.frame.size(); k++) {
            out << (k > 0 ? ", " : "") << int(vector.frame[k]);
        }

        out << "]";

        if (not vector.fields.empty()) {
            out << ", \"fields\": " << vector.fields;
        }

        out << "}" << (i + 1 < vectors.size() ? "," : "") << "\n";
    }

    out << "  ]\n}\n";
    return out.str();
}
}  // namespace

int main(int argc, char** argv) {
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

    // The payloads of the protocol, built field by field the way the link writes them
    CHECK(find("hello_ack").payload == build([](Writer& w) {
              w.u8(2);
              w.u32(0xF7737285);
              w.u16(4);
              w.u32(125);
              w.u16(credit_window);
              w.u32(0x12345678);
              w.u8(6);
              w.text("micras");
          }));
    CHECK(find("schema_page").payload == build([](Writer& w) {
              w.u32(0xF7737285);
              w.u16(0);
              w.u16(2);
              w.u8(2);
              w.u8(1);
              w.u8(1);
              w.u8(5);
              w.text("state");
              w.u8(11);
              w.u8(8);
              w.u8(4);
              w.text("maze");
              w.u8(9);
              w.text("maze-grid");
          }));
    CHECK(find("credit").payload == build([](Writer& w) { w.u32(0x00012345); }));
    CHECK(find("credit_wrapped").payload == build([](Writer& w) { w.u32(0xFFFFFF10); }));
    CHECK(find("command").payload == build([](Writer& w) {
              w.u8(5);
              w.u32(0);
          }));
    CHECK(find("command_ack").payload == build([](Writer& w) {
              w.u8(5);
              w.u8(3);
              w.u8(2);
          }));
    CHECK(find("log").payload == build([](Writer& w) {
              w.u8(1);
              w.u32(123456);
              w.text("state RUN");
          }));

    // A frame that lost a byte is discarded on its own, and the next one still arrives
    {
        const std::vector<uint8_t>& good = find("group_define").frame;
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

    // With a path, the JSON copy of the table is checked against it, or rewritten with --write
    if (argc >= 2) {
        const std::string_view path{argv[argc - 1]};
        const std::string      json = to_json();

        if (argc >= 3 and std::string_view{argv[1]} == "--write") {
            std::ofstream{std::string{path}} << json;
        } else {
            std::ifstream     file{std::string{path}};
            const std::string recorded{std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};

            if (recorded != json) {
                std::printf(
                    "%.*s is out of date: run test_frame --write %.*s\n", int(path.size()), path.data(),
                    int(path.size()), path.data()
                );
                CHECK(false);
            }
        }
    }

    std::printf("frame ok: %zu vectors\n", vectors.size());
}
