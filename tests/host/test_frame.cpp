/**
 * @file
 */

#include <bit>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
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
};

// The same table is written to vectors/v2.json, which micras-monitor reads as
// src/protocol/fixtures/frame-vectors.ts, so that both implementations are pinned to the same bytes.
const std::vector<Vector>
    vectors{
        {"hello", MessageType{0x01}, {}, {4, 1, 1, 1, 0}},
        {"hello_ack",
         MessageType{0x81},
         {2, 133, 114, 115, 247, 4, 0, 125, 0, 0, 0, 0, 1, 120, 86, 52, 18, 6, 109, 105, 99, 114, 97, 115},
         {8,   129, 2,  133, 114, 115, 247, 4,  2,   125, 1,   1, 1, 15, 1,
          120, 86,  52, 18,  6,   109, 105, 99, 114, 97,  115, 6, 8, 0}},
        {"schema_request", MessageType{0x02}, {0, 0}, {2, 2, 1, 3, 2, 6, 0}},
        {"schema_page",
         MessageType{0x82},
         {133, 114, 115, 247, 0,  0,   2,   0, 2,   1,  1,   5,   115, 116, 97,  116, 101,
          11,  8,   4,   109, 97, 122, 101, 9, 109, 97, 122, 101, 45,  103, 114, 105, 100},
         {6, 130, 133, 114, 115, 247, 1, 2,   2,  29,  2,   1,  1,   5,   115, 116, 97,  116, 101, 11,
          8, 4,   109, 97,  122, 101, 9, 109, 97, 122, 101, 45, 103, 114, 105, 100, 102, 59,  0}},
        {"group_define", MessageType{0x03}, {0, 80, 0, 2, 0, 0, 1, 0}, {2, 3, 2, 80, 2, 2, 1, 2, 1, 3, 86, 89, 0}},
        {"credit", MessageType{0x05}, {69, 35, 1, 0}, {5, 5, 69, 35, 1, 3, 110, 153, 0}},
        {"credit_wrapped", MessageType{0x05}, {16, 255, 255, 255}, {8, 5, 16, 255, 255, 255, 21, 89, 0}},
        {"write", MessageType{0x06}, {2, 0, 0, 0, 128, 63}, {3, 6, 2, 1, 1, 5, 128, 63, 199, 118, 0}},
        {"command", MessageType{0x08}, {5, 0, 0, 0, 0}, {3, 8, 5, 1, 1, 1, 3, 13, 73, 0}},
        {"command_ack", MessageType{0x88}, {5, 3, 2}, {7, 136, 5, 3, 2, 146, 57, 0}},
        {"command_ack_refused", MessageType{0x88}, {0, 2, 1}, {2, 136, 5, 2, 1, 139, 39, 0}},
        {"log",
         MessageType{0x8A},
         {1, 64, 226, 1, 0, 115, 116, 97, 116, 101, 32, 82, 85, 78},
         {6, 138, 1, 64, 226, 1, 12, 115, 116, 97, 116, 101, 32, 82, 85, 78, 232, 159, 0}},
        {"pong", MessageType{0x89}, {0, 4, 0, 0}, {2, 137, 2, 4, 1, 3, 141, 187, 0}},
        {"zeros", MessageType{0x85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {2, 133, 1, 1, 1,   1,   1,
                                                                                        1, 1,   1, 1, 1,   1,   1,
                                                                                        1, 1,   1, 3, 133, 221, 0}},
        {"ramp",
         MessageType{0x8B},
         {0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
          20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
          40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
          60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
          80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
          100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
          120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
          140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
          160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
          180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199},
         {2,   139, 202, 1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,
          19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
          40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
          61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,
          82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,  100, 101, 102,
          103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123,
          124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144,
          145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165,
          166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186,
          187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 149, 49,  0}},
        {"high",
         MessageType{0x89},
         {200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
          220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239},
         {44,  137, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212,
          213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227,
          228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 247, 247, 0}},
    };

// The fields of a message as JSON, written from the same values its payload is built from
class Fields {
public:
    Fields& number(std::string_view key, uint64_t value) { return this->raw(key, std::to_string(value)); }

    Fields& text(std::string_view key, std::string_view value) {
        return this->raw(key, "\"" + std::string{value} + "\"");
    }

    Fields& raw(std::string_view key, std::string_view json) {
        this->body += (this->body.empty() ? "" : ", ") + ("\"" + std::string{key} + "\": ") + std::string{json};
        return *this;
    }

    std::string str() const { return "{" + this->body + "}"; }

private:
    std::string body;
};

// A message of the protocol: which vector it is, its payload built field by field the way the link
// writes it, and the same fields as JSON
struct Message {
    std::string_view     name;
    std::vector<uint8_t> payload;
    std::string          fields;
};

std::vector<uint8_t> build(const std::function<void(Writer&)>& fill) {
    std::vector<uint8_t> buffer(max_payload_size);
    Writer               writer{buffer};
    fill(writer);
    const auto written = writer.done();
    return {written.begin(), written.end()};
}

struct SchemaEntry {
    uint8_t          type;
    uint8_t          access;
    std::string_view name;
    std::string_view type_tag;
};

Message hello_ack() {
    constexpr uint32_t         hash{0xF7737285};
    constexpr uint32_t         boot_id{0x12345678};
    constexpr std::string_view robot{"micras"};

    return {
        "hello_ack",
        build([&](Writer& w) {
            w.u8(protocol_version);
            w.u32(hash);
            w.u16(4);
            w.u32(125);
            w.u16(credit_window);
            w.u32(boot_id);
            w.u8(robot.size());
            w.text(robot);
        }),
        Fields{}
            .number("protocol_version", protocol_version)
            .number("schema_hash", hash)
            .number("variable_count", 4)
            .number("loop_time_us", 125)
            .number("credit_window", credit_window)
            .number("boot_id", boot_id)
            .text("robot_name", robot)
            .str(),
    };
}

Message schema_page() {
    constexpr uint32_t             hash{0xF7737285};
    const std::vector<SchemaEntry> entries{{1, 1, "state", ""}, {11, 8, "maze", "maze-grid"}};

    std::string list;

    for (const SchemaEntry& entry : entries) {
        Fields item;
        item.number("type", entry.type).number("access", entry.access).text("name", entry.name);

        if (entry.type == 11) {
            item.text("type_tag", entry.type_tag);
        }

        list += (list.empty() ? "" : ", ") + item.str();
    }

    return {
        "schema_page",
        build([&](Writer& w) {
            w.u32(hash);
            w.u16(0);
            w.u16(entries.size());
            w.u8(entries.size());

            for (const SchemaEntry& entry : entries) {
                w.u8(entry.type);
                w.u8(entry.access);
                w.u8(entry.name.size());
                w.text(entry.name);

                if (entry.type == 11) {
                    w.u8(entry.type_tag.size());
                    w.text(entry.type_tag);
                }
            }
        }),
        Fields{}
            .number("schema_hash", hash)
            .number("first", 0)
            .number("variable_count", entries.size())
            .raw("entries", "[" + list + "]")
            .str(),
    };
}

Message group_define() {
    const std::vector<uint16_t> ids{0, 1};
    std::string                 list;

    for (const uint16_t id : ids) {
        list += (list.empty() ? "" : ", ") + std::to_string(id);
    }

    return {
        "group_define",
        build([&](Writer& w) {
            w.u8(0);
            w.u16(80);
            w.u8(ids.size());

            for (const uint16_t id : ids) {
                w.u16(id);
            }
        }),
        Fields{}.number("group", 0).number("period", 80).raw("ids", "[" + list + "]").str(),
    };
}

Message u32_message(std::string_view name, std::string_view key, uint32_t value) {
    return {name, build([&](Writer& w) { w.u32(value); }), Fields{}.number(key, value).str()};
}

Message command_ack(std::string_view name, uint8_t code, uint8_t result, uint8_t reason) {
    return {
        name,
        build([&](Writer& w) {
            w.u8(code);
            w.u8(result);
            w.u8(reason);
        }),
        Fields{}.number("code", code).number("result", result).number("reason", reason).str(),
    };
}

std::vector<Message> messages() {
    return {
        {"hello", {}, Fields{}.str()},
        hello_ack(),
        {"schema_request", build([](Writer& w) { w.u16(0); }), Fields{}.number("first", 0).str()},
        schema_page(),
        group_define(),
        u32_message("credit", "consumed_total", 0x00012345),
        u32_message("credit_wrapped", "consumed_total", 0xFFFFFF10),
        {"write", build([](Writer& w) {
             w.u16(2);
             w.u32(std::bit_cast<uint32_t>(1.0F));
         }),
         Fields{}.number("id", 2).raw("value_f32", "1.0").str()},
        {"command", build([](Writer& w) {
             w.u8(5);
             w.u32(0);
         }),
         Fields{}.number("code", 5).number("argument", 0).str()},
        command_ack("command_ack", 5, 3, 2),
        command_ack("command_ack_refused", 0, 2, 1),
        {"log", build([](Writer& w) {
             w.u8(1);
             w.u32(123456);
             w.text("state RUN");
         }),
         Fields{}.number("severity", 1).number("timestamp_us", 123456).text("text", "state RUN").str()},
        u32_message("pong", "sent_total", 0x400),
    };
}

const Vector& find(std::string_view name) {
    for (const Vector& vector : vectors) {
        if (vector.name == name) {
            return vector;
        }
    }

    std::printf("no vector named %.*s\n", int(name.size()), name.data());
    CHECK(false);
    return vectors.front();
}

std::string fields_of(const std::vector<Message>& all, std::string_view name) {
    for (const Message& message : all) {
        if (message.name == name) {
            return message.fields;
        }
    }

    return {};
}

template <typename T>
std::string list_of(const std::vector<T>& values) {
    std::string out;

    for (std::size_t k = 0; k < values.size(); k++) {
        out += (k > 0 ? ", " : "") + std::to_string(int(values[k]));
    }

    return out;
}

std::string to_json(const std::vector<Message>& all) {
    std::ostringstream out;
    out << "{\n  \"protocol_version\": " << int(protocol_version) << ",\n  \"vectors\": [\n";

    for (std::size_t i = 0; i < vectors.size(); i++) {
        const Vector&     vector = vectors[i];
        const std::string fields = fields_of(all, vector.name);

        out << "    {\"name\": \"" << vector.name << "\", \"type\": " << int(vector.type) << ", \"payload\": ["
            << list_of(vector.payload) << "], \"frame\": [" << list_of(vector.frame) << "]";

        if (not fields.empty()) {
            out << ", \"fields\": " << fields;
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

    // Every message of the protocol is built from the values of its fields, and must be its vector
    const std::vector<Message> all = messages();

    for (const Message& message : all) {
        if (find(message.name).payload != message.payload) {
            std::printf("%.*s: the fields do not build the payload\n", int(message.name.size()), message.name.data());
            CHECK(false);
        }
    }

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
        const std::string      json = to_json(all);

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

    std::printf("frame ok: %zu vectors, %zu messages\n", vectors.size(), all.size());
}
