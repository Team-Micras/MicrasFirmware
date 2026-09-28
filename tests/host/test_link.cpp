/**
 * @file
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "micras/comm/frame.hpp"
#include "micras/comm/link.hpp"
#include "micras/core/byte_stream.hpp"
#include "micras/core/variable_pool.hpp"
#include "test_host.hpp"

using namespace micras;
using namespace micras::comm;
using V = std::vector<uint8_t>;

struct Loopback : core::IByteStream {
    std::deque<uint8_t> to_robot;
    std::deque<uint8_t> from_robot;
    std::size_t         capacity{4096};

    std::size_t read(std::span<uint8_t> into) override {
        std::size_t n = std::min(into.size(), to_robot.size());
        for (std::size_t i = 0; i < n; i++) {
            into[i] = to_robot.front();
            to_robot.pop_front();
        }
        return n;
    }

    std::size_t write(std::span<const uint8_t> from) override {
        if (capacity - from_robot.size() < from.size())
            return 0;
        for (uint8_t b : from)
            from_robot.push_back(b);
        return from.size();
    }
};

struct Commands : ICommandHandler {
    std::vector<std::pair<uint8_t, uint32_t>> seen;

    CommandReply handle_command(uint8_t code, uint32_t argument) override {
        seen.emplace_back(code, argument);
        switch (code) {
            case 42:
                return {.result = CommandResult::OK};
            case 43:
                return {.result = CommandResult::REFUSED, .reason = 7};
            case 44:
                return {.result = CommandResult::DEFERRED, .reason = 2};
            default:
                return {.result = CommandResult::UNKNOWN};
        }
    }
};

struct Blob : core::ISerializable {
    static constexpr std::string_view type_tag{"test-blob"};

    std::vector<uint8_t> data{1, 2, 3};

    std::vector<uint8_t> serialize() const override { return data; }

    void deserialize(const uint8_t* p, uint16_t n) override { data.assign(p, p + n); }
};

// --- host side of the protocol ---
static void send(Loopback& io, MessageType type, const V& payload) {
    V frame(max_frame_size);
    frame.resize(encode_frame(type, payload, frame));
    CHECK(!frame.empty());
    for (uint8_t b : frame)
        io.to_robot.push_back(b);
}

static V le32(uint32_t value) {
    return {uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)};
}

struct Msg {
    MessageType type;
    V           payload;
};

static bool is_metered(MessageType type) {
    return type == MessageType::SAMPLE || type == MessageType::SCHEMA_PAGE || type == MessageType::LOG;
}

// Stands in for the application: takes everything that arrived, counts the bytes of every frame the
// window charges for, whole with its delimiter, and returns the total it consumed so far.
struct Application {
    FrameReader rx;
    uint32_t    consumed_total{};
    std::size_t frame_bytes{};

    std::vector<Msg> drain(Loopback& io, bool give_credit = true) {
        std::vector<Msg> out;
        while (!io.from_robot.empty()) {
            uint8_t b = io.from_robot.front();
            io.from_robot.pop_front();
            frame_bytes++;
            if (rx.push(b)) {
                out.push_back({rx.type(), V(rx.payload().begin(), rx.payload().end())});
                if (is_metered(rx.type()))
                    consumed_total += frame_bytes;
            }
            if (b == 0)
                frame_bytes = 0;
        }
        if (give_credit)
            send(io, MessageType::CREDIT, le32(consumed_total));
        return out;
    }

    void restart() { consumed_total = 0; }
};

static const Msg& only(const std::vector<Msg>& msgs, MessageType type) {
    const Msg* found = nullptr;
    for (const Msg& m : msgs) {
        if (m.type == type) {
            CHECK(found == nullptr);
            found = &m;
        }
    }
    CHECK(found != nullptr);
    return *found;
}

static std::vector<Msg> of_type(std::vector<Msg> msgs, MessageType type) {
    msgs.erase(std::remove_if(msgs.begin(), msgs.end(), [type](const Msg& m) { return m.type != type; }), msgs.end());
    return msgs;
}

static uint16_t u16(const V& v, size_t i) {
    return uint16_t(v[i] | v[i + 1] << 8);
}

static uint32_t u32(const V& v, size_t i) {
    return uint32_t(v[i] | v[i + 1] << 8 | v[i + 2] << 16 | uint32_t(v[i + 3]) << 24);
}

static float f32(const V& v, size_t i) {
    uint32_t r = u32(v, i);
    float    f;
    std::memcpy(&f, &r, 4);
    return f;
}

template <typename T>
static T read_variable(const core::VariablePool& pool, std::string_view name) {
    uint8_t buf[sizeof(T)];
    CHECK(pool.read(*pool.find(name), buf) == sizeof(T));
    T value;
    std::memcpy(&value, buf, sizeof(T));
    return value;
}

struct HelloAck {
    uint8_t     version;
    uint32_t    schema_hash;
    uint16_t    count;
    uint32_t    loop_time_us;
    uint16_t    window;
    uint32_t    boot_id;
    std::string name;
};

static HelloAck parse_hello_ack(const V& p) {
    CHECK(p.size() >= 18);
    const uint8_t name_size = p[17];
    CHECK(p.size() == 18U + name_size);
    return {p[0], u32(p, 1), u16(p, 5), u32(p, 7), u16(p, 11), u32(p, 13), std::string(p.begin() + 18, p.end())};
}

struct SchemaEntry {
    uint8_t     type;
    uint8_t     access;
    std::string name;
    std::string tag;
};

static std::map<uint16_t, SchemaEntry> parse_schema(const std::vector<Msg>& msgs, uint32_t hash, uint16_t total) {
    std::map<uint16_t, SchemaEntry> entries;
    for (auto& m : msgs) {
        CHECK(m.type == MessageType::SCHEMA_PAGE);
        CHECK(u32(m.payload, 0) == hash);
        CHECK(u16(m.payload, 6) == total);
        const uint16_t first = u16(m.payload, 4);
        const uint8_t  count = m.payload[8];
        size_t         i = 9;
        for (uint8_t k = 0; k < count; k++) {
            SchemaEntry   entry{m.payload[i], m.payload[i + 1], {}, {}};
            const uint8_t len = m.payload[i + 2];
            entry.name = std::string(m.payload.begin() + i + 3, m.payload.begin() + i + 3 + len);
            i += 3 + len;
            if (entry.type == uint8_t(core::TypeCode::BLOB)) {
                const uint8_t tag_len = m.payload[i];
                entry.tag = std::string(m.payload.begin() + i + 1, m.payload.begin() + i + 1 + tag_len);
                i += 1 + tag_len;
            }
            entries[first + k] = entry;
        }
        CHECK(i == m.payload.size());
    }
    return entries;
}

int main() {
    core::TVariablePool<32> pool;
    float                   linear = 1.0F, angular = 2.0F, gain = 0.5F;
    uint8_t                 profile = 0;
    Blob                    maze;
    pool.add("cmd/", "linear", linear, {.stream = true});
    pool.add("cmd/", "angular", angular, {.stream = true});
    pool.add("model/", "gain", gain, {.stream = true, .write = true, .idle = true, .persist = true});
    pool.add("", "run_profile", profile, {.stream = true, .write = true});
    pool.add("", "maze", maze, {.persist = true});

    Commands    commands;
    Loopback    io;
    Application app;
    Link        link{io, pool, commands, {.loop_time_us = 125, .robot_name = "test-bot"}};
    link.register_variables(pool, "link/");
    const uint16_t total = uint16_t(pool.all().size());

    auto step = [&](uint32_t t, bool idle = true) {
        link.poll(idle);
        link.pump(t);
    };
    auto settle = [&](uint32_t& t, int n = 40, bool idle = true) {
        for (int i = 0; i < n; i++)
            step(t += 125, idle);
    };

    uint32_t now = 1000;

    // --- hello: the protocol, the schema, the window, the boot and the robot ---
    step(now);
    send(io, MessageType::HELLO, {});
    settle(now, 4);
    auto msgs = app.drain(io);
    CHECK(msgs.size() == 1 && msgs[0].type == MessageType::HELLO_ACK);
    const HelloAck hello = parse_hello_ack(msgs[0].payload);
    CHECK(hello.version == protocol_version && protocol_version == 2);
    CHECK(hello.schema_hash == pool.schema_hash());
    CHECK(hello.count == total);
    CHECK(hello.loop_time_us == 125);
    CHECK(hello.window == credit_window && credit_window == 256);
    CHECK(hello.boot_id == 1000);
    CHECK(hello.name == "test-bot");

    // --- the boot id stays for the whole boot, and another boot gets another one ---
    settle(now, 100);
    app.drain(io);
    send(io, MessageType::HELLO, {});
    settle(now, 4);
    msgs = app.drain(io);
    CHECK(parse_hello_ack(only(msgs, MessageType::HELLO_ACK).payload).boot_id == hello.boot_id);
    app.restart();
    {
        Loopback    other_io;
        Application other_app;
        Link        rebooted{other_io, pool, commands, {.loop_time_us = 125, .robot_name = "test-bot"}};
        rebooted.pump(4321);
        send(other_io, MessageType::HELLO, {});
        rebooted.poll(true);
        const HelloAck other = parse_hello_ack(only(other_app.drain(other_io), MessageType::HELLO_ACK).payload);
        CHECK(other.boot_id == 4321 && other.boot_id != hello.boot_id);
    }

    // --- schema, paged, with the type tag of the blob only ---
    send(io, MessageType::SCHEMA_REQUEST, {0, 0});
    settle(now, 40);
    const auto entries = parse_schema(app.drain(io), hello.schema_hash, total);
    CHECK(entries.size() == total);
    CHECK(entries.at(0).name == "cmd/linear" && entries.at(2).name == "model/gain");
    CHECK(entries.at(3).name == "run_profile" && entries.at(3).tag.empty());
    CHECK(entries.at(4).name == "maze" && entries.at(4).type == uint8_t(core::TypeCode::BLOB));
    CHECK(entries.at(4).tag == "test-blob");
    CHECK(entries.at(4).access == core::Access{.persist = true}.to_byte());
    CHECK(entries.at(5).name == "link/dropped_samples");
    CHECK(entries.at(8).name == "link/discarded_frames");

    // --- a group streams at its period, coherently ---
    send(io, MessageType::GROUP_DEFINE, {0, 4, 0, 2, 0, 0, 1, 0});  // group 0, period 4, ids {0,1}
    send(io, MessageType::GROUP_ENABLE, {0, 1});
    settle(now, 4);
    msgs = app.drain(io);
    CHECK(msgs.size() >= 2 && msgs[0].type == MessageType::GROUP_ACK && msgs[1].type == MessageType::GROUP_ACK);
    CHECK(u16(msgs[0].payload, 1) == 4 && u16(msgs[0].payload, 3) == 8);

    linear = 3.5F;
    angular = -1.25F;
    app.drain(io);
    settle(now, 16);
    msgs = app.drain(io);
    CHECK(msgs.size() == 4);  // 16 iterations at period 4

    for (size_t k = 0; k < msgs.size(); k++) {
        CHECK(msgs[k].type == MessageType::SAMPLE);
        CHECK(msgs[k].payload[0] == 0);
        CHECK(u16(msgs[k].payload, 1) == u16(msgs[0].payload, 1) + k);
        CHECK(f32(msgs[k].payload, 7) == 3.5F);
        CHECK(f32(msgs[k].payload, 11) == -1.25F);
    }

    send(io, MessageType::GROUP_ENABLE, {0, 0});
    settle(now, 8);
    app.drain(io);

    // --- writes, with the idle guard ---
    send(io, MessageType::WRITE, {2, 0, 0, 0, 0x80, 0x3F});  // model/gain = 1.0
    settle(now, 4, false);
    msgs = app.drain(io);
    CHECK(only(msgs, MessageType::WRITE_ACK).payload[2] == uint8_t(core::VariablePool::WriteStatus::NEEDS_IDLE));
    CHECK(gain == 0.5F);

    send(io, MessageType::WRITE, {2, 0, 0, 0, 0x80, 0x3F});
    settle(now, 4, true);
    msgs = app.drain(io);
    CHECK(only(msgs, MessageType::WRITE_ACK).payload[2] == uint8_t(core::VariablePool::WriteStatus::OK));
    CHECK(gain == 1.0F);

    // --- read ---
    send(io, MessageType::READ, {2, 0});
    settle(now, 4);
    msgs = app.drain(io);
    CHECK(f32(only(msgs, MessageType::VALUE).payload, 2) == 1.0F);

    // --- commands are edges, acted on once, and the reply says why ---
    send(io, MessageType::COMMAND, {42, 7, 0, 0, 0});
    settle(now, 8);
    msgs = app.drain(io);
    CHECK(commands.seen.size() == 1);
    CHECK(commands.seen[0].first == 42 && commands.seen[0].second == 7);
    CHECK((only(msgs, MessageType::COMMAND_ACK).payload == V{42, uint8_t(CommandResult::OK), 0}));

    send(io, MessageType::COMMAND, {43, 0, 0, 0, 0});
    settle(now, 4);
    CHECK((only(app.drain(io), MessageType::COMMAND_ACK).payload == V{43, uint8_t(CommandResult::REFUSED), 7}));

    send(io, MessageType::COMMAND, {44, 0, 0, 0, 0});
    settle(now, 4);
    CHECK((only(app.drain(io), MessageType::COMMAND_ACK).payload == V{44, uint8_t(CommandResult::DEFERRED), 2}));

    send(io, MessageType::COMMAND, {99, 0, 0, 0, 0});
    settle(now, 4);
    CHECK((only(app.drain(io), MessageType::COMMAND_ACK).payload == V{99, uint8_t(CommandResult::UNKNOWN), 0}));
    CHECK(commands.seen.size() == 4);

    // --- a log carries its severity, its time and its text ---
    link.log(Severity::WARNING, 123456, "state RUN");
    msgs = app.drain(io);
    const V log_payload = only(msgs, MessageType::LOG).payload;
    CHECK(log_payload.size() == 5 + 9);
    CHECK(log_payload[0] == uint8_t(Severity::WARNING));
    CHECK(u32(log_payload, 1) == 123456);
    CHECK(std::string(log_payload.begin() + 5, log_payload.end()) == "state RUN");

    link.log(Severity::INFO, 1, std::string(300, 'x'));
    msgs = app.drain(io);
    CHECK(only(msgs, MessageType::LOG).payload.size() == max_payload_size);

    // --- a closed credit window drops samples and logs, and the sequence shows the gap ---
    app.drain(io);
    send(io, MessageType::HELLO, {});  // resets the totals of the window
    settle(now, 4);
    app.drain(io, false);
    app.restart();
    send(io, MessageType::GROUP_DEFINE, {0, 1, 0, 2, 0, 0, 1, 0});
    send(io, MessageType::GROUP_ENABLE, {0, 1});
    settle(now, 4);
    app.drain(io, false);
    {
        const uint32_t samples_before = read_variable<uint32_t>(pool, "link/dropped_samples");
        const uint32_t logs_before = read_variable<uint32_t>(pool, "link/dropped_logs");

        settle(now, 400);  // far more than the window allows
        link.log(Severity::INFO, now, "dropped, since the window is closed");
        msgs = of_type(app.drain(io, false), MessageType::SAMPLE);
        CHECK(!msgs.empty() && msgs.size() < 40);  // the window, not the loop, set the rate
        CHECK(read_variable<int32_t>(pool, "link/credit") < 20);
        CHECK(read_variable<uint32_t>(pool, "link/dropped_samples") - samples_before + msgs.size() == 400);
        CHECK(read_variable<uint32_t>(pool, "link/dropped_logs") == logs_before + 1);

        const uint16_t last_sent = u16(msgs.back().payload, 1);

        // a credit that was lost costs nothing: the next one carries the same total
        const uint32_t lost_total = app.consumed_total;
        settle(now, 8);
        CHECK(of_type(app.drain(io, false), MessageType::SAMPLE).empty());
        send(io, MessageType::CREDIT, le32(lost_total));
        link.poll(true);
        CHECK(read_variable<int32_t>(pool, "link/credit") == credit_window);
        settle(now, 8);
        msgs = of_type(app.drain(io), MessageType::SAMPLE);
        CHECK(!msgs.empty());
        CHECK(u16(msgs.front().payload, 1) > last_sent + 1);

        // a credit that arrives late, behind the last one, changes nothing
        settle(now, 400);
        app.drain(io, false);
        const int32_t closed = read_variable<int32_t>(pool, "link/credit");
        send(io, MessageType::CREDIT, le32(lost_total));
        link.poll(true);
        CHECK(read_variable<int32_t>(pool, "link/credit") == closed);

        // a total ahead of what was sent opens the window only as far as what was sent
        send(io, MessageType::CREDIT, le32(app.consumed_total + 100000));
        link.poll(true);
        CHECK(read_variable<int32_t>(pool, "link/credit") == credit_window);
    }

    send(io, MessageType::GROUP_ENABLE, {0, 0});
    settle(now, 8);
    app.drain(io);

    // --- a corrupted frame costs only itself, and is counted ---
    app.drain(io);
    const uint32_t    discarded_before = read_variable<uint32_t>(pool, "link/discarded_frames");
    const std::size_t corrupt_at = io.to_robot.size() + 2;
    send(io, MessageType::READ, {0, 0});
    io.to_robot[corrupt_at] ^= 0xFF;  // flip a byte inside the first frame
    send(io, MessageType::READ, {1, 0});
    settle(now, 8);
    msgs = app.drain(io);
    auto values = std::count_if(msgs.begin(), msgs.end(), [](const Msg& m) { return m.type == MessageType::VALUE; });
    CHECK(values == 1);
    for (auto& m : msgs)
        if (m.type == MessageType::VALUE)
            CHECK(u16(m.payload, 0) == 1);
    CHECK(read_variable<uint32_t>(pool, "link/discarded_frames") == discarded_before + 1);

    std::puts("link ok");
}
