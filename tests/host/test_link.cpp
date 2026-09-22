/**
 * @file
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "micras/comm/frame.hpp"
#include "micras/comm/link.hpp"
#include "micras/comm/trace.hpp"
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
        if (writable() < from.size())
            return 0;
        for (uint8_t b : from)
            from_robot.push_back(b);
        return from.size();
    }

    std::size_t writable() const override { return capacity - from_robot.size(); }
};

struct Commands : ICommandHandler {
    std::vector<std::pair<uint8_t, uint32_t>> seen;

    CommandResult handle_command(uint8_t code, uint32_t argument) override {
        seen.emplace_back(code, argument);
        return code == 42 ? CommandResult::OK : CommandResult::UNKNOWN;
    }
};

// --- host side of the protocol ---
static void send(Loopback& io, MessageType type, const V& payload) {
    V frame(max_frame_size);
    frame.resize(encode_frame(type, payload, frame));
    CHECK(!frame.empty());
    for (uint8_t b : frame)
        io.to_robot.push_back(b);
}

struct Msg {
    MessageType type;
    V           payload;
};

// Stands in for the application: takes everything that arrived and returns the credit for it,
// which is what keeps the window open.
static std::vector<Msg> drain(Loopback& io) {
    static FrameReader rx;
    std::vector<Msg>   out;
    std::size_t        taken = io.from_robot.size();
    while (!io.from_robot.empty()) {
        uint8_t b = io.from_robot.front();
        io.from_robot.pop_front();
        if (rx.push(b))
            out.push_back({rx.type(), V(rx.payload().begin(), rx.payload().end())});
    }
    if (taken > 0)
        send(io, MessageType::CREDIT, {uint8_t(taken), uint8_t(taken >> 8)});
    return out;
}

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

int main() {
    core::TVariablePool<32> pool;
    float                   linear = 1.0F, angular = 2.0F, gain = 0.5F;
    uint8_t                 profile = 0;
    pool.add("cmd/", "linear", linear, {.stream = true});
    pool.add("cmd/", "angular", angular, {.stream = true});
    pool.add("model/", "gain", gain, {.stream = true, .write = true, .idle = true, .persist = true});
    pool.add("", "run_profile", profile, {.stream = true, .write = true});

    std::vector<uint8_t> ring(1024);
    Trace                trace{pool, ring};
    Commands             commands;
    Loopback             io;
    Link                 link{io, pool, trace, commands, {.loop_time_us = 125}};

    auto step = [&](uint32_t t, bool idle = true) {
        link.poll(idle);
        link.pump(t);
        trace.sample(t);
    };
    auto settle = [&](uint32_t& t, int n = 40, bool idle = true) {
        for (int i = 0; i < n; i++)
            step(t += 125, idle);
    };

    uint32_t now = 0;

    // --- hello ---
    send(io, MessageType::HELLO, {});
    settle(now, 4);
    auto msgs = drain(io);
    CHECK(msgs.size() == 1 && msgs[0].type == MessageType::HELLO_ACK);
    CHECK(msgs[0].payload[0] == protocol_version);
    const uint32_t hash = u32(msgs[0].payload, 1);
    CHECK(hash == pool.schema_hash());
    CHECK(u16(msgs[0].payload, 5) == 4);
    CHECK(u32(msgs[0].payload, 7) == 125);

    // --- schema, paged ---
    send(io, MessageType::SCHEMA_REQUEST, {0, 0});
    settle(now, 40);
    msgs = drain(io);
    std::map<uint16_t, std::string> names;
    for (auto& m : msgs) {
        CHECK(m.type == MessageType::SCHEMA_PAGE);
        CHECK(u32(m.payload, 0) == hash);
        uint16_t first = u16(m.payload, 4), total = u16(m.payload, 6);
        CHECK(total == 4);
        uint8_t count = m.payload[8];
        size_t  i = 9;
        for (uint8_t k = 0; k < count; k++) {
            uint8_t len = m.payload[i + 2];
            names[first + k] = std::string(m.payload.begin() + i + 3, m.payload.begin() + i + 3 + len);
            i += 3 + len;
        }
    }
    CHECK(names.size() == 4);
    CHECK(names[0] == "cmd/linear" && names[2] == "model/gain" && names[3] == "run_profile");

    // --- a group streams at its period, coherently ---
    send(io, MessageType::GROUP_DEFINE, {0, 4, 0, 2, 0, 0, 1, 0});  // group 0, period 4, ids {0,1}
    send(io, MessageType::GROUP_ENABLE, {0, 1});
    settle(now, 4);
    msgs = drain(io);
    CHECK(msgs.size() >= 2 && msgs[0].type == MessageType::GROUP_ACK && msgs[1].type == MessageType::GROUP_ACK);
    CHECK(u16(msgs[0].payload, 1) == 4 && u16(msgs[0].payload, 3) == 8);

    linear = 3.5F;
    angular = -1.25F;
    drain(io);
    settle(now, 16);
    msgs = drain(io);
    CHECK(msgs.size() == 4);  // 16 iterations at period 4

    for (size_t k = 0; k < msgs.size(); k++) {
        CHECK(msgs[k].type == MessageType::SAMPLE);
        CHECK(msgs[k].payload[0] == 0);
        CHECK(u16(msgs[k].payload, 1) == u16(msgs[0].payload, 1) + k);
        CHECK(f32(msgs[k].payload, 7) == 3.5F);
        CHECK(f32(msgs[k].payload, 11) == -1.25F);
    }

    // --- writes, with the idle guard ---
    send(io, MessageType::WRITE, {2, 0, 0, 0, 0x80, 0x3F});  // model/gain = 1.0
    settle(now, 4, false);
    msgs = drain(io);
    CHECK(only(msgs, MessageType::WRITE_ACK).payload[2] == uint8_t(core::VariablePool::WriteStatus::NEEDS_IDLE));
    CHECK(gain == 0.5F);

    send(io, MessageType::WRITE, {2, 0, 0, 0, 0x80, 0x3F});
    settle(now, 4, true);
    msgs = drain(io);
    CHECK(only(msgs, MessageType::WRITE_ACK).payload[2] == uint8_t(core::VariablePool::WriteStatus::OK));
    CHECK(gain == 1.0F);

    // --- read ---
    send(io, MessageType::READ, {2, 0});
    settle(now, 4);
    msgs = drain(io);
    CHECK(f32(only(msgs, MessageType::VALUE).payload, 2) == 1.0F);

    // --- commands are edges, acted on once ---
    send(io, MessageType::COMMAND, {42, 7, 0, 0, 0});
    settle(now, 8);
    msgs = drain(io);
    CHECK(commands.seen.size() == 1);
    CHECK(commands.seen[0].first == 42 && commands.seen[0].second == 7);
    CHECK(only(msgs, MessageType::COMMAND_ACK).payload[1] == uint8_t(CommandResult::OK));

    // --- a closed credit window drops samples and the sequence shows the gap ---
    drain(io);
    send(io, MessageType::HELLO, {});  // resets credit to the initial window
    settle(now, 4);
    drain(io);
    send(io, MessageType::GROUP_DEFINE, {0, 1, 0, 2, 0, 0, 1, 0});
    send(io, MessageType::GROUP_ENABLE, {0, 1});
    settle(now, 4);
    drain(io);
    {
        core::TVariablePool<8> probe;
        link.register_variables(probe, "link/");

        uint8_t  baseline_buf[4];
        uint32_t baseline = 0;
        probe.read(*probe.find("link/dropped_samples"), baseline_buf);
        std::memcpy(&baseline, baseline_buf, 4);

        settle(now, 400);  // far more than the window allows
        msgs = drain(io);
        msgs.erase(
            std::remove_if(msgs.begin(), msgs.end(), [](const Msg& m) { return m.type != MessageType::SAMPLE; }),
            msgs.end()
        );
        CHECK(!msgs.empty() && msgs.size() < 40);  // the window, not the loop, set the rate

        uint8_t  buf[4];
        uint32_t dropped = 0;
        probe.read(*probe.find("link/dropped_samples"), buf);
        std::memcpy(&dropped, buf, 4);
        CHECK(dropped - baseline + msgs.size() == 400);  // every attempt is either sent or counted

        const uint16_t last_sent = u16(msgs.back().payload, 1);

        // returning credit starts it again, and the sequence shows what was missed
        send(io, MessageType::CREDIT, {0x00, 0x01});
        settle(now, 8);
        msgs = drain(io);
        msgs.erase(
            std::remove_if(msgs.begin(), msgs.end(), [](const Msg& m) { return m.type != MessageType::SAMPLE; }),
            msgs.end()
        );
        CHECK(!msgs.empty());
        CHECK(u16(msgs.front().payload, 1) > last_sent + 1);
    }

    send(io, MessageType::GROUP_ENABLE, {0, 0});
    settle(now, 8);
    drain(io);

    // --- a corrupted frame costs only itself ---
    drain(io);
    const std::size_t corrupt_at = io.to_robot.size() + 2;
    send(io, MessageType::READ, {0, 0});
    io.to_robot[corrupt_at] ^= 0xFF;  // flip a byte inside the first frame
    send(io, MessageType::READ, {1, 0});
    settle(now, 8);
    msgs = drain(io);
    auto values = std::count_if(msgs.begin(), msgs.end(), [](const Msg& m) { return m.type == MessageType::VALUE; });
    CHECK(values == 1);
    for (auto& m : msgs)
        if (m.type == MessageType::VALUE)
            CHECK(u16(m.payload, 0) == 1);

    // --- trace: full rate capture with pre-trigger, read out afterwards ---
    drain(io);
    send(io, MessageType::GROUP_DEFINE, {1, 1, 0, 1, 0, 0});  // group 1, period 1, just cmd/linear
    settle(now, 4);
    drain(io);

    // trigger when cmd/linear rises through 10
    V arm{1, 50, uint8_t(TriggerType::ABOVE), 0, 0, 0, 0, 0x20, 0x41};  // threshold 10.0F
    linear = 0.0F;
    send(io, MessageType::TRACE_ARM, arm);
    settle(now, 4);
    msgs = drain(io);
    CHECK(only(msgs, MessageType::TRACE_STATUS).payload[0] == uint8_t(TraceState::ARMED));

    const std::size_t capacity = ring.size() / 4;
    const std::size_t pre = capacity / 2;

    for (std::size_t i = 0; i < capacity; i++) {
        linear = -float(i);
        step(now += 125);
    }
    CHECK(trace.state() == TraceState::ARMED);  // nothing crossed the threshold yet

    for (std::size_t i = 0; i < pre; i++) {
        linear = 50.0F + float(i);
        step(now += 125);
    }
    CHECK(trace.state() == TraceState::FULL);
    CHECK(trace.held() == capacity);
    CHECK(trace.pre_trigger() == pre);
    CHECK(trace.sample_size() == 4);
    CHECK(trace.period() == 1);

    drain(io);
    send(io, MessageType::TRACE_READ, {0, 0, 0, 0});

    // the readout runs at whatever rate the application returns credit at, which is the point
    V dump;
    for (int round = 0; round < 200 && dump.size() < capacity * 4; round++) {
        settle(now, 8);
        msgs = drain(io);

        for (auto& m : msgs) {
            if (m.type != MessageType::TRACE_DATA)
                continue;
            CHECK(u32(m.payload, 0) == dump.size());
            dump.insert(dump.end(), m.payload.begin() + 4, m.payload.end());
        }
    }
    CHECK(dump.size() == capacity * 4);

    // the oldest half is what preceded the trigger, the newest half what followed it
    for (std::size_t i = 0; i < pre; i++) {
        CHECK(f32(dump, i * 4) == -float(pre + i));
    }
    for (std::size_t i = 0; i < capacity - pre; i++) {
        CHECK(f32(dump, (pre + i) * 4) == 50.0F + float(i));
    }

    std::puts("link ok");
}
