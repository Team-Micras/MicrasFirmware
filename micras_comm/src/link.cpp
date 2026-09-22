/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>

#include "micras/comm/frame.hpp"
#include "micras/comm/group.hpp"
#include "micras/comm/link.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/comm/trace.hpp"
#include "micras/core/byte_stream.hpp"
#include "micras/core/serializable.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
Link::Link(
    core::IByteStream& stream, core::VariablePool& pool, Trace& trace, ICommandHandler& commands, const Config& config
) :
    stream{stream}, pool{pool}, trace{trace}, commands{commands}, config{config} {
    this->schema_index = pool.all().size();
}

void Link::register_variables(core::VariablePool& pool, std::string_view prefix) {
    pool.add(prefix, "dropped_samples", this->dropped_samples, {.stream = true});
    pool.add(prefix, "dropped_logs", this->dropped_logs, {.stream = true});
    pool.add(prefix, "credit", this->credit, {.stream = true});
}

void Link::poll(bool robot_is_idle) {
    if (this->consumed == this->staged) {
        this->staged = this->stream.read(this->staging);
        this->consumed = 0;
    }

    while (this->consumed < this->staged) {
        if (this->reader.push(this->staging.at(this->consumed++))) {
            this->execute(robot_is_idle);
            return;
        }
    }
}

void Link::execute(bool robot_is_idle) {
    Reader payload_reader{this->reader.payload()};

    switch (this->reader.type()) {
        case MessageType::HELLO:
            this->on_hello();
            return;

        case MessageType::SCHEMA_REQUEST:
            this->on_schema_request(payload_reader);
            return;

        case MessageType::GROUP_DEFINE:
            this->on_group_define(payload_reader);
            return;

        case MessageType::GROUP_ENABLE:
            this->on_group_enable(payload_reader);
            return;

        case MessageType::CREDIT:
            this->on_credit(payload_reader);
            return;

        case MessageType::WRITE:
            this->on_write(payload_reader, robot_is_idle);
            return;

        case MessageType::READ:
            this->on_read(payload_reader);
            return;

        case MessageType::COMMAND:
            this->on_command(payload_reader);
            return;

        case MessageType::TRACE_ARM:
            this->on_trace_arm(payload_reader);
            return;

        case MessageType::TRACE_READ:
            this->on_trace_read(payload_reader);
            return;

        case MessageType::PING:
            this->send(MessageType::PONG, {});
            return;

        default:
            this->send_error(ErrorCode::UNKNOWN_TYPE, std::to_underlying(this->reader.type()));
            return;
    }
}

void Link::on_hello() {
    for (Group& group : this->groups) {
        group = {};
    }

    this->trace.stop();
    this->trace_dumping = false;
    this->schema_index = this->pool.all().size();
    this->credit = initial_credit;

    Writer writer{this->payload};
    writer.u8(protocol_version);
    writer.u32(this->pool.schema_hash());
    writer.u16(this->pool.all().size());
    writer.u32(this->config.loop_time_us);
    writer.u16(initial_credit);

    this->send(MessageType::HELLO_ACK, writer.done());
}

void Link::on_schema_request(Reader& reader) {
    const uint16_t first = reader.u16();

    if (not reader.valid() or first > this->pool.all().size()) {
        this->send_error(ErrorCode::MALFORMED, first);
        return;
    }

    this->schema_index = first;
}

void Link::on_group_define(Reader& reader) {
    const uint8_t  index = reader.u8();
    const uint16_t period = reader.u16();
    const uint8_t  count = reader.u8();

    if (not reader.valid() or index >= max_groups) {
        this->send_error(ErrorCode::NO_SUCH_GROUP, index);
        return;
    }

    if (count > max_group_variables) {
        this->send_error(ErrorCode::GROUP_TOO_LARGE, count);
        return;
    }

    Group group{};
    group.count = count;
    group.period = std::max<uint16_t>(period, 1);

    for (uint8_t position = 0; position < count; position++) {
        const core::VariableId id = reader.u16();

        if (id >= this->pool.all().size()) {
            this->send_error(ErrorCode::NO_SUCH_VARIABLE, id);
            return;
        }

        const core::Variable& variable = this->pool.at(id);

        if (not variable.access.stream) {
            this->send_error(ErrorCode::NOT_STREAMABLE, id);
            return;
        }

        group.ids.at(position) = id;
        group.sample_size += variable.size;
    }

    if (not reader.valid() or group.sample_size + 7U > max_payload_size) {
        this->send_error(ErrorCode::GROUP_TOO_LARGE, group.sample_size);
        return;
    }

    this->groups.at(index) = group;

    Writer writer{this->payload};
    writer.u8(index);
    writer.u16(group.period);
    writer.u16(group.sample_size);

    this->send(MessageType::GROUP_ACK, writer.done());
}

void Link::on_group_enable(Reader& reader) {
    const uint8_t index = reader.u8();
    const bool    enable = reader.u8() != 0;

    if (not reader.valid() or index >= max_groups or this->groups.at(index).count == 0) {
        this->send_error(ErrorCode::NO_SUCH_GROUP, index);
        return;
    }

    Group& group = this->groups.at(index);
    group.enabled = enable;
    group.counter = 0;

    Writer writer{this->payload};
    writer.u8(index);
    writer.u16(group.period);
    writer.u16(group.sample_size);

    this->send(MessageType::GROUP_ACK, writer.done());
}

void Link::on_credit(Reader& reader) {
    const uint16_t bytes = reader.u16();

    if (reader.valid()) {
        this->credit = std::min<int32_t>(this->credit + bytes, initial_credit);
    }
}

void Link::on_write(Reader& reader, bool robot_is_idle) {
    const core::VariableId id = reader.u16();

    if (not reader.valid()) {
        this->send_error(ErrorCode::MALFORMED, id);
        return;
    }

    const auto status = this->pool.write(id, reader.rest(), robot_is_idle);

    Writer writer{this->payload};
    writer.u16(id);
    writer.u8(std::to_underlying(status));

    this->send(MessageType::WRITE_ACK, writer.done());
}

void Link::on_read(Reader& reader) {
    const core::VariableId id = reader.u16();

    if (not reader.valid() or id >= this->pool.all().size()) {
        this->send_error(ErrorCode::NO_SUCH_VARIABLE, id);
        return;
    }

    Writer writer{this->payload};
    writer.u16(id);

    const core::Variable& variable = this->pool.at(id);

    if (variable.type == core::TypeCode::BLOB) {
        const auto data = static_cast<const core::ISerializable*>(variable.address)->serialize();

        if (data.size() > writer.left()) {
            this->send_error(ErrorCode::GROUP_TOO_LARGE, id);
            return;
        }

        writer.raw(data);
    } else {
        writer.raw({std::bit_cast<const uint8_t*>(variable.address), variable.size});
    }

    this->send(MessageType::VALUE, writer.done());
}

void Link::on_command(Reader& reader) {
    const uint8_t  code = reader.u8();
    const uint32_t argument = reader.u32();

    if (not reader.valid()) {
        this->send_error(ErrorCode::MALFORMED, code);
        return;
    }

    const CommandResult result = this->commands.handle_command(code, argument);

    Writer writer{this->payload};
    writer.u8(code);
    writer.u8(std::to_underlying(result));

    this->send(MessageType::COMMAND_ACK, writer.done());
}

void Link::on_trace_arm(Reader& reader) {
    const uint8_t          index = reader.u8();
    const uint8_t          pre_trigger = reader.u8();
    const auto             trigger = static_cast<TriggerType>(reader.u8());
    const core::VariableId watched = reader.u16();
    const uint32_t         raw_threshold = reader.u32();

    if (not reader.valid() or index >= max_groups) {
        this->send_error(ErrorCode::NO_SUCH_GROUP, index);
        return;
    }

    this->trace_dumping = false;

    if (not this->trace.arm(
            this->groups.at(index), pre_trigger, trigger, watched, std::bit_cast<float>(raw_threshold)
        )) {
        this->send_error(ErrorCode::MALFORMED, index);
        return;
    }

    this->send_trace_status();
}

void Link::on_trace_read(Reader& reader) {
    const uint32_t offset = reader.u32();

    if (not reader.valid()) {
        this->send_error(ErrorCode::MALFORMED, 0);
        return;
    }

    if (this->trace.state() != TraceState::FULL) {
        this->send_trace_status();
        return;
    }

    this->trace_offset = offset;
    this->trace_dumping = true;
}

void Link::send_trace_status() {
    Writer writer{this->payload};
    writer.u8(std::to_underlying(this->trace.state()));
    writer.u32(this->trace.held());
    writer.u32(this->trace.pre_trigger());
    writer.u16(this->trace.sample_size());
    writer.u16(this->trace.period());
    writer.u32(this->trace.timestamp());

    this->send(MessageType::TRACE_STATUS, writer.done());
}

void Link::send_error(ErrorCode code, uint16_t context) {
    Writer writer{this->payload};
    writer.u8(std::to_underlying(code));
    writer.u16(context);

    this->send(MessageType::ERROR, writer.done());
}

void Link::log(Severity severity, std::string_view text) {
    Writer writer{this->payload};
    writer.u8(std::to_underlying(severity));
    writer.text(text.substr(0, std::min<std::size_t>(text.size(), max_payload_size - 1)));

    if (not this->send(MessageType::LOG, writer.done())) {
        this->dropped_logs++;
    }
}

bool Link::send(MessageType type, std::span<const uint8_t> payload) {
    const std::size_t size = encode_frame(type, payload, this->frame);

    if (size == 0 or this->stream.writable() < size) {
        return false;
    }

    this->stream.write(std::span{this->frame}.first(size));
    return true;
}

bool Link::send_metered(MessageType type, std::span<const uint8_t> payload) {
    const std::size_t size = encode_frame(type, payload, this->frame);

    if (size == 0 or std::cmp_less(this->credit, size) or this->stream.writable() < size) {
        return false;
    }

    this->stream.write(std::span{this->frame}.first(size));
    this->credit -= static_cast<int32_t>(size);
    return true;
}

bool Link::send_schema_page() {
    const std::span<const core::Variable> variables = this->pool.all();

    if (this->schema_index >= variables.size()) {
        return false;
    }

    constexpr std::size_t count_offset{8};

    Writer writer{this->payload};
    writer.u32(this->pool.schema_hash());
    writer.u16(this->schema_index);
    writer.u16(variables.size());
    writer.u8(0);

    uint16_t index = this->schema_index;
    uint8_t  count = 0;

    while (index < variables.size() and count < UINT8_MAX) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access) bounded by the loop
        const core::Variable& variable = variables[index];
        const std::size_t     name_size = variable.prefix.size() + variable.name.size();

        if (name_size + 3 > writer.left()) {
            break;
        }

        writer.u8(std::to_underlying(variable.type));
        writer.u8(std::bit_cast<uint8_t>(variable.access));
        writer.u8(name_size);
        writer.text(variable.prefix);
        writer.text(variable.name);

        index++;
        count++;
    }

    if (count == 0) {
        this->schema_index = variables.size();
        return false;
    }

    this->payload.at(count_offset) = count;

    if (not this->send_metered(MessageType::SCHEMA_PAGE, writer.done())) {
        return false;
    }

    this->schema_index = index;
    return true;
}

bool Link::send_trace_block() {
    if (not this->trace_dumping) {
        return false;
    }

    constexpr std::size_t header_size{4};

    const std::size_t taken = this->trace.read(this->trace_offset, std::span{this->payload}.subspan(header_size));

    if (taken == 0) {
        this->trace_dumping = false;
        return false;
    }

    Writer writer{this->payload};
    writer.u32(this->trace_offset);

    if (not this->send_metered(MessageType::TRACE_DATA, std::span{this->payload}.first(header_size + taken))) {
        return false;
    }

    this->trace_offset += taken;
    return true;
}

void Link::pump(uint32_t timestamp_us) {
    for (uint8_t index = 0; index < max_groups; index++) {
        Group& group = this->groups.at(index);

        if (not group.enabled) {
            continue;
        }

        if (group.counter > 0) {
            group.counter--;
            continue;
        }

        group.counter = group.period - 1;

        constexpr std::size_t header_size{7};

        Writer writer{this->payload};
        writer.u8(index);
        writer.u16(group.sequence++);
        writer.u32(timestamp_us);

        group.sample(this->pool, std::span{this->payload}.subspan(header_size, group.sample_size));

        const std::span<const uint8_t> message = std::span{this->payload}.first(header_size + group.sample_size);

        if (not this->send_metered(MessageType::SAMPLE, message)) {
            this->dropped_samples++;
        }
    }

    if (not this->send_schema_page()) {
        this->send_trace_block();
    }
}
}  // namespace micras::comm
