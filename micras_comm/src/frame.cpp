/**
 * @file
 */

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <utility>

#include "micras/comm/frame.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/core/cobs.hpp"

namespace micras::comm {
// The buffers here are all fixed size arrays and spans the caller owns, indexed right after the
// check that bounds the index, so the checked accessors would only repeat the comparison.
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access,cppcoreguidelines-pro-bounds-constant-array-index)
Writer::Writer(std::span<uint8_t> into) : buffer{into} { }

void Writer::u8(uint8_t value) {
    if (this->size >= this->buffer.size()) {
        this->overflowed = true;
        return;
    }

    this->buffer[this->size++] = value;
}

void Writer::u16(uint16_t value) {
    this->u8(value);
    this->u8(value >> 8);
}

void Writer::u32(uint32_t value) {
    this->u16(value);
    this->u16(value >> 16);
}

void Writer::raw(std::span<const uint8_t> bytes) {
    if (bytes.size() > this->buffer.size() - this->size) {
        this->overflowed = true;
        return;
    }

    if (not bytes.empty()) {
        std::memcpy(&this->buffer[this->size], bytes.data(), bytes.size());
        this->size += bytes.size();
    }
}

void Writer::text(std::string_view value) {
    this->raw({std::bit_cast<const uint8_t*>(value.data()), value.size()});
}

std::span<const uint8_t> Writer::done() const {
    return this->overflowed ? std::span<const uint8_t>{} : this->buffer.first(this->size);
}

std::size_t Writer::left() const {
    return this->overflowed ? 0 : this->buffer.size() - this->size;
}

Reader::Reader(std::span<const uint8_t> from) : buffer{from} { }

uint8_t Reader::u8() {
    if (this->index >= this->buffer.size()) {
        this->underflowed = true;
        return 0;
    }

    return this->buffer[this->index++];
}

uint16_t Reader::u16() {
    const uint16_t low = this->u8();
    return static_cast<uint16_t>(low | this->u8() << 8);
}

uint32_t Reader::u32() {
    const uint32_t low = this->u16();
    return low | static_cast<uint32_t>(this->u16()) << 16;
}

std::span<const uint8_t> Reader::rest() const {
    return this->underflowed ? std::span<const uint8_t>{} : this->buffer.subspan(this->index);
}

bool Reader::valid() const {
    return not this->underflowed;
}

static uint16_t fletcher16(std::span<const uint8_t> data) {
    uint16_t low = 0;
    uint16_t high = 0;

    for (const uint8_t byte : data) {
        low = static_cast<uint16_t>((low + byte) % 255);
        high = static_cast<uint16_t>((high + low) % 255);
    }

    return static_cast<uint16_t>(high << 8U | low);
}

std::size_t encode_frame(MessageType type, std::span<const uint8_t> payload, std::span<uint8_t> into) {
    if (payload.size() > max_payload_size or into.empty()) {
        return 0;
    }

    std::array<uint8_t, max_payload_size + 3> plain{};
    plain[0] = std::to_underlying(type);

    if (not payload.empty()) {
        std::memcpy(&plain[1], payload.data(), payload.size());
    }

    const std::size_t checked_size = payload.size() + 1;
    const uint16_t    check = fletcher16(std::span{plain}.first(checked_size));

    plain[checked_size] = check;
    plain[checked_size + 1] = check >> 8;

    const std::size_t size = core::cobs_encode(std::span{plain}.first(checked_size + 2), into.first(into.size() - 1));

    if (size == 0) {
        return 0;
    }

    into[size] = core::cobs_delimiter;
    return size + 1;
}

bool FrameReader::push(uint8_t byte) {
    if (byte != core::cobs_delimiter) {
        if (this->encoded_size >= this->encoded.size()) {
            this->overrun = true;
        } else {
            this->encoded[this->encoded_size++] = byte;
        }

        return false;
    }

    const bool complete = not this->overrun and this->encoded_size > 0 and this->finish();

    if (not complete and (this->overrun or this->encoded_size > 0)) {
        this->discarded_frames++;
    }

    this->encoded_size = 0;
    this->overrun = false;

    return complete;
}

bool FrameReader::finish() {
    const std::size_t size = core::cobs_decode(std::span{this->encoded}.first(this->encoded_size), this->decoded);

    if (size < 3) {
        return false;
    }

    const std::span<const uint8_t> checked = std::span{this->decoded}.first(size - 2);
    const auto expected = static_cast<uint16_t>(this->decoded[size - 2] | this->decoded[size - 1] << 8);

    if (fletcher16(checked) != expected) {
        return false;
    }

    this->decoded_size = size - 2;
    return true;
}

MessageType FrameReader::type() const {
    return static_cast<MessageType>(this->decoded[0]);
}

std::span<const uint8_t> FrameReader::payload() const {
    return std::span{this->decoded}.subspan(1, this->decoded_size - 1);
}

uint32_t FrameReader::discarded() const {
    return this->discarded_frames;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access,cppcoreguidelines-pro-bounds-constant-array-index)
}  // namespace micras::comm
