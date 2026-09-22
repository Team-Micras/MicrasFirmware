/**
 * @file
 */

#ifndef MICRAS_COMM_FRAME_HPP
#define MICRAS_COMM_FRAME_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "micras/comm/protocol.hpp"

namespace micras::comm {
/**
 * @brief Append little endian values to a payload buffer.
 *
 * @note Every write is checked against the end of the buffer and the first one that does not fit
 * poisons the writer, so a caller only has to check once, at the end.
 */
class Writer {
public:
    /**
     * @brief Construct a new Writer object.
     *
     * @param into Buffer to write into.
     */
    explicit Writer(std::span<uint8_t> into);

    /**
     * @brief Append a value of one, two or four bytes.
     */
    ///@{
    void u8(uint8_t value);
    void u16(uint16_t value);
    void u32(uint32_t value);
    ///@}

    /**
     * @brief Append raw bytes.
     *
     * @param bytes Bytes to append.
     */
    void raw(std::span<const uint8_t> bytes);

    /**
     * @brief Append a string with no length prefix.
     *
     * @param text Text to append.
     */
    void text(std::string_view value);

    /**
     * @brief Get what has been written so far.
     *
     * @return View over the written bytes, empty if anything did not fit.
     */
    std::span<const uint8_t> done() const;

    /**
     * @brief Get how many bytes can still be appended.
     *
     * @return Number of bytes left in the buffer.
     */
    std::size_t left() const;

private:
    std::span<uint8_t> buffer;
    std::size_t        size{};
    bool               overflowed{};
};

/**
 * @brief Take little endian values from a payload buffer.
 *
 * @note Reading past the end yields zero and marks the reader as bad, so a caller only has to
 * check once, after taking everything it expects.
 */
class Reader {
public:
    /**
     * @brief Construct a new Reader object.
     *
     * @param from Buffer to read from.
     */
    explicit Reader(std::span<const uint8_t> from);

    /**
     * @brief Take a value of one, two or four bytes.
     */
    ///@{
    uint8_t  u8();
    uint16_t u16();
    uint32_t u32();
    ///@}

    /**
     * @brief Take everything that is left.
     *
     * @return View over the remaining bytes.
     */
    std::span<const uint8_t> rest() const;

    /**
     * @brief Check whether every value taken so far was inside the buffer.
     *
     * @return True if nothing was read past the end, false otherwise.
     */
    bool valid() const;

private:
    std::span<const uint8_t> buffer;
    std::size_t              index{};
    bool                     underflowed{};
};

/**
 * @brief Build a complete frame around a payload.
 *
 * @param type Type of the message.
 * @param payload Payload of the message.
 * @param into Buffer to build the frame in, which should hold max_frame_size bytes.
 * @return Number of bytes written, or zero if the frame does not fit.
 */
std::size_t encode_frame(MessageType type, std::span<const uint8_t> payload, std::span<uint8_t> into);

/**
 * @brief Recover frames from a stream of bytes.
 *
 * @note Every encoded frame is free of the delimiter by construction, so a corrupt or truncated
 * frame costs exactly the bytes up to the next delimiter and never desynchronizes the reader.
 */
class FrameReader {
public:
    /**
     * @brief Feed one byte.
     *
     * @param byte Byte taken from the stream.
     * @return True if a whole valid frame is now available, false otherwise.
     */
    bool push(uint8_t byte);

    /**
     * @brief Get the type of the frame that was just completed.
     *
     * @return Type of the message.
     */
    MessageType type() const;

    /**
     * @brief Get the payload of the frame that was just completed.
     *
     * @return View over the payload.
     */
    std::span<const uint8_t> payload() const;

    /**
     * @brief Get how many frames were thrown away for failing the frame check or for being
     * malformed.
     *
     * @return Number of frames discarded since the reader was created.
     */
    uint32_t discarded() const;

private:
    /**
     * @brief Decode and check the frame currently in the buffer.
     *
     * @return True if the frame is whole and passes the frame check, false otherwise.
     */
    bool finish();

    /**
     * @brief Bytes of the current frame, still encoded.
     */
    std::array<uint8_t, max_frame_size> encoded{};

    /**
     * @brief Decoded type, payload and frame check of the current frame.
     */
    std::array<uint8_t, max_payload_size + 3> decoded{};

    std::size_t encoded_size{};
    std::size_t decoded_size{};
    uint32_t    discarded_frames{};
    bool        overrun{};
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_FRAME_HPP
