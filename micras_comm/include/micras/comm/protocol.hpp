/**
 * @file
 */

#ifndef MICRAS_COMM_PROTOCOL_HPP
#define MICRAS_COMM_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

#include "micras/core/cobs.hpp"

namespace micras::comm {
/**
 * @brief Version of the wire format, refused by the application when it does not match.
 */
constexpr uint8_t protocol_version{2};

/**
 * @brief Largest payload of a message, not counting the type and the frame check.
 */
constexpr std::size_t max_payload_size{200};

/**
 * @brief Largest size of an encoded frame, including its delimiter.
 */
constexpr std::size_t max_frame_size{core::cobs_encoded_size(max_payload_size + 3) + 1};

/**
 * @brief Number of groups that can be defined at the same time.
 */
constexpr uint8_t max_groups{4};

/**
 * @brief Number of variables a single group can hold.
 */
constexpr uint8_t max_group_variables{16};

/**
 * @brief Number of bytes the robot may have sent on its own initiative that the application has
 * not yet said it consumed.
 *
 * @note The radio module has no flow control and drops silently when its buffer fills, so this is
 * the only thing keeping the firmware from overrunning it.
 */
constexpr uint16_t credit_window{256};

/**
 * @brief Type of a message, which is the first byte of every frame.
 *
 * @note Requests have the top bit clear and responses have it set, so that the direction of a
 * frame can be told from its type alone.
 *
 * @note PING exists because a radio link can stall without either end being told, and because the
 * only other round trip that proves the robot is alive is HELLO, which resets the session.
 *
 * @note The payload of each message is listed next to it, field by field and in order, every
 * integer little endian. A name is its bytes with no terminator, its length given before it.
 */
enum class MessageType : uint8_t {
    HELLO = 0x01,           ///< Empty. Starts a new session.
    SCHEMA_REQUEST = 0x02,  ///< u16 first variable id.
    GROUP_DEFINE = 0x03,    ///< u8 group, u16 period in loop iterations, u8 count, count times u16 id.
    GROUP_ENABLE = 0x04,    ///< u8 group, u8 enable.
    CREDIT = 0x05,          ///< u32 metered bytes consumed since HELLO, wrapping, never going back.
    WRITE = 0x06,           ///< u16 id, the new value.
    READ = 0x07,            ///< u16 id.
    COMMAND = 0x08,         ///< u8 code, u32 argument.
    PING = 0x09,            ///< Empty.

    /**
     * @brief u8 protocol version, u32 schema hash, u16 variable count, u32 loop time in us, u16
     * credit window, u32 boot id, u8 name length, robot name.
     */
    HELLO_ACK = 0x81,

    /**
     * @brief u32 schema hash, u16 first id, u16 variable count, u8 entries, then per entry u8 type
     * code, u8 access flags, u8 name length, name, and for a BLOB only, u8 tag length, type tag.
     */
    SCHEMA_PAGE = 0x82,

    GROUP_ACK = 0x83,    ///< u8 group, u16 period, u16 sample size.
    SAMPLE = 0x85,       ///< u8 group, u16 sequence, u32 timestamp in us, the values in group order.
    WRITE_ACK = 0x86,    ///< u16 id, u8 write status.
    VALUE = 0x87,        ///< u16 id, the value, or the serialized object for a BLOB.
    COMMAND_ACK = 0x88,  ///< u8 code, u8 command result, u8 reason, which the robot defines.
    PONG = 0x89,         ///< Empty.
    LOG = 0x8A,          ///< u8 severity, u32 timestamp in us, text, the rest of the payload.
    ERROR = 0x8F         ///< u8 error code, u16 context.
};

/**
 * @brief Reason a frame could not be acted on.
 */
enum class ErrorCode : uint8_t {
    UNKNOWN_TYPE = 0,
    MALFORMED = 1,
    NO_SUCH_GROUP = 2,
    GROUP_TOO_LARGE = 3,
    NOT_STREAMABLE = 4,
    NO_SUCH_VARIABLE = 5
};

/**
 * @brief Severity of a log message.
 */
enum class Severity : uint8_t {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_PROTOCOL_HPP
