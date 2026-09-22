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
constexpr uint8_t protocol_version{1};

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
 * @brief Number of bytes the robot may send before the application returns any credit.
 *
 * @note The radio module has no flow control and drops silently when its buffer fills, so this is
 * the only thing keeping the firmware from overrunning it.
 */
constexpr uint16_t initial_credit{256};

/**
 * @brief Type of a message, which is the first byte of every frame.
 *
 * @note Requests have the top bit clear and responses have it set, so that the direction of a
 * frame can be told from its type alone.
 *
 * @note PING exists because a radio link can stall without either end being told, and because the
 * only other round trip that proves the robot is alive is HELLO, which resets the session.
 */
enum class MessageType : uint8_t {
    HELLO = 0x01,
    SCHEMA_REQUEST = 0x02,
    GROUP_DEFINE = 0x03,
    GROUP_ENABLE = 0x04,
    CREDIT = 0x05,
    WRITE = 0x06,
    READ = 0x07,
    COMMAND = 0x08,
    TRACE_ARM = 0x09,
    TRACE_READ = 0x0A,
    PING = 0x0B,

    HELLO_ACK = 0x81,
    SCHEMA_PAGE = 0x82,
    GROUP_ACK = 0x83,
    SAMPLE = 0x85,
    WRITE_ACK = 0x86,
    VALUE = 0x87,
    COMMAND_ACK = 0x88,
    LOG = 0x89,
    TRACE_STATUS = 0x8A,
    TRACE_DATA = 0x8B,
    PONG = 0x8C,
    ERROR = 0x8F
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

/**
 * @brief What starts a trace capture.
 */
enum class TriggerType : uint8_t {
    IMMEDIATE = 0,
    COMMAND = 1,
    ABOVE = 2,
    BELOW = 3
};

/**
 * @brief State of the trace capture.
 */
enum class TraceState : uint8_t {
    IDLE = 0,
    ARMED = 1,
    TRIGGERED = 2,
    FULL = 3
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_PROTOCOL_HPP
