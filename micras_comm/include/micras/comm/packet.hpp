// start destination source payload_size payload check_sum end

#ifndef MICRAS_COMM_PACKET_HPP
#define MICRAS_COMM_PACKET_HPP

#include <cstdint>
#include <vector>

namespace micras::comm {
/**
 * @brief Class for controlling the communication packets.
 *
 * @note The packet structure is defined as follows:
 * - Header byte: 0x42
 * - message type: 1 byte
 * - id byte: 2 bytes
 * - Payload size: 2 bytes
 * - Payload: variable size
 * - Checksum: 1 byte
 * - Tail byte: 0x7F
 */
class Packet {
public:
    static constexpr uint8_t header_byte{0x42};
    static constexpr uint8_t tail_byte{0x7F};
    static constexpr uint8_t escape_byte{0x7D};
    static constexpr uint8_t minimum_size{7};

    enum class MessageType : uint8_t {
        PING = 0x00,
        PONG = 0x01,
        SERIAL_VARIABLE_MAP_REQUEST = 0x02,
        SERIAL_VARIABLE_MAP_RESPONSE = 0x03,
        SERIAL_VARIABLE = 0x04,
        DEBUG_LOG = 0x05,
        ERROR = 0x06,
    };

    explicit Packet(MessageType type, uint16_t id, const std::vector<uint8_t>& payload);

    explicit Packet(MessageType type, const std::vector<uint8_t>& payload);

    explicit Packet(MessageType type);

    explicit Packet(const std::vector<uint8_t>& serialized_packet);

    explicit Packet(const uint8_t* serialized_packet, uint16_t size);

    std::vector<uint8_t> serialize() const;

    static bool is_valid(const std::vector<uint8_t>& serialized_packet);

    MessageType get_type() const;

    uint16_t get_id() const;

    std::vector<uint8_t> get_payload() const;

private:
    MessageType type;

    uint16_t id;

    std::vector<uint8_t> payload;

    std::vector<uint8_t> escape_payload(const std::vector<uint8_t>& payload) const;

    std::vector<uint8_t> unescape_payload(const std::vector<uint8_t>& escaped_payload) const;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_PACKET_HPP
