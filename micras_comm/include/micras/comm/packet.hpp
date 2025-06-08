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
 * | Offset | Description            | Size (bytes)       |
 * |--------|------------------------|--------------------|
 * | 0      | Header Byte            | 1                  |
 * | 1      | Message Type           | 1                  |
 * | 2      | ID                     | 2                  |
 * | 4      | Payload Size           | 2                  |
 * | 6      | Payload                | Variable           |
 * | ...    | Checksum               | 1                  |
 * | ...    | Tail Byte              | 1                  |
 */
class Packet {
public:
    static constexpr uint8_t header_byte{0x42};
    static constexpr uint8_t tail_byte{0x7F};
    static constexpr uint8_t escape_byte{0x7D};
    static constexpr uint8_t minimum_size{7};

    /**
     * @brief Message types for the communication protocol.
     */
    enum class MessageType : uint8_t {
        PING = 0x00,
        PONG = 0x01,
        SERIAL_VARIABLE_MAP_REQUEST = 0x02,
        SERIAL_VARIABLE_MAP_RESPONSE = 0x03,
        SERIAL_VARIABLE = 0x04,
        DEBUG_LOG = 0x05,
        ERROR = 0x06,
        SERIAL_VARIABLE_CONTROL = 0x07,
    };

    /**
     * @brief Construct a new Packet object.
     *
     * @param type Message type of the packet.
     * @param id ID of the packet.
     * @param payload Payload of the packet.
     */
    explicit Packet(MessageType type, uint16_t id, const std::vector<uint8_t>& payload);

    /**
     * @brief Construct a new Packet object.
     *
     * @param type Message type of the packet.
     * @param payload Payload of the packet.
     */
    explicit Packet(MessageType type, const std::vector<uint8_t>& payload);

    /**
     * @brief Construct a new Packet object.
     *
     * @param type Message type of the packet.
     */
    explicit Packet(MessageType type);

    /**
     * @brief Construct a new Packet object from a serialized packet.
     *
     * @param serialized_packet Serialized packet data.
     */
    explicit Packet(const std::vector<uint8_t>& serialized_packet);

    /**
     * @brief Construct a new Packet object from a serialized packet.
     *
     * @param serialized_packet Serialized packet data.
     * @param size Size of the serialized packet.
     */
    explicit Packet(const uint8_t* serialized_packet, uint16_t size);

    /**
     * @brief Serialize the packet to a byte array.
     *
     * @return std::vector<uint8_t> Serialized packet data.
     */
    std::vector<uint8_t> serialize() const;

    /**
     * @brief Check if the packet is valid.
     *
     * @param serialized_packet Serialized packet data.
     * @return true if the packet is valid.
     * @return false if the packet is invalid.
     */
    static bool is_valid(const std::vector<uint8_t>& serialized_packet);

    /**
     * @brief Get the message type of the packet.
     *
     * @return MessageType Message type of the packet.
     */
    MessageType get_type() const;

    /**
     * @brief Get the ID of the packet.
     *
     * @return uint16_t ID of the packet.
     */
    uint16_t get_id() const;

    /**
     * @brief Get the payload of the packet.
     *
     * @return std::vector<uint8_t> Payload of the packet.
     */
    std::vector<uint8_t> get_payload() const;

private:
    /**
     * @brief Message type of the packet.
     */
    MessageType type;

    /**
     * @brief ID of the packet.
     */
    uint16_t id;

    /**
     * @brief Payload of the packet.
     */
    std::vector<uint8_t> payload;

    /**
     * @brief Escape the payload to avoid special characters.
     *
     * @param payload Unescaped payload data.
     * @return std::vector<uint8_t> Escaped payload data.
     */
    static std::vector<uint8_t> escape_payload(const std::vector<uint8_t>& payload);

    /**
     * @brief Unescape the payload to restore original data.
     *
     * @param escaped_payload Escaped payload data.
     * @return std::vector<uint8_t> Unescaped payload data.
     */
    static std::vector<uint8_t> unescape_payload(const std::vector<uint8_t>& escaped_payload);
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_PACKET_HPP
