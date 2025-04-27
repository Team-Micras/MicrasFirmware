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
 * - Payload type: 1 byte
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

    enum class PayloadType : uint8_t {
        ping = 0x00,
        pong = 0x01,
        serial_variable_map_request = 0x02,
        serial_variable_map_response = 0x03,
        serial_variable = 0x04,
        debug_log = 0x05,
        error = 0x06,
    };

    explicit Packet(PayloadType type, const std::vector<uint8_t>& payload);

    explicit Packet(PayloadType type);

    explicit Packet(const std::vector<uint8_t>& serialized_packet);

    explicit Packet(const uint8_t* serialized_packet, uint16_t size);

    std::vector<uint8_t> serialize() const;

    void deserialize(const uint8_t* buffer, uint16_t size);

    bool is_valid(const std::vector<uint8_t>& serialized_packet);

private:
    PayloadType type;

    std::vector<uint8_t> payload;

    std::vector<uint8_t> escape_payload(const std::vector<uint8_t>& payload) const;

    std::vector<uint8_t> unescape_payload(const std::vector<uint8_t>& escaped_payload) const;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_PACKET_HPP
