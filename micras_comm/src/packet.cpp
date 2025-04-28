#include "micras/comm/packet.hpp"

namespace micras::comm {

Packet::Packet(MessageType type, uint16_t id, const std::vector<uint8_t>& payload) :
    type{type}, id{id}, payload{payload} { }

Packet::Packet(MessageType type, const std::vector<uint8_t>& payload) : Packet(type, 0, payload) { }

Packet::Packet(MessageType type) : Packet(type, 0, {0}) { }

Packet::Packet(const uint8_t* serialized_packet, uint16_t size) :
    Packet(std::vector<uint8_t>(serialized_packet, serialized_packet + size)) { }

Packet::Packet(const std::vector<uint8_t>& serialized_packet) {
    if (not this->is_valid(serialized_packet)) {
        this->type = MessageType::ERROR;
        this->id = 0;
        this->payload = {0};
        return;
    }

    this->type = static_cast<MessageType>(serialized_packet[1]);
    this->id = (serialized_packet[2] << 8) | serialized_packet[3];
    // uint16_t payload_size = (serialized_packet[4] << 8) | serialized_packet[5];
    std::vector<uint8_t> read_payload(serialized_packet.begin() + 6, serialized_packet.end() - 2);
    this->payload = this->unescape_payload(read_payload);
}

std::vector<uint8_t> Packet::serialize() const {
    std::vector<uint8_t> data;

    data.emplace_back(header_byte);

    data.emplace_back(static_cast<uint8_t>(this->id >> 8));
    data.emplace_back(static_cast<uint8_t>(this->id & 0xFF));

    data.emplace_back(static_cast<uint8_t>(this->type));

    data.emplace_back(this->payload.size() >> 8);
    data.emplace_back(this->payload.size() & 0xFF);

    std::vector<uint8_t> escaped_payload = this->escape_payload(this->payload);
    data.insert(data.end(), escaped_payload.begin(), escaped_payload.end());

    uint8_t checksum = 0;
    for (uint16_t i = 1; i < data.size(); ++i) {
        checksum += data[i];
    }
    checksum %= 256;
    data.emplace_back(checksum);

    data.emplace_back(tail_byte);

    return data;
}

std::vector<uint8_t> Packet::escape_payload(const std::vector<uint8_t>& payload) const {
    std::vector<uint8_t> escaped;

    for (auto byte : payload) {
        if ((byte == header_byte) || (byte == tail_byte) || (byte == escape_byte)) {
            escaped.emplace_back(escape_byte);
        }
        escaped.emplace_back(byte);
    }

    return escaped;
}

std::vector<uint8_t> Packet::unescape_payload(const std::vector<uint8_t>& escaped_payload) const {
    std::vector<uint8_t> payload;

    for (uint32_t i = 0; i < escaped_payload.size(); ++i) {
        if (escaped_payload[i] == escape_byte) {
            if (i + 1 < escaped_payload.size()) {
                payload.emplace_back(escaped_payload[i + 1]);
                ++i;
            }
        } else {
            payload.emplace_back(escaped_payload[i]);
        }
    }

    return payload;
}

bool Packet::is_valid(const std::vector<uint8_t>& serialized_packet) {
    if (serialized_packet.size() < minimum_size) {
        return false;
    }

    if (serialized_packet[0] != header_byte || serialized_packet.back() != tail_byte) {
        return false;
    }

    uint8_t checksum = 0;
    for (uint16_t i = 1; i < serialized_packet.size() - 2; ++i) {
        checksum += serialized_packet[i];
    }
    checksum %= 256;

    return checksum == serialized_packet[serialized_packet.size() - 2];
}

Packet::MessageType Packet::get_type() const {
    return this->type;
}

uint16_t Packet::get_id() const {
    return this->id;
}

std::vector<uint8_t> Packet::get_payload() const {
    return this->payload;
}

}  // namespace micras::comm
