#include "micras/comm/communication_service.hpp"

namespace micras::comm {
CommunicationService::CommunicationService(SerialVariablePool& pool, Logger& logger) : pool{pool}, logger{logger} { }

void CommunicationService::register_communication_functions(SendDataFunction send_func, GetDataFunction get_func) {
    this->send_data_func = std::move(send_func);
    this->get_data_func = std::move(get_func);
    this->functions_registered = true;
}

void CommunicationService::update() {
    if (!this->functions_registered) {
        return;
    }

    this->update_incoming_packets();
    this->process_incomming_packets();
    this->send_debug_logs();
    this->send_serial_variables();
}

void CommunicationService::update_incoming_packets() {
    auto data = this->get_data_func();

    for (const auto& byte : data) {
        this->incoming_data_queue.push_back(byte);

        if (this->has_valid_packet_tail(this->incoming_data_queue)) {
            std::vector<uint8_t> packet_data = this->extract_valid_packet(this->incoming_data_queue);

            if (Packet::is_valid(packet_data)) {  // check is redundant
                this->incoming_packets.push(Packet(packet_data));
            }

            this->incoming_data_queue.clear();
        }
    }
}

bool CommunicationService::has_valid_packet_tail(const std::deque<uint8_t>& queue) {
    if (queue.size() < Packet::minimum_size) {
        return false;
    }

    return (queue[queue.size() - 1] != Packet::escape_byte && queue.back() == Packet::tail_byte);
}

std::vector<uint8_t> CommunicationService::extract_valid_packet(std::deque<uint8_t>& queue) {
    std::vector<uint8_t> packet_data;

    while (!queue.empty() && queue.front() != Packet::header_byte) {
        if (queue.front() == Packet::escape_byte) {
            queue.pop_front();
        }

        queue.pop_front();
    }

    packet_data.assign(queue.begin(), queue.end());
    return packet_data;
}

void CommunicationService::process_incomming_packets() {
    while (!this->incoming_packets.empty()) {
        Packet next_packet = this->incoming_packets.front();
        this->incoming_packets.pop();
        this->consume_packet(next_packet);
    }
}

void CommunicationService::send_debug_logs() {
    if (!this->logger.is_enabled()) {
        return;
    }

    while (this->logger.has_logs()) {
        std::string log = this->logger.get_next_log();
        Packet      packet(Packet::MessageType::DEBUG_LOG, {log.begin(), log.end()});
        this->send_data_func(packet.serialize());
    }
}

void CommunicationService::send_serial_variables() {
    this->pool.for_each_read_only_variable([this](uint16_t id, ISerialVariable& variable) {
        Packet packet(Packet::MessageType::SERIAL_VARIABLE, id, variable.serialize());
        this->send_data_func(packet.serialize());
    });
}

void CommunicationService::consume_packet(const Packet& packet) {
    switch (packet.get_type()) {
        case Packet::MessageType::PING:
            this->send_data_func(Packet(Packet::MessageType::PONG).serialize());
            break;

        case Packet::MessageType::SERIAL_VARIABLE_MAP_REQUEST:
            this->send_data_func(this->pool.serialize_var_map());
            break;

        case Packet::MessageType::SERIAL_VARIABLE:
            this->pool.write(packet.get_id(), packet.get_payload());
            break;

        default:
            break;
    }
}
}  // namespace micras::comm
