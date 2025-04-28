#include "micras/comm/communication_service.hpp"

namespace micras::comm {
CommunicationService::CommunicationService(SerialVariablePool& pool) : pool{pool} { }

void CommunicationService::update() {
    this->update_incoming_packets();
    this->process_incomming_packets();
    this->send_debug_logs();
    this->send_serial_variables();
}

void CommunicationService::update_incoming_packets() {
    auto data = this->receive_data();

    this->incoming_data_queue.insert(this->incoming_data_queue.end(), data.begin(), data.end());
}

void CommunicationService::process_incomming_packets() {
    while (!this->incoming_packets.empty()) {
        Packet next_packet = this->incoming_packets.front();
        this->incoming_packets.pop();
        this->consume_packet(next_packet);
    }
}

void CommunicationService::send_debug_logs() {
}

void CommunicationService::send_serial_variables() {
    this->pool.for_each_read_only_variable([this](uint16_t id, ISerialVariable& variable) {
        Packet packet(Packet::MessageType::SERIAL_VARIABLE, id, variable.serialize());
        this->send_data(packet.serialize());
    });
}

void CommunicationService::consume_packet(const Packet& packet) {
    switch (packet.get_type()) {
        case Packet::MessageType::PING:
            this->send_data(Packet(Packet::MessageType::PONG).serialize());
            break;

        case Packet::MessageType::SERIAL_VARIABLE_MAP_REQUEST:
            this->send_data(this->pool.serialize_var_map());
            break;

        case Packet::MessageType::SERIAL_VARIABLE:
            this->pool.write(packet.get_id(), packet.get_payload());
            break;

        default:
            break;
    }
}
