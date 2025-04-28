#ifndef MICRAS_COMM_COMMUNICATION_SERVICE_HPP
#define MICRAS_COMM_COMMUNICATION_SERVICE_HPP

#include <deque>
#include <queue>
#include "micras/comm/serial_variable_pool.hpp"
#include "micras/comm/logger.hpp"
#include "micras/comm/packet.hpp"

namespace micras::comm {
class CommunicationService {
public:
    explicit CommunicationService(SerialVariablePool& pool, Logger& logger);

    void update();

private:
    void update_incoming_packets();
    void process_incomming_packets();
    void send_debug_logs();
    void send_serial_variables();
    void consume_packet(const Packet& packet);

    void send_data(const std::vector<uint8_t>& data);

    std::vector<uint8_t> get_data();

    bool has_valid_packet_tail(const std::deque<uint8_t>& queue);

    std::vector<uint8_t> extract_valid_packet(std::deque<uint8_t>& queue);

    SerialVariablePool& pool;

    Logger& logger;

    std::deque<uint8_t> incoming_data_queue{};

    std::queue<Packet> incoming_packets{};

};
}

#endif // MICRAS_COMM_COMMUNICATION_SERVICE_HPP
