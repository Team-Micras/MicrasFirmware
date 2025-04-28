#ifndef MICRAS_COMM_COMMUNICATION_SERVICE_HPP
#define MICRAS_COMM_COMMUNICATION_SERVICE_HPP

#include <deque>
#include <queue>
#include <functional>

#include "micras/comm/serial_variable_pool.hpp"
#include "micras/comm/logger.hpp"
#include "micras/comm/packet.hpp"

namespace micras::comm {
class CommunicationService {
public:
    using SendDataFunction = std::function<void(const std::vector<uint8_t>&)>;
    using GetDataFunction = std::function<std::vector<uint8_t>()>;

    explicit CommunicationService(SerialVariablePool& pool, Logger& logger);

    void register_communication_functions(SendDataFunction send_func, GetDataFunction get_func);

    void update();

private:
    void update_incoming_packets();
    void process_incomming_packets();
    void send_debug_logs();
    void send_serial_variables();
    void consume_packet(const Packet& packet);

    bool has_valid_packet_tail(const std::deque<uint8_t>& queue);

    std::vector<uint8_t> extract_valid_packet(std::deque<uint8_t>& queue);

    SerialVariablePool& pool;
    Logger& logger;

    SendDataFunction send_data_func;
    GetDataFunction get_data_func;
    bool functions_registered = false;

    std::deque<uint8_t> incoming_data_queue{};
    std::queue<Packet> incoming_packets{};

};
}

#endif // MICRAS_COMM_COMMUNICATION_SERVICE_HPP
