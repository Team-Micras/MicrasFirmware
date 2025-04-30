#ifndef MICRAS_COMM_COMMUNICATION_SERVICE_HPP
#define MICRAS_COMM_COMMUNICATION_SERVICE_HPP

#include <deque>
#include <functional>
#include <queue>

#include "micras/comm/logger.hpp"
#include "micras/comm/packet.hpp"
#include "micras/comm/serial_variable_pool.hpp"

namespace micras::comm {
/**
 * @brief Class for controlling the communication service.
 *
 * @note The communication service is responsible for sending and receiving data
 *       over the serial interface. It handles incoming packets, processes them,
 *       and sends responses.
 */
class CommunicationService {
public:
    /**
     * @brief Function type for sending data.
     */
    using SendDataFunction = std::function<void(const std::vector<uint8_t>&)>;

    /**
     * @brief Function type for getting data.
     */
    using GetDataFunction = std::function<std::vector<uint8_t>()>;

    /**
     * @brief Construct a new CommunicationService object.
     *
     * @param pool Reference to the SerialVariablePool.
     * @param logger Reference to the Logger.
     */
    explicit CommunicationService(std::shared_ptr<SerialVariablePool>& pool, std::shared_ptr<Logger>& logger);

    /**
     * @brief Register the communication functions for sending and receiving data.
     *
     * @param send_func Function to send data.
     * @param get_func Function to get data.
     */
    void register_communication_functions(SendDataFunction send_func, GetDataFunction get_func);

    /**
     * @brief Update the communication service.
     *
     * This function processes incoming packets, sends debug logs, and sends
     * serial variables.
     */
    void update();

private:
    /**
     * @brief Read data through the registered function and adds incoming data to the queue.
     */
    void update_incoming_packets();

    /**
     * @brief Process incoming packets.
     */
    void process_incomming_packets();

    /**
     * @brief Send debug logs through the registered send function.
     */
    void send_debug_logs();

    /**
     * @brief Send serial variables through the registered send function.
     */
    void send_serial_variables();

    /**
     * @brief Respond to each incoming packet according to its type.
     *
     * @param packet The incoming packet to process.
     */
    void consume_packet(const Packet& packet);

    /**
     * @brief Check if the incoming data queue has a valid packet tail.
     *
     * @param queue The incoming data queue.
     * @return true if the packet tail is valid, false otherwise.
     */
    static bool has_valid_packet_tail(const std::deque<uint8_t>& queue);

    /**
     * @brief Extract a valid packet from the incoming data queue.
     *
     * @param queue The incoming data queue.
     * @return std::vector<uint8_t> The extracted packet data.
     */
    static std::vector<uint8_t> extract_valid_packet(std::deque<uint8_t>& queue);

    /**
     * @brief Pool containing the variables to be sent and received.
     */
    std::shared_ptr<SerialVariablePool> pool;

    /**
     * @brief Logger for logging messages.
     */
    std::shared_ptr<Logger> logger;

    /**
     * @brief Function to send data.
     */
    SendDataFunction send_data_func;

    /**
     * @brief Function to get data.
     */
    GetDataFunction get_data_func;

    /**
     * @brief Flag indicating whether the communication functions are registered.
     */
    bool functions_registered = false;

    /**
     * @brief Queue for incoming raw data.
     */
    std::deque<uint8_t> incoming_data_queue;

    /**
     * @brief Queue for incoming packets.
     */
    std::queue<Packet> incoming_packets;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_COMMUNICATION_SERVICE_HPP
