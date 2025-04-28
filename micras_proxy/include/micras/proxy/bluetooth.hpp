/**
 * @file
 */

#ifndef MICRAS_PROXY_BLUETOOTH_HPP
#define MICRAS_PROXY_BLUETOOTH_HPP

#include <array>
#include <cstdint>
#include <span>
#include <unordered_map>

#include <vector>
#include <queue>

#include "micras/hal/uart_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the Bluetooth communication.
 *
 * @todo Add ACK to received messages.
 */
class Bluetooth {
public:
    /**
     * @brief Configuration struct for the Bluetooth.
     */
    struct Config {
        hal::UartDma::Config uart;
    };

    /**
     * @brief Construct a new Bluetooth object.
     *
     * @param config Configuration for the UART communication.
     */
    explicit Bluetooth(const Config& config);

    /**
     * @brief Process all received messages and send chosen variables.
     */
    void update();

    void send_data(std::vector<uint8_t> data);

    std::vector<uint8_t> get_data();

private:

    /**
     * @brief Size of the UART buffers.
     */
    static constexpr uint16_t buffer_max_size{1000};

    /**
     * @brief UART for the bluetooth connection.
     */
    hal::UartDma uart;

    /**
     * @brief Buffers for the UART communication.
     */
    std::array<uint8_t, buffer_max_size> rx_buffer{};
    std::array<uint8_t, buffer_max_size> tx_buffer{};

    std::queue<uint8_t> tx_queue{};

    /**
     * @brief Cursor for navigating the buffers.
     */
    uint16_t rx_cursor{};
    uint16_t tx_cursor{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_HPP
