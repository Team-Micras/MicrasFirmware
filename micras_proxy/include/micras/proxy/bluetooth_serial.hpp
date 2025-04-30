/**
 * @file
 */

#ifndef MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
#define MICRAS_PROXY_BLUETOOTH_SERIAL_HPP

#include <array>
#include <cstdint>
#include <vector>

#include "micras/hal/uart_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for handling sending and receiving data over Bluetooth using UART.
 */
class BluetoothSerial {
public:
    /**
     * @brief Configuration struct for the BluetoothSerial.
     */
    struct Config {
        hal::UartDma::Config uart;
    };

    /**
     * @brief Construct a new BluetoothSerial object.
     *
     * @param config Configuration for the BluetoothSerial.
     */
    explicit BluetoothSerial(const Config& config);

    /**
     * @brief Process all received messages and send queued data.
     *
     * This method handles the actual sending and receiving of data over UART.
     * It should be called regularly in the main loop.
     */
    void update();

    /**
     * @brief Queue data to be sent during the next update call.
     *
     * @param data Bytes to be sent over Bluetooth.
     */
    void send_data(std::vector<uint8_t> data);

    /**
     * @brief Get data that has been received since the last call.
     *
     * @return Vector containing received bytes (will be empty if no new data)
     */
    std::vector<uint8_t> get_data();

private:
    /**
     * @brief Process received data from UART
     */
    void process_rx_data();

    /**
     * @brief Send queued data to UART
     */
    void process_tx_data();

    /**
     * @brief Size of the UART buffers.
     */
    static constexpr uint16_t buffer_max_size{1000};

    /**
     * @brief Maximum size of the tx queue.
     */
    static constexpr uint16_t tx_queue_max_size{1500};

    /**
     * @brief UART for the bluetooth connection.
     */
    hal::UartDma uart;

    /**
     * @brief DMA buffers for the UART communication.
     */
    ///@{
    std::array<uint8_t, buffer_max_size> rx_buffer{};
    std::array<uint8_t, buffer_max_size> tx_buffer{};
    ///@}

    /**
     * @brief Buffer for data waiting to be sent in the next update
     */
    std::vector<uint8_t> tx_queue;

    /**
     * @brief Buffer for received data waiting to be retrieved
     */
    std::vector<uint8_t> received_data;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
