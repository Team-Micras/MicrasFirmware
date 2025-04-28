/**
 * @file
 */

#include <bit>

#include "micras/proxy/bluetooth.hpp"

namespace micras::proxy {
Bluetooth::Bluetooth(const hal::UartDma::Config& config) : uart(config) {
    this->uart.start_rx_dma(this->rx_buffer);
}

void Bluetooth::update() {
}

std::vector<uint8_t> Bluetooth::get_data() {
    std::vector<uint8_t> data;

    this->uart.stop_rx_dma();

    uint16_t received_bytes = this->rx_buffer.size() - this->uart.get_rx_dma_counter() - 1;

    for (uint16_t i = 0; i < received_bytes; i++) {
        data.emplace_back(this->rx_buffer.at(i));
    }

    this->uart.start_rx_dma(this->rx_buffer);

    return data;
}

void Bluetooth::send_data(std::vector<uint8_t> data) {
    if (this->uart.is_tx_busy()) {
        for (const auto& byte : data) {
            this->tx_queue.push(byte);
        }

        return;
    }

    this->tx_cursor = 0;

    while (!this->tx_queue.empty()) {
        this->tx_buffer.at(this->tx_cursor) = this->tx_queue.front();
        this->tx_queue.pop();
        this->tx_cursor += 1;
    }

    for (const auto& byte : data) {
        this->tx_buffer.at(this->tx_cursor) = byte;
        this->tx_cursor += 1;
    }

    this->uart.start_tx_dma({this->tx_buffer.data(), this->tx_cursor});
}
}  // namespace micras::proxy
