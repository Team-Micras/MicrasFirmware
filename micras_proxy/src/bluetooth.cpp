/**
 * @file
 */

#include <bit>
#include <algorithm>

#include "micras/proxy/bluetooth.hpp"

namespace micras::proxy {
Bluetooth::Bluetooth(const Config& config) : uart{config.uart} {
    this->uart.start_rx_dma(this->rx_buffer);
}

void Bluetooth::update() {
    process_rx_data();
    process_tx_data();
}

void Bluetooth::process_rx_data() {
    this->uart.stop_rx_dma();

    uint16_t received_bytes = this->rx_buffer.size() - this->uart.get_rx_dma_counter() - 1;

    for (uint16_t i = 0; i < received_bytes; i++) {
        this->received_data.emplace_back(this->rx_buffer.at(i));
    }

    this->uart.start_rx_dma(this->rx_buffer);
}

void Bluetooth::process_tx_data() {
    if (this->tx_queue.empty() || this->uart.is_tx_busy()) {
        return;
    }

    if (this->tx_queue.size() > this->tx_queue_max_size) {
        this->tx_queue.erase(this->tx_queue.begin(), this->tx_queue.begin() + (this->tx_queue.size() - this->tx_queue_max_size));
    }

    uint16_t bytes_to_send = std::min(this->tx_queue.size(), this->buffer_max_size);
    for (uint16_t i = 0; i < bytes_to_send; i++) {
        this->tx_buffer[i] = this->tx_queue[i];
    }

    this->tx_queue.erase(this->tx_queue.begin(), this->tx_queue.begin() + bytes_to_send);

    this->uart.start_tx_dma({this->tx_buffer.data(), bytes_to_send});
}

std::vector<uint8_t> Bluetooth::get_data() {
    std::vector<uint8_t> data = std::move(this->received_data);
    this->received_data.clear();
    return data;
}

void Bluetooth::send_data(std::vector<uint8_t> data) {
    this->tx_queue.insert(this->tx_queue.end(), data.begin(), data.end());
}
}  // namespace micras::proxy
