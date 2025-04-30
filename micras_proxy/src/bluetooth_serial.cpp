/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <span>

#include "micras/proxy/bluetooth_serial.hpp"

namespace micras::proxy {
BluetoothSerial::BluetoothSerial(const Config& config) : uart{config.uart} {
    this->uart.start_rx_dma(this->rx_buffer);
}

void BluetoothSerial::update() {
    process_rx_data();
    process_tx_data();
}

void BluetoothSerial::process_rx_data() {
    this->uart.stop_rx_dma();

    const uint16_t received_bytes = this->rx_buffer.size() - this->uart.get_rx_dma_counter() - 1;

    for (uint16_t i = 0; i < received_bytes; i++) {
        this->received_data.emplace_back(this->rx_buffer.at(i));
    }

    this->uart.start_rx_dma(this->rx_buffer);
}

void BluetoothSerial::process_tx_data() {
    if (this->tx_queue.empty() or this->uart.is_tx_busy()) {
        return;
    }

    if (this->tx_queue.size() > BluetoothSerial::tx_queue_max_size) {
        this->tx_queue.erase(
            this->tx_queue.begin(),
            this->tx_queue.begin() + static_cast<std::vector<uint8_t>::difference_type>(
                                         this->tx_queue.size() - BluetoothSerial::tx_queue_max_size
                                     )
        );
    }

    const uint16_t bytes_to_send =
        std::min(static_cast<uint16_t>(this->tx_queue.size()), BluetoothSerial::buffer_max_size);

    std::copy(this->tx_queue.begin(), this->tx_queue.begin() + bytes_to_send, this->tx_buffer.begin());

    this->tx_queue.erase(this->tx_queue.begin(), this->tx_queue.begin() + bytes_to_send);

    this->uart.start_tx_dma(std::span<uint8_t>(this->tx_buffer.data(), bytes_to_send));
}

std::vector<uint8_t> BluetoothSerial::get_data() {
    std::vector<uint8_t> data = std::move(this->received_data);
    this->received_data.clear();
    return data;
}

void BluetoothSerial::send_data(std::vector<uint8_t> data) {
    this->tx_queue.insert(this->tx_queue.end(), data.begin(), data.end());
}
}  // namespace micras::proxy
