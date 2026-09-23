/**
 * @file
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "micras/proxy/bluetooth_serial.hpp"

namespace micras::proxy {
BluetoothSerial::BluetoothSerial(const Config& config, std::span<uint8_t> rx_buffer, std::span<uint8_t> tx_buffer) :
    uart{{.init_function = config.init_function, .handle = config.handle}}, tx_buffer{tx_buffer} {
    this->uart.start_rx(rx_buffer);
}

void BluetoothSerial::update() {
    if (this->uart.is_transmitting()) {
        return;
    }

    this->tail = (this->tail + this->pending) % this->tx_buffer.size();
    this->stored -= this->pending;
    this->pending = 0;

    if (this->stored == 0) {
        return;
    }

    const std::size_t run = std::min(this->stored, this->tx_buffer.size() - this->tail);

    if (this->uart.start_tx(this->tx_buffer.subspan(this->tail, run))) {
        this->pending = run;
    }
}

std::size_t BluetoothSerial::read(std::span<uint8_t> into) {
    return this->uart.read(into);
}

std::size_t BluetoothSerial::write(std::span<const uint8_t> from) {
    if (from.size() > this->tx_buffer.size() - this->stored) {
        return 0;
    }

    const std::size_t first = std::min(from.size(), this->tx_buffer.size() - this->head);

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access) bounded just above
    std::memcpy(&this->tx_buffer[this->head], from.data(), first);

    if (first < from.size()) {
        std::memcpy(this->tx_buffer.data(), &from[first], from.size() - first);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)

    this->head = (this->head + from.size()) % this->tx_buffer.size();
    this->stored += from.size();

    return from.size();
}

bool BluetoothSerial::was_initialized() const {
    return this->uart.was_initialized();
}
}  // namespace micras::proxy
