/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "micras/hal/uart_dma.hpp"

namespace micras::hal {
UartDma::UartDma(const Config& config) : handle{config.handle} {
    if (this->handle->gState == HAL_UART_STATE_RESET) {
        config.init_function();
    }
}

bool UartDma::start_rx(std::span<uint8_t> buffer) {
    this->rx_buffer = buffer;
    this->rx_tail = 0;
    this->initialized = HAL_UART_Receive_DMA(this->handle, buffer.data(), buffer.size()) == HAL_OK;

    return this->initialized;
}

std::size_t UartDma::rx_head() const {
    return this->rx_buffer.size() - __HAL_DMA_GET_COUNTER(this->handle->hdmarx);
}

std::size_t UartDma::read(std::span<uint8_t> into) {
    if (this->rx_buffer.empty()) {
        return 0;
    }

    if (this->handle->RxState != HAL_UART_STATE_BUSY_RX) {
        this->start_rx(this->rx_buffer);
        return 0;
    }

    const std::size_t size = this->rx_buffer.size();
    const std::size_t available = (this->rx_head() + size - this->rx_tail) % size;
    const std::size_t taken = std::min(available, into.size());
    const std::size_t first = std::min(taken, size - this->rx_tail);

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access) bounded just above
    std::memcpy(into.data(), &this->rx_buffer[this->rx_tail], first);

    if (first < taken) {
        std::memcpy(&into[first], this->rx_buffer.data(), taken - first);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)

    this->rx_tail = (this->rx_tail + taken) % size;

    return taken;
}

bool UartDma::start_tx(std::span<const uint8_t> from) {
    if (from.empty() or this->is_transmitting()) {
        return false;
    }

    return HAL_UART_Transmit_DMA(this->handle, std::bit_cast<const uint8_t*>(from.data()), from.size()) == HAL_OK;
}

bool UartDma::is_transmitting() const {
    return this->handle->gState == HAL_UART_STATE_BUSY_TX;
}

bool UartDma::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
