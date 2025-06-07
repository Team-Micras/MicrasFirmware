/**
 * @file
 */

#include "micras/hal/uart_dma.hpp"

namespace micras::hal {
UartDma::UartDma(const UartDma::Config& config) : handle{config.handle} {
    config.init_function();
}

void UartDma::start_tx_dma(std::span<uint8_t> buffer) {
    HAL_UART_Transmit_DMA(this->handle, buffer.data(), buffer.size());
}

void UartDma::start_rx_dma(std::span<uint8_t> buffer) {
    HAL_UART_Receive_DMA(this->handle, buffer.data(), buffer.size());
}

void UartDma::stop_rx_dma() {
    // HAL_UART_AbortReceive(this->handle);
    HAL_UART_DMAStop(this->handle);
}

bool UartDma::is_tx_busy() const {
    return this->handle->gState == HAL_UART_STATE_BUSY_TX;
}

uint16_t UartDma::get_rx_dma_counter() const {
    return __HAL_DMA_GET_COUNTER(this->handle->hdmarx);
}
}  // namespace micras::hal
