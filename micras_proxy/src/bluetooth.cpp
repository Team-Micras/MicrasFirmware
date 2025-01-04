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
    if (this->uart.is_tx_busy()) {
        return;
    }

    this->rx_cursor = 0;
    this->tx_cursor = 0;

    this->uart.stop_rx_dma();

    while (this->process_message() != Status::NO_MESSAGE) {
        this->rx_cursor++;
    }

    this->uart.start_rx_dma(this->rx_buffer);

    for (const auto& [id, variable] : this->variable_dict) {
        send_variable(this->tx_buffer.at(this->tx_cursor), id, variable);
        this->tx_cursor += variable.size() + 2;
    }

    this->uart.start_tx_dma({this->tx_buffer.data(), this->tx_cursor});
}

Bluetooth::Status Bluetooth::process_message() {
    uint16_t received_bytes = this->rx_buffer.size() - this->uart.get_rx_dma_counter() - 1;

    while ((this->rx_buffer.at(this->rx_cursor) & RxMessage::begin_bits_mask) != RxMessage::Symbols::begin) {
        if (this->rx_cursor >= received_bytes) {
            return Status::NO_MESSAGE;
        }

        this->rx_cursor++;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto&     message = reinterpret_cast<const RxMessage&>(this->rx_buffer.at(this->rx_cursor));
    RxMessage::Size size = message.size;

    if (message.rw == RxMessage::RW::READ) {
        if (message.type.read.end != RxMessage::Symbols::end) {
            return Status::INVALID_MESSAGE;
        }

        uint8_t id = message.type.read.id;

        if (message.type.read.start_stop == RxMessage::StartStop::START) {
            this->variable_dict[id] = std::span<uint8_t>(std::bit_cast<uint8_t*>(message.address), 1 << size);
        } else {
            this->variable_dict.erase(id);
        }

        send_variable(this->tx_buffer.at(this->tx_cursor), 0, {&id, 1});
        return Status::OK;
    }

    switch (size) {
        case RxMessage::Size::BYTE:
            return receive_variable<uint8_t>(message.address, message.type.write_byte);

        case RxMessage::Size::HALF_WORD:
            return receive_variable<uint16_t>(message.address, message.type.write_half_word);

        case RxMessage::Size::WORD:
            return receive_variable<uint32_t>(message.address, message.type.write_word);

        case RxMessage::Size::DOUBLE_WORD:
            return receive_variable<uint64_t>(message.address, message.type.write_double_word);
    }

    return Status::INVALID_MESSAGE;
}
}  // namespace micras::proxy
