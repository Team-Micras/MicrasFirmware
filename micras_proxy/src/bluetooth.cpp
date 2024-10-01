#include <bit>

#include "micras/proxy/bluetooth.hpp"

namespace micras::proxy {
Bluetooth::Bluetooth(const hal::UartDma::Config& config) : uart(config) {
    this->uart.start_rx_dma(this->rx_buffer);
}

void Bluetooth::update() {
    this->uart.stop_rx_dma();
    this->rx_cursor = 0;

    while (this->process_message() != Status::NO_MESSAGE) {
        this->rx_cursor++;
    }

    this->uart.start_rx_dma(this->rx_buffer);
    this->send_variables();
}

Bluetooth::Status Bluetooth::process_message() {
    uint16_t received_bytes = this->rx_buffer.size() - this->uart.get_rx_dma_counter() - 1;

    while ((this->rx_buffer.at(this->rx_cursor) & RxMessage::begin_bits_mask) != RxMessage::Symbols::begin) {
        if (this->rx_cursor >= received_bytes) {
            return Status::NO_MESSAGE;
        }

        this->rx_cursor++;
    }

    const RxMessage& message = reinterpret_cast<const RxMessage&>(this->rx_buffer.at(this->rx_cursor));

    RxMessage::Size size = message.fields.size;

    if (message.fields.rw == RxMessage::RW::READ) {
        if (message.fields.type.read.end != RxMessage::Symbols::end) {
            return Status::INVALID_MESSAGE;
        }

        uint8_t id = message.fields.type.read.id;

        if (message.fields.type.read.start_stop == RxMessage::StartStop::START) {
            this->variable_dict[id] = std::span<uint8_t>(std::bit_cast<uint8_t*>(message.fields.address), 1 << size);
        } else {
            this->variable_dict.erase(id);
        }
    } else {
        switch (size) {
            case RxMessage::Size::BYTE:
                return receive_variable<uint8_t>(message.fields.address, message.fields.type.write_byte);

            case RxMessage::Size::HALF_WORD:
                return receive_variable<uint16_t>(message.fields.address, message.fields.type.write_half_word);

            case RxMessage::Size::WORD:
                return receive_variable<uint32_t>(message.fields.address, message.fields.type.write_word);

            case RxMessage::Size::DOUBLE_WORD:
                return receive_variable<uint64_t>(message.fields.address, message.fields.type.write_double_word);
        }
    }

    return Status::OK;
}

void Bluetooth::send_variables() {
    uint16_t tx_cursor = 0;

    for (auto& [id, variable] : this->variable_dict) {
        switch (variable.size()) {
            case 1:
                write_tx_frame<uint8_t>(this->tx_buffer.at(tx_cursor), id, variable.data());
                break;

            case 2:
                write_tx_frame<uint16_t>(this->tx_buffer.at(tx_cursor), id, variable.data());
                break;

            case 4:
                write_tx_frame<uint32_t>(this->tx_buffer.at(tx_cursor), id, variable.data());
                break;

            case 8:
                write_tx_frame<uint64_t>(this->tx_buffer.at(tx_cursor), id, variable.data());
                break;
        }

        tx_cursor += variable.size() + 2;
    }

    this->uart.start_tx_dma({this->tx_buffer.data(), tx_cursor});
}
}  // namespace micras::proxy
