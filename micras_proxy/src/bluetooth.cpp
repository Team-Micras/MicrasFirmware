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
            case RxMessage::Size::BYTE: {
                uint8_t write_value = message.fields.type.write_byte.value;

                if (not validate_checksum(
                        message.fields.address ^ write_value, message.fields.type.write_byte.checksum
                    )) {
                    return Status::INVALID_MESSAGE;
                }

                if (message.fields.type.write_byte.end != RxMessage::Symbols::end) {
                    return Status::INVALID_MESSAGE;
                }

                uint8_t* address = std::bit_cast<uint8_t*>(message.fields.address);
                *address = write_value;
                break;
            }
            case RxMessage::Size::HALF_WORD: {
                uint16_t write_value = message.fields.type.write_half_word.value;

                if (not validate_checksum(
                        message.fields.address ^ write_value, message.fields.type.write_half_word.checksum
                    )) {
                    return Status::INVALID_MESSAGE;
                }

                if (message.fields.type.write_half_word.end != RxMessage::Symbols::end) {
                    return Status::INVALID_MESSAGE;
                }

                uint16_t* address = std::bit_cast<uint16_t*>(message.fields.address);
                *address = write_value;
                break;
            }
            case RxMessage::Size::WORD: {
                uint32_t write_value = message.fields.type.write_word.value;

                if (not validate_checksum(
                        message.fields.address ^ write_value, message.fields.type.write_word.checksum
                    )) {
                    return Status::INVALID_MESSAGE;
                }

                if (message.fields.type.write_word.end != RxMessage::Symbols::end) {
                    return Status::INVALID_MESSAGE;
                }

                uint32_t* address = std::bit_cast<uint32_t*>(message.fields.address);
                *address = write_value;
                break;
            }
            case RxMessage::Size::DOUBLE_WORD: {
                uint64_t write_value = message.fields.type.write_double_word.value;

                if (not validate_checksum(
                        message.fields.address ^ write_value, message.fields.type.write_double_word.checksum
                    )) {
                    return Status::INVALID_MESSAGE;
                }

                if (message.fields.type.write_double_word.end != RxMessage::Symbols::end) {
                    return Status::INVALID_MESSAGE;
                }

                uint64_t* address = std::bit_cast<uint64_t*>(message.fields.address);
                *address = write_value;
                break;
            }
        }
    }

    return Status::OK;
}

bool Bluetooth::validate_checksum(uint64_t data, uint8_t checksum) {
    for (uint8_t i = 0; i < 8; i++) {
        checksum ^= data >> (i * 8);
    }

    return checksum == 0xFF;
}

void Bluetooth::send_variables() {
    uint16_t tx_cursor = 0;

    for (auto& [id, variable] : this->variable_dict) {
        switch (variable.size()) {
            case 1: {
                TxMessage<uint8_t>& message = reinterpret_cast<TxMessage<uint8_t>&>(this->tx_buffer.at(tx_cursor));
                message.fields.id = id;
                message.fields.value = variable[0];
                break;
            }
            case 2: {
                TxMessage<uint16_t>& message = reinterpret_cast<TxMessage<uint16_t>&>(this->tx_buffer.at(tx_cursor));
                message.fields.id = id;
                message.fields.value = *std::bit_cast<uint16_t*>(variable.data());
                break;
            }
            case 4: {
                TxMessage<uint32_t>& message = reinterpret_cast<TxMessage<uint32_t>&>(this->tx_buffer.at(tx_cursor));
                message.fields.id = id;
                message.fields.value = *std::bit_cast<uint32_t*>(variable.data());
                break;
            }
            case 8: {
                TxMessage<uint64_t>& message = reinterpret_cast<TxMessage<uint64_t>&>(this->tx_buffer.at(tx_cursor));
                message.fields.id = id;
                message.fields.value = *std::bit_cast<uint64_t*>(variable.data());
                break;
            }
        }

        tx_cursor += variable.size() + 2;
    }

    this->uart.start_tx_dma({this->tx_buffer.data(), tx_cursor});
}

}  // namespace micras::proxy
