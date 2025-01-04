/**
 * @file
 */

#ifndef MICRAS_PROXY_BLUETOOTH_HPP
#define MICRAS_PROXY_BLUETOOTH_HPP

#include <array>
#include <cstdint>
#include <span>
#include <unordered_map>

#include "micras/hal/uart_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the Bluetooth communication.
 *
 * @todo Add ACK to received messages.
 */
class Bluetooth {
public:
    /**
     * @brief Construct a new Bluetooth object.
     *
     * @param config Configuration for the UART communication.
     */
    explicit Bluetooth(const hal::UartDma::Config& config);

    /**
     * @brief Process all received messages and send chosen variables.
     */
    void update();

private:
    /**
     * @brief Union for representing the received messages.
     *
     * @details
     * Follow the protocol:
     *
     *     7      1       32       2     6         1        7
     * | BEGIN | READ | ADDRESS | SIZE | ID | START/STOP | END |
     *
     *     7       1       32       2     8*2^SIZE     7        7
     * | BEGIN | WRITE | ADDRESS | SIZE |  VALUE  | CHECKSUM | END |
     */
    struct __attribute__((__packed__)) RxMessage {
        enum Size : uint8_t {
            BYTE,
            HALF_WORD,
            WORD,
            DOUBLE_WORD,
        };

        enum RW : uint8_t {
            READ = 0,
            WRITE = 1,
        };

        enum StartStop : uint8_t {
            STOP = 0,
            START = 1,
        };

        static constexpr uint8_t begin_bits_mask{0xFE};

        struct Symbols {
            static constexpr uint8_t begin{0x42};
            static constexpr uint8_t end{0x7F};
        };

        union Type {
            struct __attribute__((__packed__)) Read {
                uint8_t id         : 6;
                uint8_t start_stop : 1;
                uint8_t end        : 7;
            };

            template <typename T>
            struct __attribute__((__packed__)) Write {
                T       value    : 8 * sizeof(T);
                uint8_t checksum : 7;
                uint8_t end      : 7;
            };

            Read            read;
            Write<uint8_t>  write_byte;
            Write<uint16_t> write_half_word;
            Write<uint32_t> write_word;
            Write<uint64_t> write_double_word;
        };

        uint8_t  begin   : 7;
        RW       rw      : 1;
        uint32_t address : 32;
        Size     size    : 2;
        Type     type;
    };

    /**
     * @brief Union for representing the messages to be sent.
     *
     * @details
     * Follow the protocol:
     *
     *     5     6   8*sizeof(T)   5
     * | BEGIN | ID |   VALUE   | END |
     *
     * The ACK message has ID 0 and the value is the ID of the message to be acknowledged.
     *
     * @tparam T Type of the variable to send.
     */
    template <typename T>
    struct __attribute__((__packed__)) TxMessage {
        struct Symbols {
            static constexpr uint8_t begin{0x19};
            static constexpr uint8_t end{0x13};
        };

        uint8_t begin : 5;
        uint8_t id    : 6;
        T       value : 8 * sizeof(T);
        uint8_t end   : 5;
    };

    /**
     * @brief Enum for representing the status of the message processing.
     */
    enum Status : uint8_t {
        OK,
        NO_MESSAGE,
        INVALID_MESSAGE
    };

    /**
     * @brief Process a single message from the rx_buffer.
     *
     * @return Status Status of the message processed.
     */
    Status process_message();

    /**
     * @brief Validate the checksum of the message.
     *
     * @param data Data to check.
     * @param checksum Checksum to validate.
     * @return True if the checksum is valid, false otherwise.
     */
    static constexpr bool validate_checksum(uint64_t data, uint8_t checksum) {
        for (uint8_t i = 0; i < 8; i++) {
            checksum ^= data >> (i * 8);
        }

        return checksum == 0xFF;
    }

    /**
     * @brief Receive a variable from the message and write the value to the address.
     *
     * @tparam T Type of the variable.
     * @param address Address of the variable.
     * @param message Message to receive the variable from.
     * @return Status Status of the variable received.
     */
    template <typename T>
    static constexpr Status receive_variable(uint32_t address, const RxMessage::Type::Write<T>& message) {
        if (not validate_checksum(address ^ message.value, message.checksum)) {
            return Status::INVALID_MESSAGE;
        }

        if (message.end != RxMessage::Symbols::end) {
            return Status::INVALID_MESSAGE;
        }

        T* variable = std::bit_cast<T*>(address);
        *variable = message.value;
        return Status::OK;
    }

    /**
     * @brief Write a variable to the tx_buffer.
     *
     * @tparam T Type of the variable.
     * @param start_byte Start byte of the message on the buffer.
     * @param id ID of the variable.
     * @param variable_address Address of the variable on the memory.
     */
    template <typename T>
    static constexpr void write_tx_frame(uint8_t& start_byte, uint8_t id, uint8_t* const variable_address) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        auto& message = reinterpret_cast<TxMessage<T>&>(start_byte);
        message.begin = TxMessage<T>::Symbols::begin;
        message.id = id;
        message.value = *std::bit_cast<T* const>(variable_address);
        message.end = TxMessage<T>::Symbols::end;
    }

    /**
     * @brief Write a variable to the tx_buffer.
     *
     * @param start_byte Start byte of the message on the buffer.
     * @param id ID of the variable.
     * @param variable Variable to send.
     */
    static constexpr void send_variable(uint8_t& start_byte, uint8_t id, std::span<uint8_t> variable) {
        switch (variable.size()) {
            case 1:
                write_tx_frame<uint8_t>(start_byte, id, variable.data());
                break;

            case 2:
                write_tx_frame<uint16_t>(start_byte, id, variable.data());
                break;

            case 4:
                write_tx_frame<uint32_t>(start_byte, id, variable.data());
                break;

            case 8:
                write_tx_frame<uint64_t>(start_byte, id, variable.data());
                break;

            default:
                break;
        }
    }

    /**
     * @brief Size of the UART buffers.
     */
    static constexpr uint16_t buffer_max_size{1000};

    /**
     * @brief UART for the bluetooth connection.
     */
    hal::UartDma uart;

    /**
     * @brief Buffers for the UART communication.
     */
    std::array<uint8_t, buffer_max_size> rx_buffer{};
    std::array<uint8_t, buffer_max_size> tx_buffer{};

    /**
     * @brief Cursor for navigating the buffers.
     */
    uint16_t rx_cursor{};
    uint16_t tx_cursor{};

    /**
     * @brief Map for storing the variables to send.
     */
    std::unordered_map<uint8_t, std::span<uint8_t>> variable_dict;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_HPP
