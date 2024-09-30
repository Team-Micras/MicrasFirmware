/**
 * @file bluetooth.hpp
 *
 * @brief Proxy Bluetooth class declaration
 *
 * @date 09/2024
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
 * @brief Class for controlling the Bluetooth communication
 *
 * @todo Add ACK to received messages
 */
class Bluetooth {
public:
    /**
     * @brief Construct a new Bluetooth object
     *
     * @param config Configuration for the UART communication
     */
    Bluetooth(const hal::UartDma::Config& config);

    /**
     * @brief Process all received messages and send chosen variables
     */
    void update();

private:
    /**
     * @brief Union for representing the received messages
     * Follows the protocol:
     *
     *     7      1       32       2     6         1        7
     * | BEGIN | READ | ADDRESS | SIZE | ID | START/STOP | END |
     *
     *     7       1       32       2     8*2^SIZE     7        7
     * | BEGIN | WRITE | ADDRESS | SIZE |  VALUE  | CHECKSUM | END |
     */
    union RxMessage {
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

        struct __attribute__((__packed__)) Fields {
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

        Fields  fields;
        uint8_t data[15];
    };

    /**
     * @brief Union for representing the messages to be sent
     * Follows the protocol:
     *
     *     5     6   8*sizeof(T)   5
     * | BEGIN | ID |   VALUE   | END |
     *
     * @tparam T
     */
    template <typename T>
    union TxMessage {
        struct __attribute__((__packed__)) Fields {
            uint8_t begin : 5 = 0x19;
            uint8_t id    : 6;
            T       value : 8 * sizeof(T);
            uint8_t end   : 5 = 0x13;
        };

        Fields  fields;
        uint8_t data[sizeof(T) + 2];
    };

    /**
     * @brief Enum for representing the status of the message processing
     */
    enum Status {
        OK,
        NO_MESSAGE,
        INVALID_MESSAGE
    };

    /**
     * @brief Process a single message from the rx_buffer
     *
     * @return Status Status of the message processed
     */
    Status process_message();

    /**
     * @brief Validate the checksum of the message
     *
     * @param data Data to check
     * @param checksum Checksum to validate
     *
     * @return true if the checksum is valid, false otherwise
     */
    bool validate_checksum(uint64_t data, uint8_t checksum);

    /**
     * @brief Send all the variables through the Bluetooth
     */
    void send_variables();

    /**
     * @brief Size of the UART buffers
     */
    static constexpr uint8_t buffer_max_size{50};

    /**
     * @brief UART for the bluetooth connection
     */
    hal::UartDma uart;

    /**
     * @brief Buffers for the UART communication
     */
    std::array<uint8_t, buffer_max_size> rx_buffer{};
    std::array<uint8_t, buffer_max_size> tx_buffer{};

    /**
     * @brief Cursor for navigating the rx_buffer
     */
    uint16_t rx_cursor;

    /**
     * @brief Map for storing the variables to send
     */
    std::unordered_map<uint8_t, std::span<uint8_t>> variable_dict;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_HPP
