/**
 * @file
 */

#ifndef MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
#define MICRAS_PROXY_BLUETOOTH_SERIAL_HPP

#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/core/byte_stream.hpp"
#include "micras/hal/uart_dma.hpp"

namespace micras::proxy {
/**
 * @brief Byte stream over the UART the radio module is wired to.
 *
 * @note The module has no flow control on this board and drops silently when its buffer fills, so
 * the ring here only bounds what the firmware can hand to the peripheral. What bounds what the
 * module can take is the credit window in the session layer.
 */
class BluetoothSerial : public core::IByteStream {
public:
    /**
     * @brief Configuration struct for the bluetooth serial.
     */
    struct Config {
        void (*init_function)();
        UART_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new BluetoothSerial object.
     *
     * @note The buffers are borrowed and have to outlive the object, and both are written by the
     * DMA controller rather than by the processor.
     *
     * @param config Configuration for the bluetooth serial.
     * @param rx_buffer Buffer the reception wraps around, which bounds how long the link can go
     * unpolled before bytes are lost.
     * @param tx_buffer Buffer the frames waiting to be sent are queued in.
     */
    BluetoothSerial(const Config& config, std::span<uint8_t> rx_buffer, std::span<uint8_t> tx_buffer);

    /**
     * @brief Hand the peripheral whatever is queued.
     */
    void update();

    /**
     * @brief Take the bytes that arrived since the last call.
     *
     * @param into Buffer to read into.
     * @return Number of bytes read.
     */
    std::size_t read(std::span<uint8_t> into) override;

    /**
     * @brief Queue data to be sent.
     *
     * @note Either all of the data is queued or none of it is, so a frame is never cut in half.
     * A frame that does not fit is refused, and the session drops it and counts it.
     *
     * @param from Data to send.
     * @return Number of bytes queued.
     */
    std::size_t write(std::span<const uint8_t> from) override;

    /**
     * @brief Check if the peripheral was initialized and is receiving.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    hal::UartDma uart;

    /**
     * @brief Bytes queued to be sent, written at the head and sent from the tail.
     */
    std::span<uint8_t> tx_buffer;

    std::size_t head{};
    std::size_t tail{};

    /**
     * @brief Number of bytes in the ring, including the ones already handed to the DMA.
     */
    std::size_t stored{};

    /**
     * @brief Number of bytes the current transfer is sending, counted from the tail.
     */
    std::size_t pending{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
