/**
 * @file
 */

#ifndef MICRAS_HAL_UART_DMA_HPP
#define MICRAS_HAL_UART_DMA_HPP

#include <cstddef>
#include <cstdint>
#include <main.h>
#include <span>

namespace micras::hal {
/**
 * @brief Class to handle a UART peripheral on STM32 microcontrollers using DMA.
 *
 * @note Reception is a circular DMA that is never stopped, and is polled by comparing the transfer
 * counter against what was already taken. Nothing is done in an interrupt, there is no window
 * between stopping and restarting in which bytes are lost, and the latency is one control loop
 * iteration, which is far below anything the radio adds.
 */
class UartDma {
public:
    /**
     * @brief Configuration struct for the UART.
     */
    struct Config {
        void (*init_function)();
        UART_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new UartDma object.
     *
     * @param config UART configuration struct.
     */
    explicit UartDma(const Config& config);

    /**
     * @brief Special member functions deleted.
     *
     * @note The object owns a transfer in flight that points into its own buffers, so it cannot be
     * copied or moved.
     */
    ///@{
    UartDma(const UartDma&) = delete;
    UartDma(UartDma&&) = delete;
    UartDma& operator=(const UartDma&) = delete;
    UartDma& operator=(UartDma&&) = delete;
    ~UartDma() = default;
    ///@}

    /**
     * @brief Start receiving into a circular buffer.
     *
     * @note The buffer is borrowed and has to outlive the reception. It should live in a memory
     * the DMA controller can reach without going through the bus matrix of another domain.
     *
     * @param buffer Buffer the DMA writes into, wrapping around.
     * @return True if the reception was started, false otherwise.
     */
    bool start_rx(std::span<uint8_t> buffer);

    /**
     * @brief Take the bytes that arrived since the last call.
     *
     * @note Bytes are lost silently if this is called more rarely than the buffer takes to fill,
     * which the framing recovers from by resynchronizing on the next delimiter.
     *
     * @param into Buffer to copy the received bytes into.
     * @return Number of bytes copied.
     */
    std::size_t read(std::span<uint8_t> into);

    /**
     * @brief Start sending a buffer.
     *
     * @note The buffer is borrowed and has to stay valid until the transfer completes.
     *
     * @param from Data to send.
     * @return True if the transfer was started, false if one is still running or it failed.
     */
    bool start_tx(std::span<const uint8_t> from);

    /**
     * @brief Check if a transfer is still running.
     *
     * @return True if the peripheral is still sending, false otherwise.
     */
    bool is_transmitting() const;

    /**
     * @brief Check if the peripheral was initialized and is receiving.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Get how many bytes the DMA has written into the buffer so far.
     *
     * @return Index the DMA will write the next byte at.
     */
    std::size_t rx_head() const;

    /**
     * @brief UART handle.
     */
    UART_HandleTypeDef* handle;

    /**
     * @brief Destination buffer of the reception, written around by the DMA.
     */
    std::span<uint8_t> rx_buffer;

    /**
     * @brief Index of the first byte that has not been taken yet.
     */
    std::size_t rx_tail{};

    /**
     * @brief Whether the reception is running.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_UART_DMA_HPP
