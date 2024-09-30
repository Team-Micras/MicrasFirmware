/**
 * @file uart_dma.hpp
 *
 * @brief HAL UART DMA class header
 *
 * @date 09/2024
 */

#ifndef MICRAS_HAL_UART_DMA_HPP
#define MICRAS_HAL_UART_DMA_HPP

#include <cstdint>
#include <span>
#include <usart.h>

namespace micras::hal {
/**
 * @brief Class to handle UART DMA on STM32 microcontrollers
 */
class UartDma {
public:
    /**
     * @brief UART configuration struct
     */
    struct Config {
        void (*init_function)();
        UART_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new UartDma object
     *
     * @param config Configuration for the UART
     */
    explicit UartDma(const Config& config);

    /**
     * @brief Start the TX DMA transfer
     *
     * @param buffer Buffer to transmit
     */
    void start_tx_dma(std::span<uint8_t> buffer);

    /**
     * @brief Start the RX DMA transfer
     *
     * @param buffer Buffer to receive data into
     */
    void start_rx_dma(std::span<uint8_t> buffer);

    /**
     * @brief Stop the RX DMA transfer
     */
    void stop_rx_dma();

    /**
     * @brief Get the number of bytes left in the buffer
     *
     * @return uint16_t Number of bytes left in the buffer
     */
    uint16_t get_rx_dma_counter() const;

private:
    /**
     * @brief Handle for the UART
     */
    UART_HandleTypeDef* handle;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_UART_DMA_HPP
