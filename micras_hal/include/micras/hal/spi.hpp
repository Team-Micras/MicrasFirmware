/**
 * @file
 */

#ifndef MICRAS_HAL_SPI_HPP
#define MICRAS_HAL_SPI_HPP

#include <cstdint>
#include <span>

#include <main.h>
#include "micras/hal/gpio.hpp"

namespace micras::hal {
/**
 * @brief Class to handle SPI peripheral on STM32 microcontrollers.
 */
class Spi {
public:
    /**
     * @brief SPI configuration struct.
     *
     * @note The clock polarity and phase belong to the device, not to the bus: several devices with
     * different SPI modes can share one peripheral, and select_device reconfigures it when the mode
     * of the device being selected differs from the one currently programmed.
     */
    struct Config {
        void (*init_function)();
        SPI_HandleTypeDef* handle;
        hal::Gpio::Config  cs_gpio;
        uint32_t           timeout;
        uint32_t           clock_polarity;
        uint32_t           clock_phase;
    };

    /**
     * @brief Construct a new Spi object.
     *
     * @param config Configuration for the SPI.
     */
    explicit Spi(const Config& config);

    /**
     * @brief Activate the chip select, reconfiguring the bus for this device if needed.
     *
     * @note Blocks for up to the configured timeout waiting for the peripheral to become ready, so
     * that an SPI stuck in an error state is reported instead of hanging the caller.
     *
     * @return True if the device was successfully selected, false otherwise.
     */
    bool select_device();

    /**
     * @brief Deactivate the chip select.
     */
    void unselect_device();

    /**
     * @brief Transmit data over SPI.
     *
     * @param data Data to transmit.
     * @return True if the transfer completed, false otherwise.
     */
    bool transmit(std::span<const uint8_t> data);

    /**
     * @brief Receive data over SPI.
     *
     * @param data Buffer to receive data into.
     * @return True if the transfer completed, false otherwise.
     */
    bool receive(std::span<uint8_t> data);

    /**
     * @brief Transmit and receive data over SPI in the same transfer.
     *
     * @param transmitted Data to transmit.
     * @param received Buffer to receive data into, at least as large as the transmitted one.
     * @return True if the transfer completed, false otherwise.
     */
    bool transmit_receive(std::span<const uint8_t> transmitted, std::span<uint8_t> received);

    /**
     * @brief Check if the SPI was successfully initialized.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Handle for the SPI.
     */
    SPI_HandleTypeDef* handle;

    /**
     * @brief GPIO for the chip select pin.
     */
    hal::Gpio cs_gpio;

    /**
     * @brief Timeout for the SPI operations in ms.
     */
    uint32_t timeout;

    /**
     * @brief Clock polarity required by the device behind this chip select.
     */
    uint32_t clock_polarity;

    /**
     * @brief Clock phase required by the device behind this chip select.
     */
    uint32_t clock_phase;

    /**
     * @brief Flag to check if the SPI was initialized.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_SPI_HPP
