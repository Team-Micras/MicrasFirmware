/**
 * @file
 */

#ifndef MICRAS_HAL_SPI_HPP
#define MICRAS_HAL_SPI_HPP

#include <cstdint>
#include <span>
#include <spi.h>

#include "micras/hal/gpio.hpp"

namespace micras::hal {
/**
 * @brief Class to handle SPI peripheral on STM32 microcontrollers.
 */
class Spi {
public:
    /**
     * @brief SPI configuration struct.
     */
    struct Config {
        void (*init_function)();
        SPI_HandleTypeDef* handle;
        hal::Gpio::Config  cs_gpio;
        uint32_t           timeout;
    };

    /**
     * @brief Construct a new Spi object.
     *
     * @param config Configuration for the SPI.
     */
    explicit Spi(const Config& config);

    /**
     * @brief Activate the chip select.
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
     */
    void transmit(std::span<uint8_t> data);

    /**
     * @brief Receive data over SPI.
     *
     * @param data Data to receive data.
     */
    void receive(std::span<uint8_t> data);

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
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_SPI_HPP
