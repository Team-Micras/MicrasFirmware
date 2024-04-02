/**
 * @file spi.hpp
 *
 * @brief STM32 SPI HAL wrapper
 *
 * @date 03/2024
 */

#ifndef MICRAS_HAL_SPI_HPP
#define MICRAS_HAL_SPI_HPP

#include <cstdint>
#include <spi.h>

#include "hal/gpio.hpp"

namespace hal {
/**
 * @brief Class to handle SPI peripheral on STM32 microcontrollers
 */
class Spi {
    public:
        /**
         * @brief SPI configuration struct
         */
        struct Config {
            SPI_HandleTypeDef* handle;
            void               (* init_function)(void);
            hal::Gpio::Config  gpio;
            uint32_t           timeout;
        };

        /**
         * @brief Construct a new Spi object
         *
         * @param config Configuration for the SPI
         */
        Spi(const Config& config);

        /**
         * @brief Activate the chip select
         *
         * @return true if the device was successfully selected, false otherwise
         */
        bool select_device();

        /**
         * @brief Deactivate the chip select
         */
        void unselect_device();

        /**
         * @brief Transmit data over SPI
         *
         * @param data Data to transmit
         * @param size Size of the buffer
         */
        void transmit(uint8_t data[], uint32_t size);

        /**
         * @brief Receive data over SPI
         *
         * @param data Data to receive data
         * @param size Size of the data
         */
        void receive(uint8_t data[], uint32_t size);

    private:
        /**
         * @brief Handle for the SPI
         */
        SPI_HandleTypeDef* handle;

        /**
         * @brief GPIO for the chip select pin
         */
        hal::Gpio gpio;

        /**
         * @brief Timeout for the SPI operations in ms
         */
        uint32_t timeout;
};
}  // namespace hal

#endif // MICRAS_HAL_SPI_HPP
