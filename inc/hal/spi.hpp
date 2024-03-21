/**
 * @file spi.hpp
 *
 * @brief STM32 SPI HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __SPI_HPP__
#define __SPI_HPP__

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
        };

        /**
         * @brief Construct a new Spi object
         *
         * @param spi_config Configuration for the SPI
         */
        Spi(Config& spi_config);

        /**
         * @brief Activate the chip select
         */
        void select_device();

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
};
}  // namespace hal

#endif // __SPI_HPP__
