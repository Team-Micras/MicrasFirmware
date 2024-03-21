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
#include <functional>
#include <spi.h>

#include "hal/gpio.hpp"

namespace hal {
/**
 * @brief Class to handle SPI peripheral on STM32 microcontrollers
 */
template <uint8_t num_of_devices>
class Spi {
    public:
        /**
         * @brief SPI configuration struct
         */
        struct Config {
            SPI_HandleTypeDef*                            handle;
            std::function<void(void)>                     init_function;
            std::array<hal::Gpio::Config, num_of_devices> gpio_array;
        };

        /**
         * @brief Construct a new Spi object
         *
         * @param spi_config Configuration for the SPI
         */
        Spi(const Config& spi_config);

        /**
         * @brief Activate the chip select for a device
         */
        void select_device(uint8_t device);

        /**
         * @brief Deactivate the chip select for a device
         */
        void unselect_device(uint8_t device);

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
         * @brief Array of GPIOs for the switches
         */
        std::array<hal::Gpio, num_of_devices> gpio_array;
};
}  // namespace hal

#endif // __SPI_HPP__
