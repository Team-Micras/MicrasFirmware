/**
 * @file spi.cpp
 *
 * @brief Proxy SPI Switch class source
 *
 * @date 03/2024
 */

#include "hal/spi.hpp"

namespace hal {
Spi::Spi(Config& spi_config) : handle{spi_config.handle}, gpio{spi_config.gpio} {
}

void Spi::select_device() {
    this->gpio.write(false);
}

void Spi::unselect_device() {
    this->gpio.write(true);
}

void Spi::transmit(uint8_t data[], uint32_t size) {
    HAL_SPI_Transmit_DMA(this->handle, data, size);
}

void Spi::receive(uint8_t data[], uint32_t size) {
    HAL_SPI_Receive_DMA(this->handle, data, size);
}
}  // namespace proxy
