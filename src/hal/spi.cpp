/**
 * @file spi.cpp
 *
 * @brief Proxy SPI Switch class source
 *
 * @date 03/2024
 */

#include "hal/spi.hpp"

namespace hal {
Spi::Spi(Config& config) : handle{config.handle}, gpio{config.gpio}, timeout{config.timeout} {
}

void Spi::select_device() {
    if (this->handle->Lock == HAL_LOCKED) {
        return;
    }

    this->gpio.write(false);
}

void Spi::unselect_device() {
    this->gpio.write(true);
}

void Spi::transmit(uint8_t data[], uint32_t size) {
    HAL_SPI_Transmit(this->handle, data, size, this->timeout);
}

void Spi::receive(uint8_t data[], uint32_t size) {
    HAL_SPI_Receive(this->handle, data, size, this->timeout);
}
}  // namespace proxy
