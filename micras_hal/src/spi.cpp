/**
 * @file spi.cpp
 *
 * @brief Proxy SPI Switch class source
 *
 * @date 03/2024
 */

#include "micras/hal/spi.hpp"

namespace micras::hal {
Spi::Spi(const Config& config) : handle{config.handle}, cs_gpio{config.cs_gpio}, timeout{config.timeout} {
    config.init_function();
}

bool Spi::select_device() {
    if (HAL_SPI_GetState(this->handle) != HAL_SPI_STATE_READY) {
        return false;
    }

    this->cs_gpio.write(false);
    return true;
}

void Spi::unselect_device() {
    this->cs_gpio.write(true);
}

void Spi::transmit(std::span<uint8_t> data) {
    HAL_SPI_Transmit(this->handle, data.data(), data.size(), this->timeout);
}

void Spi::receive(std::span<uint8_t> data) {
    HAL_SPI_Receive(this->handle, data.data(), data.size(), this->timeout);
}
}  // namespace micras::hal
