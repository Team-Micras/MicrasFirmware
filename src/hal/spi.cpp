/**
 * @file spi.cpp
 *
 * @brief Proxy SPI Switch class source
 *
 * @date 03/2024
 */

#include "hal/spi.hpp"

namespace hal {
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

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Spi::transmit(uint8_t data[], uint32_t size) {
    HAL_SPI_Transmit(this->handle, data, size, this->timeout);
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Spi::receive(uint8_t data[], uint32_t size) {
    HAL_SPI_Receive(this->handle, data, size, this->timeout);
}
}  // namespace hal
