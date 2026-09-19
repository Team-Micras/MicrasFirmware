/**
 * @file
 */

#include <cstdint>
#include <span>

#include "micras/hal/spi.hpp"
#include "micras/hal/timer.hpp"

namespace micras::hal {
Spi::Spi(const Config& config) :
    handle{config.handle},
    cs_gpio{config.cs_gpio},
    timeout{config.timeout},
    clock_polarity{config.clock_polarity},
    clock_phase{config.clock_phase} {
    // Deassert before the bus exists, so that a device is never left selected by the reset state
    // of its chip select pin
    this->unselect_device();

    if (this->handle->State == HAL_SPI_STATE_RESET) {
        config.init_function();
    }

    this->initialized = this->handle->State == HAL_SPI_STATE_READY;
}

bool Spi::select_device() {
    const uint32_t start = Timer::get_counter();
    const uint32_t limit = Timer::to_cycles(1000 * this->timeout);

    while (HAL_SPI_GetState(this->handle) != HAL_SPI_STATE_READY) {
        if (Timer::get_counter() - start > limit) {
            return false;
        }
    }

    if (this->handle->Init.CLKPolarity != this->clock_polarity or this->handle->Init.CLKPhase != this->clock_phase) {
        this->handle->Init.CLKPolarity = this->clock_polarity;
        this->handle->Init.CLKPhase = this->clock_phase;

        if (HAL_SPI_Init(this->handle) != HAL_OK) {
            return false;
        }
    }

    this->cs_gpio.write(false);
    return true;
}

void Spi::unselect_device() {
    this->cs_gpio.write(true);
}

bool Spi::transmit(std::span<const uint8_t> data) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) the HAL takes a mutable pointer it only reads
    auto* buffer = const_cast<uint8_t*>(data.data());

    return HAL_SPI_Transmit(this->handle, buffer, data.size(), this->timeout) == HAL_OK;
}

bool Spi::receive(std::span<uint8_t> data) {
    return HAL_SPI_Receive(this->handle, data.data(), data.size(), this->timeout) == HAL_OK;
}

bool Spi::transmit_receive(std::span<const uint8_t> transmitted, std::span<uint8_t> received) {
    if (received.size() < transmitted.size()) {
        return false;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) the HAL takes a mutable pointer it only reads
    auto* buffer = const_cast<uint8_t*>(transmitted.data());

    return HAL_SPI_TransmitReceive(this->handle, buffer, received.data(), transmitted.size(), this->timeout) == HAL_OK;
}

bool Spi::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
