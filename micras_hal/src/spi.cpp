/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "micras/hal/spi.hpp"
#include "micras/hal/timer.hpp"

extern "C" {
/**
 * @brief Callback of the vendor HAL for the end of a transfer in both directions.
 *
 * @param hspi Handle of the bus.
 */
// NOLINTNEXTLINE(readability-identifier-naming) the name is fixed by the vendor HAL
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    micras::hal::Spi::on_transfer_end(hspi, true);
}

/**
 * @brief Callback of the vendor HAL for a transfer that ended in a bus or DMA error.
 *
 * @param hspi Handle of the bus.
 */
// NOLINTNEXTLINE(readability-identifier-naming) the name is fixed by the vendor HAL
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    micras::hal::Spi::on_transfer_end(hspi, false);
}
}

namespace micras::hal {
std::array<Spi*, Spi::max_transfers> Spi::transferring{};

Spi::Spi(const Config& config) :
    handle{config.handle},
    cs_gpio{config.cs_gpio},
    timeout{config.timeout},
    clock_polarity{config.clock_polarity},
    clock_phase{config.clock_phase} {
    this->unselect_device();

    if (this->handle->State == HAL_SPI_STATE_RESET) {
        config.init_function();
    }

    this->initialized = this->handle->State == HAL_SPI_STATE_READY;
}

Spi::~Spi() {
    if (this->transfer == Transfer::RUNNING) {
        HAL_SPI_Abort(this->handle);
        this->unselect_device();
    }

    std::ranges::replace(transferring, this, static_cast<Spi*>(nullptr));
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

bool Spi::start_transfer(std::span<const uint8_t> transmitted, std::span<uint8_t> received) {
    auto* const slot = std::ranges::find(transferring, nullptr);

    if (received.size() < transmitted.size() or slot == transferring.end() or not this->select_device()) {
        this->transfer = Transfer::FAILED;
        return false;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) the HAL takes a mutable pointer it only reads
    auto* buffer = const_cast<uint8_t*>(transmitted.data());

    *slot = this;
    this->transfer = Transfer::RUNNING;

    if (HAL_SPI_TransmitReceive_DMA(this->handle, buffer, received.data(), transmitted.size()) != HAL_OK) {
        *slot = nullptr;
        this->transfer = Transfer::FAILED;
        this->unselect_device();
        return false;
    }

    return true;
}

Spi::Transfer Spi::get_transfer() const {
    return this->transfer;
}

void Spi::on_transfer_end(const SPI_HandleTypeDef* handle, bool succeeded) {
    for (Spi*& device : transferring) {
        if (device != nullptr and device->handle == handle) {
            device->unselect_device();
            device->transfer = succeeded ? Transfer::COMPLETE : Transfer::FAILED;
            device = nullptr;
            return;
        }
    }
}

bool Spi::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::hal
