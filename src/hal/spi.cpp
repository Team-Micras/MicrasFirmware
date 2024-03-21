/**
 * @file spi.cpp
 *
 * @brief Proxy SPI Switch class source
 *
 * @date 03/2024
 */

#include "hal/mcu.hpp"
#include "hal/spi.hpp"

namespace hal {
template <uint8_t num_of_devices>
Spi<num_of_devices>::Spi(const Config& spi_config) : handle{spi_config.handle} {
    for (uint8_t i = 0; i < num_of_devices; i++) {
        this->gpio_array[i] = spi_config.gpio_array[i];
    }
}

template <uint8_t num_of_devices>
void Spi<num_of_devices>::select_device(uint8_t device) {
    this->gpio_array[device].write(false);
}

template <uint8_t num_of_devices>
void Spi<num_of_devices>::unselect_device(uint8_t device) {
    this->gpio_array[device].write(true);
}

template <uint8_t num_of_devices>
void Spi<num_of_devices>::transmit(uint8_t data[], uint32_t size) {
    HAL_SPI_Transmit_DMA(this->handle, data, size);
}

template <uint8_t num_of_devices>
void Spi<num_of_devices>::receive(uint8_t data[], uint32_t size) {
    HAL_SPI_Receive_DMA(this->handle, data, size);
}
}  // namespace proxy
