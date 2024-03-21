/**
 * @file imu.cpp
 *
 * @brief Proxy Imu class source
 *
 * @date 03/2024
 */

#include "proxy/imu.hpp"

namespace proxy {
Imu::Imu(Config& imu_config) : spi{imu_config.spi} {
    this->dev_ctx.write_reg =
        [](void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) -> int32_t {
            auto spi = static_cast<hal::Spi*>(handle);

            spi->select_device();
            spi->transmit(&reg, 1);
            spi->transmit((uint8_t*) bufp, len);
            spi->unselect_device();

            return 0;
        };

    this->dev_ctx.read_reg =
        [](void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) -> int32_t {
            auto spi = static_cast<hal::Spi*>(handle);

            spi->select_device();
            spi->transmit(&reg, 1);
            spi->receive(bufp, len);
            spi->unselect_device();

            return 0;
        };

    this->dev_ctx.handle = &imu_config.spi;
}
}  // namespace proxy
