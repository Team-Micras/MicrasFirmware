/**
 * @file
 */

#include <array>
#include <cmath>
#include <cstdint>

#include "lsm6dsv_reg.h"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
Imu::Imu(const Config& config) :
    spi{config.spi},
    gy_factor{
        mdps_to_radps * 4.375F *
        (1 << (config.gyroscope_scale == LSM6DSV_4000dps ? 5 : static_cast<uint8_t>(config.gyroscope_scale)))
    },
    xl_factor{mg_to_mps2 * (0.061F * (1 << static_cast<uint8_t>(config.accelerometer_scale)))} {
    this->dev_ctx.read_reg = platform_read;
    this->dev_ctx.write_reg = platform_write;
    this->dev_ctx.mdelay = proxy::Stopwatch::sleep_ms;
    this->dev_ctx.handle = &this->spi;

    proxy::Stopwatch::sleep_ms(10);

    if (not this->check_whoami()) {
        return;
    }

    lsm6dsv_sw_por(&(this->dev_ctx));
    lsm6dsv_block_data_update_set(&(this->dev_ctx), PROPERTY_ENABLE);

    lsm6dsv_gy_data_rate_set(&(this->dev_ctx), config.gyroscope_data_rate);
    lsm6dsv_xl_data_rate_set(&(this->dev_ctx), config.accelerometer_data_rate);

    lsm6dsv_gy_full_scale_set(&(this->dev_ctx), config.gyroscope_scale);
    lsm6dsv_xl_full_scale_set(&(this->dev_ctx), config.accelerometer_scale);

    lsm6dsv_filt_settling_mask_set(&dev_ctx, {.drdy = 1, .ois_drdy = 0, .irq_xl = 0, .irq_g = 0});

    lsm6dsv_filt_gy_lp1_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&(this->dev_ctx), config.gyroscope_filter);
    lsm6dsv_filt_xl_lp2_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&(this->dev_ctx), config.accelerometer_filter);
    this->initialized = true;
}

bool Imu::check_whoami() {
    uint8_t whoami = 0;

    lsm6dsv_device_id_get(&(this->dev_ctx), &whoami);

    return whoami == LSM6DSV_ID;
}

void Imu::update() {
    std::array<int16_t, 3> raw_data{};
    lsm6dsv_all_sources_t  all_sources;
    lsm6dsv_all_sources_get(&dev_ctx, &all_sources);

    if (all_sources.drdy_xl) {
        lsm6dsv_acceleration_raw_get(&dev_ctx, raw_data.data());
        std::get<0>(this->linear_acceleration) = std::get<0>(raw_data) * xl_factor;
        std::get<1>(this->linear_acceleration) = std::get<1>(raw_data) * xl_factor;
        std::get<2>(this->linear_acceleration) = std::get<2>(raw_data) * xl_factor;
    }

    if (all_sources.drdy_gy) {
        lsm6dsv_angular_rate_raw_get(&dev_ctx, raw_data.data());
        std::get<0>(this->angular_velocity) = std::get<0>(raw_data) * gy_factor;
        std::get<1>(this->angular_velocity) = std::get<1>(raw_data) * gy_factor;
        std::get<2>(this->angular_velocity) = std::get<2>(raw_data) * gy_factor;
    }

    if (not this->calibrated) {
        this->calibration_filter.update(std::get<2>(this->angular_velocity));
    }
}

float Imu::get_angular_velocity(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return std::get<0>(this->angular_velocity);

        case Axis::Y:
            return std::get<1>(this->angular_velocity);

        case Axis::Z:
            return std::get<2>(this->angular_velocity) - this->calibration_filter.get_last();

        default:
            return 0.0F;
    }
}

float Imu::get_linear_acceleration(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return std::get<0>(this->linear_acceleration);

        case Axis::Y:
            return std::get<1>(this->linear_acceleration);

        case Axis::Z:
            return std::get<2>(this->linear_acceleration);

        default:
            return 0.0F;
    }
}

int32_t Imu::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
    auto* spi = static_cast<hal::Spi*>(handle);

    while (not spi->select_device()) { }

    reg |= 0x80;
    spi->transmit({&reg, 1});
    spi->receive({bufp, len});
    spi->unselect_device();

    return 0;
}

int32_t Imu::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) {
    auto* spi = static_cast<hal::Spi*>(handle);

    while (not spi->select_device()) { }

    spi->transmit({&reg, 1});
    spi->transmit({const_cast<uint8_t*>(bufp), len});  // NOLINT(cppcoreguidelines-pro-type-const-cast)
    spi->unselect_device();

    return 0;
}

void Imu::calibrate() {
    this->calibrated = true;
}

bool Imu::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::proxy
