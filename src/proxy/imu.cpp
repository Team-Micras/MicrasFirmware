/**
 * @file imu.cpp
 *
 * @brief Proxy Imu class source
 *
 * @date 03/2024
 */

#include "hal/timer.hpp"
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

            reg |= 0x80;
            spi->select_device();
            spi->transmit(&reg, 1);
            spi->receive(bufp, len);
            spi->unselect_device();

            return 0;
        };

    this->dev_ctx.handle = &imu_config.spi;

    hal::Timer::sleep_ms(10);

    lsm6dsv_reset_set(&(this->dev_ctx), LSM6DSV_RESTORE_CTRL_REGS);

    lsm6dsv_reset_t rst;

    do {
        lsm6dsv_reset_get(&(this->dev_ctx), &rst);
    } while (rst != LSM6DSV_READY);

    lsm6dsv_block_data_update_set(&(this->dev_ctx), PROPERTY_ENABLE);

    lsm6dsv_xl_data_rate_set(&(this->dev_ctx), LSM6DSV_ODR_HA02_AT_6400Hz);
    lsm6dsv_gy_data_rate_set(&(this->dev_ctx), LSM6DSV_ODR_HA02_AT_6400Hz);

    lsm6dsv_xl_full_scale_set(&(this->dev_ctx), LSM6DSV_8g);
    lsm6dsv_gy_full_scale_set(&(this->dev_ctx), LSM6DSV_4000dps);

    lsm6dsv_filt_gy_lp1_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&(this->dev_ctx), LSM6DSV_GY_ULTRA_LIGHT);
    lsm6dsv_filt_xl_lp2_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&(this->dev_ctx), LSM6DSV_XL_STRONG);
}

float Imu::get_orientation(Axis axis) {
    return 0.0f;
}

float Imu::get_angular_velocity(Axis axis) {
    lsm6dsv_angular_rate_raw_get(&(this->dev_ctx), this->angular_velocity.data());

    switch (axis) {
        case Axis::X:
            return lsm6dsv_from_fs4000_to_mdps(this->angular_velocity[0]);

        case Axis::Y:
            return lsm6dsv_from_fs4000_to_mdps(this->angular_velocity[1]);

        case Axis::Z:
            return lsm6dsv_from_fs4000_to_mdps(this->angular_velocity[2]);

        default:
            return 0.0;
    }
}

float Imu::get_linear_acceleration(Axis axis) {
    lsm6dsv_acceleration_raw_get(&(this->dev_ctx), this->linear_acceleration.data());

    switch (axis) {
        case Axis::X:
            return lsm6dsv_from_fs8_to_mg(this->linear_acceleration[0]);

        case Axis::Y:
            return lsm6dsv_from_fs8_to_mg(this->linear_acceleration[1]);

        case Axis::Z:
            return lsm6dsv_from_fs8_to_mg(this->linear_acceleration[2]);

        default:
            return 0.0;
    }
}
}  // namespace proxy
