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
Imu::Imu(Config& config) :
    convert_ang_vel{config.convert_ang_vel}, convert_lin_acc{config.convert_lin_acc}, spi{config.spi} {
    this->dev_ctx.read_reg = platform_read;
    this->dev_ctx.write_reg = platform_write;
    this->dev_ctx.handle = &config.spi;

    hal::Timer::sleep_ms(10);

    lsm6dsv_reset_set(&(this->dev_ctx), LSM6DSV_RESTORE_CTRL_REGS);

    lsm6dsv_reset_t rst;

    do {
        lsm6dsv_reset_get(&(this->dev_ctx), &rst);
    } while (rst != LSM6DSV_READY);

    lsm6dsv_block_data_update_set(&(this->dev_ctx), PROPERTY_ENABLE);

    lsm6dsv_gy_data_rate_set(&(this->dev_ctx), config.gyroscope_data_rate);
    lsm6dsv_xl_data_rate_set(&(this->dev_ctx), config.accelerometer_data_rate);

    lsm6dsv_gy_full_scale_set(&(this->dev_ctx), config.gyroscope_full_scale);
    lsm6dsv_xl_full_scale_set(&(this->dev_ctx), config.accelerometer_full_scale);

    lsm6dsv_filt_settling_mask_t filt_settling_mask;
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv_filt_settling_mask_set(&dev_ctx, filt_settling_mask);

    lsm6dsv_filt_gy_lp1_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&(this->dev_ctx), config.gyroscope_filter);
    lsm6dsv_filt_xl_lp2_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&(this->dev_ctx), config.accelerometer_filter);
}

float Imu::get_orientation(Axis  /*axis*/) {
    return 0.0f;
}

float Imu::get_angular_velocity(Axis axis) {
    lsm6dsv_angular_rate_raw_get(&(this->dev_ctx), this->angular_velocity.data());
    float ang_vel;

    switch (axis) {
        case Axis::X:
            ang_vel = convert_ang_vel(this->angular_velocity[0]);
            break;

        case Axis::Y:
            ang_vel = convert_ang_vel(this->angular_velocity[1]);
            break;

        case Axis::Z:
            ang_vel = convert_ang_vel(this->angular_velocity[2]);
            break;

        default:
            return 0.0f;
    }

    return ang_vel * mdps_to_radps;
}

float Imu::get_linear_acceleration(Axis axis) {
    lsm6dsv_acceleration_raw_get(&(this->dev_ctx), this->linear_acceleration.data());
    float lin_acc;

    switch (axis) {
        case Axis::X:
            lin_acc = this->convert_lin_acc(this->linear_acceleration[0]);
            break;

        case Axis::Y:
            lin_acc = this->convert_lin_acc(this->linear_acceleration[1]);
            break;

        case Axis::Z:
            lin_acc = this->convert_lin_acc(this->linear_acceleration[2]);
            break;

        default:
            return 0.0f;
    }

    return lin_acc * mg_to_mps2;
}

int32_t Imu::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
    auto spi = static_cast<hal::Spi*>(handle);

    while (not spi->select_device()) {
        continue;
    }

    reg |= 0x80;
    spi->transmit(&reg, 1);
    spi->receive(bufp, len);
    spi->unselect_device();

    return 0;
}

int32_t Imu::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) {
    auto spi = static_cast<hal::Spi*>(handle);

    while (not spi->select_device()) {
        continue;
    }

    spi->transmit(&reg, 1);
    spi->transmit((uint8_t*) bufp, len);
    spi->unselect_device();

    return 0;
};
}  // namespace proxy
