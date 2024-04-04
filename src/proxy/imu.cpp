/**
 * @file imu.cpp
 *
 * @brief Proxy Imu class source
 *
 * @date 03/2024
 */

#include <cmath>

#include "hal/timer.hpp"
#include "proxy/imu.hpp"

namespace proxy {
Imu::Imu(const Config& config) :
    spi{config.spi},
    gy_factor{
        mdps_to_radps * 4.375F *
        (1 << (config.gyroscope_scale == LSM6DSV_4000dps ? 5 : static_cast<uint8_t>(config.gyroscope_scale)))},
    xl_factor{mg_to_mps2 * (0.61F * (1 << static_cast<uint8_t>(config.accelerometer_scale)))} {
    this->dev_ctx.read_reg = platform_read;
    this->dev_ctx.write_reg = platform_write;
    this->dev_ctx.handle = &this->spi;

    hal::Timer::sleep_ms(10);

    lsm6dsv_reset_set(&(this->dev_ctx), LSM6DSV_RESTORE_CTRL_REGS);

    lsm6dsv_reset_t rst{};

    do {
        lsm6dsv_reset_get(&(this->dev_ctx), &rst);
    } while (rst != LSM6DSV_READY);

    lsm6dsv_block_data_update_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_fifo_watermark_set(&(this->dev_ctx), 3);
    lsm6dsv_fifo_stop_on_wtm_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_fifo_mode_set(&(this->dev_ctx), LSM6DSV_STREAM_MODE);

    lsm6dsv_gy_data_rate_set(&(this->dev_ctx), config.gyroscope_data_rate);
    lsm6dsv_xl_data_rate_set(&(this->dev_ctx), config.accelerometer_data_rate);
    lsm6dsv_sflp_data_rate_set(&(this->dev_ctx), config.orientation_data_rate);

    lsm6dsv_fifo_gy_batch_set(&(this->dev_ctx), static_cast<lsm6dsv_fifo_gy_batch_t>(config.gyroscope_data_rate));
    lsm6dsv_fifo_xl_batch_set(&(this->dev_ctx), static_cast<lsm6dsv_fifo_xl_batch_t>(config.accelerometer_data_rate));
    lsm6dsv_fifo_sflp_batch_set(&dev_ctx, {.game_rotation = 1, .gravity = 0, .gbias = 0});

    lsm6dsv_gy_full_scale_set(&(this->dev_ctx), config.gyroscope_scale);
    lsm6dsv_xl_full_scale_set(&(this->dev_ctx), config.accelerometer_scale);

    lsm6dsv_filt_settling_mask_set(&dev_ctx, {.drdy = 1, .ois_drdy = 0, .irq_xl = 1, .irq_g = 1});

    lsm6dsv_filt_gy_lp1_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&(this->dev_ctx), config.gyroscope_filter);
    lsm6dsv_filt_xl_lp2_set(&(this->dev_ctx), PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&(this->dev_ctx), config.accelerometer_filter);

    lsm6dsv_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
}

void Imu::update_data() {
    lsm6dsv_fifo_status_t fifo_status;
    lsm6dsv_fifo_status_get(&(this->dev_ctx), &fifo_status);

    if (not fifo_status.fifo_th) {
        return;
    }

    uint16_t samples = fifo_status.fifo_level;

    while ((samples--) > 0) {
        lsm6dsv_fifo_out_raw_t f_data;
        lsm6dsv_fifo_out_raw_get(&(this->dev_ctx), &f_data);
        auto* axis = reinterpret_cast<int16_t*>(f_data.data);

        switch (f_data.tag) {
            case lsm6dsv_fifo_out_raw_t::LSM6DSV_GY_NC_TAG:
                this->angular_velocity[0] = axis[0] * gy_factor;
                this->angular_velocity[1] = axis[1] * gy_factor;
                this->angular_velocity[2] = axis[2] * gy_factor;
                break;

            case lsm6dsv_fifo_out_raw_t::LSM6DSV_XL_NC_TAG:
                this->linear_acceleration[0] = axis[0] * xl_factor;
                this->linear_acceleration[1] = axis[1] * xl_factor;
                this->linear_acceleration[2] = axis[2] * xl_factor;
                break;

            case lsm6dsv_fifo_out_raw_t::LSM6DSV_SFLP_GAME_ROTATION_VECTOR_TAG:
                convert_orientation(this->orientation, reinterpret_cast<uint16_t*>(axis));
                break;

            default:
                break;
        }
    }
}

float Imu::get_orientation(Axis axis) const {
    switch (axis) {
        case Axis::W:
            return this->orientation[3];

        case Axis::X:
            return this->orientation[0];

        case Axis::Y:
            return this->orientation[1];

        case Axis::Z:
            return this->orientation[2];

        default:
            return 0.0F;
    }
}

float Imu::get_angular_velocity(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->angular_velocity[0];

        case Axis::Y:
            return this->angular_velocity[1];

        case Axis::Z:
            return this->angular_velocity[2];

        default:
            return 0.0F;
    }
}

float Imu::get_linear_acceleration(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return this->linear_acceleration[0];

        case Axis::Y:
            return this->linear_acceleration[1];

        case Axis::Z:
            return this->linear_acceleration[2];

        default:
            return 0.0F;
    }
}

int32_t Imu::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
    auto* spi = static_cast<hal::Spi*>(handle);

    while (not spi->select_device()) { }

    reg |= 0x80;
    spi->transmit(&reg, 1);
    spi->receive(bufp, len);
    spi->unselect_device();

    return 0;
}

int32_t Imu::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) {
    auto* spi = static_cast<hal::Spi*>(handle);

    while (not spi->select_device()) { }

    spi->transmit(&reg, 1);
    spi->transmit(const_cast<uint8_t*>(bufp), len);  // NOLINT(cppcoreguidelines-pro-type-const-cast)
    spi->unselect_device();

    return 0;
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Imu::convert_orientation(std::array<float, 4>& quat, const uint16_t sflp[3]) {
    float sumsq = 0;

    quat[0] = half_to_float(sflp[0]);
    quat[1] = half_to_float(sflp[1]);
    quat[2] = half_to_float(sflp[2]);

    for (uint8_t i = 0; i < 3; i++) {
        sumsq += quat[i] * quat[i];  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    if (sumsq > 1.0F) {
        float norm = std::sqrt(sumsq);
        quat[0] /= norm;
        quat[1] /= norm;
        quat[2] /= norm;
        sumsq = 1.0F;
    }

    quat[3] = std::sqrt(1.0F - sumsq);
}

// NOLINTBEGIN(readability-identifier-length, readability-implicit-bool-conversion)
float Imu::half_to_float(uint16_t x) {
    static constexpr auto as_float = [](uint32_t x) -> float {
        void* aux = &x;
        return *reinterpret_cast<float*>(aux);
    };
    static constexpr auto as_uint = [](float x) -> uint32_t {
        void* aux = &x;
        return *reinterpret_cast<uint32_t*>(aux);
    };

    const uint32_t e = (x & 0x7C00) >> 10;
    const uint32_t m = (x & 0x03FF) << 13;
    const uint32_t v = as_uint(static_cast<float>(m)) >> 23;
    return as_float(
        (x & 0x8000) << 16 | (e != 0) * ((e + 112) << 23 | m) |
        ((e == 0) & (m != 0)) * ((v - 37) << 23 | ((m << (150 - v)) & 0x007FE000))
    );
}

// NOLINTEND(readability-identifier-length, readability-implicit-bool-conversion)
}  // namespace proxy
