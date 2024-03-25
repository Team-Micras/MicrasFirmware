/**
 * @file imu.hpp
 *
 * @brief STM32 IMU HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __IMU_HPP__
#define __IMU_HPP__

#include <array>
#include <cstdint>
#include <lsm6dsv_reg.h>

#include "hal/spi.hpp"

namespace proxy {
/**
 * @brief Class to handle IMU peripheral on STM32 microcontrollers
 */
class Imu {
    public:
        /**
         * @brief IMU configuration struct
         */
        struct Config {
            hal::Spi::Config                spi;
            lsm6dsv_data_rate_t             gyroscope_data_rate;
            lsm6dsv_data_rate_t             accelerometer_data_rate;
            lsm6dsv_gy_full_scale_t         gyroscope_full_scale;
            lsm6dsv_xl_full_scale_t         accelerometer_full_scale;
            float                           (* convert_ang_vel)(int16_t);
            float                           (* convert_lin_acc)(int16_t);
            lsm6dsv_filt_gy_lp1_bandwidth_t gyroscope_filter;
            lsm6dsv_filt_xl_lp2_bandwidth_t accelerometer_filter;
        };

        /**
         * @brief Construct a new Imu object
         *
         * @param config Configuration for the IMU
         */
        Imu(Config& config);

        enum Axis {
            W,
            X,
            Y,
            Z
        };

        /**
         * @brief Get the IMU orientation over an axis
         * @todo implement function using sensior fusion
         *
         * @param axis Axis to get the orientation from
         *
         * @return Orientation over the desired axis using quaternions
         */
        float get_orientation(Axis axis);

        /**
         * @brief Get the IMU angular velocity over an axis
         *
         * @param axis Axis to get the angular velocity from
         *
         * @return Angular velocity over the desired axis in rad/s
         */
        float get_angular_velocity(Axis axis);

        /**
         * @brief Get the IMU linear acceleration over an axis
         *
         * @param axis Axis to get the linear acceleration from
         *
         * @return Linear acceleration over the desired axis in m/s²
         */
        float get_linear_acceleration(Axis axis);

    private:
        /**
         * @brief Read data from the IMU
         *
         * @param handle Pointer to a SPI object
         * @param reg Register to read from
         * @param bufp Buffer to read
         * @param len Length of the buffer
         *
         * @return 0 if the operation was successful, -1 otherwise
         */
        static int32_t platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);

        /**
         * @brief Write data to the IMU
         *
         * @param handle Pointer to a SPI object
         * @param reg Register to write to
         * @param bufp Buffer to write
         * @param len Length of the buffer
         *
         * @return 0 if the operation was successful, -1 otherwise
         */
        static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);

        /**
         * @brief Function to convert raw data to angular velocity
         *
         * @param raw_data Raw data from the IMU
         *
         * @return Angular velocity in md/s
         */
        float (* convert_ang_vel)(int16_t);

        /**
         * @brief Function to convert raw data to linear acceleration
         *
         * @param raw_data Raw data from the IMU
         *
         * @return Linear acceleration in mg
         */
        float (* convert_lin_acc)(int16_t);

        /**
         * @brief SPI for the IMU communication
         */
        hal::Spi spi;

        /**
         * @brief Device context for the IMU library
         */
        stmdev_ctx_t dev_ctx;

        /**
         * @brief Current angular velocity on each axis
         */
        std::array<int16_t, 3> angular_velocity;

        /**
         * @brief Current linear acceleration on each axis
         */
        std::array<int16_t, 3> linear_acceleration;
};
}  // namespace proxy

#endif // __IMU_HPP__
