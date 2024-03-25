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
            hal::Spi::Config spi;
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
