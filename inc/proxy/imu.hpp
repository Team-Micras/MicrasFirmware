/**
 * @file imu.hpp
 *
 * @brief STM32 IMU HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __IMU_HPP__
#define __IMU_HPP__

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
         * @param imu_config Configuration for the IMU
         */
        Imu(Config& imu_config);

    private:
        /**
         * @brief SPI for the IMU communication
         */
        hal::Spi spi;

        /**
         * @brief Device context for the IMU library
         */
        stmdev_ctx_t dev_ctx;
};
}  // namespace proxy

#endif // __IMU_HPP__
