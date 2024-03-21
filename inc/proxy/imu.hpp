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
            hal::Spi<3>::Config spi;  // nossa nossa nossa nossa, que tristeza T.T
        };

        /**
         * @brief Construct a new Imu object
         *
         * @param imu_config Configuration for the IMU
         */
        Imu(const Config& imu_config);

    private:
        /**
         * @brief SPI for the IMU communication
         */
        hal::Spi<3> spi;  // nossa nossa nossa nossa, que tristeza T.T
};
}  // namespace proxy

#endif // __IMU_HPP__
