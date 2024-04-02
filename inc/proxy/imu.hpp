/**
 * @file imu.hpp
 *
 * @brief STM32 IMU HAL wrapper
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_IMU_HPP
#define MICRAS_PROXY_IMU_HPP

#include <array>
#include <cstdint>
#include <lsm6dsv_reg.h>
#include <numbers>

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
            lsm6dsv_sflp_data_rate_t        orientation_data_rate;
            lsm6dsv_gy_full_scale_t         gyroscope_scale;
            lsm6dsv_xl_full_scale_t         accelerometer_scale;
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
         * @brief Update the IMU data
         */
        void update_data();

        /**
         * @brief Get the IMU orientation over an axis
         * @todo implement function using sensior fusion
         *
         * @param axis Axis to get the orientation from
         *
         * @return Orientation over the desired axis using quaternions
         */
        float get_orientation(Axis axis) const;

        /**
         * @brief Get the IMU angular velocity over an axis
         *
         * @param axis Axis to get the angular velocity from
         *
         * @return Angular velocity over the desired axis in rad/s
         */
        float get_angular_velocity(Axis axis) const;

        /**
         * @brief Get the IMU linear acceleration over an axis
         *
         * @param axis Axis to get the linear acceleration from
         *
         * @return Linear acceleration over the desired axis in m/s²
         */
        float get_linear_acceleration(Axis axis) const;

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
         * @brief Function to convert raw data to orientation
         *
         * @param quat Quaternion to store the orientation
         * @param sflp Raw data from the IMU
         *
         * @return Orientation in quaternions
         */
        static void convert_orientation(std::array<float, 4>& quat, const uint16_t sflp[3]);

        /**
         * @brief Function to convert half precision float to single precision float
         *
         * @param x Half precision float
         *
         * @return Single precision float
         */
        static float half_to_float(uint16_t x);

        /**
         * @brief Conversion constants
         */
        static constexpr float mdps_to_radps{std::numbers::pi_v<float> / 180000.0F};
        static constexpr float mg_to_mps2{0.00980665F};

        /**
         * @brief SPI for the IMU communication
         */
        hal::Spi spi;

        /**
         * @brief Device context for the IMU library
         */
        stmdev_ctx_t dev_ctx{ };

        /**
         * @brief Current angular velocity on each axis
         */
        std::array<int16_t, 3> angular_velocity{ };

        /**
         * @brief Current linear acceleration on each axis
         */
        std::array<int16_t, 3> linear_acceleration{ };

        /**
         * @brief Current orientation
         */
        std::array<float, 4> orientation{ };

        /**
         * @brief Gyroscope conversion factor
         */
        const float gy_factor;

        /**
         * @brief Accelerometer conversion factor
         */
        const float xl_factor;
};
}  // namespace proxy

#endif // MICRAS_PROXY_IMU_HPP
