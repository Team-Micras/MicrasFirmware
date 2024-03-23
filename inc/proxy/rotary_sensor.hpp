/**
 * @file rotary_sensor.hpp
 *
 * @brief STM32 rotary sensor HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __ROTARY_SENSOR_HPP__
#define __ROTARY_SENSOR_HPP__

#include <cstdint>

#include "hal/encoder.hpp"
#include "hal/spi.hpp"

namespace proxy {
/**
 * @brief Class to handle rotary sensor peripheral on STM32 microcontrollers
 */
class RotarySensor {
    public:
        /**
         * @brief rotary sensor configuration struct
         */
        struct Config {
            hal::Spi::Config     spi;
            hal::Encoder::Config encoder;
            uint32_t             resolution;
        };

        /**
         * @brief Construct a new RotarySensor object
         *
         * @param rotary_sensor_config Configuration for the rotary sensor
         */
        RotarySensor(Config& rotary_sensor_config);

        /**
         * @brief Get the rotary sensor position over an axis
         *
         * @return Current angular position of the sensor in radians
         */
        float get_position();

    private:
        /**
         * @brief SPI for the rotary sensor configuration
         */
        hal::Spi spi;

        /**
         * @brief Encoder for getting the rotary sensor data
         */
        hal::Encoder encoder;

        /**
         * @brief Resolution of the rotary sensor
         */
        uint32_t resolution;
};
}  // namespace proxy

#endif // __ROTARY_SENSOR_HPP__
