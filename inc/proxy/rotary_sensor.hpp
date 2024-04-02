/**
 * @file rotary_sensor.hpp
 *
 * @brief STM32 rotary sensor HAL wrapper
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <cstdint>

#include "hal/crc.hpp"
#include "hal/encoder.hpp"
#include "hal/spi.hpp"

namespace proxy {
/**
 * @brief Class to handle rotary sensor peripheral on STM32 microcontrollers
 */
class RotarySensor {
    public:
    #include "proxy/rotary_sensor_reg.hpp"

        /**
         * @brief Rotary sensor configuration struct
         */
        struct Config {
            hal::Spi::Config     spi;
            hal::Encoder::Config encoder;
            hal::Crc::Config     crc;
            uint32_t             resolution;
            Registers            registers;
        };

        /**
         * @brief Construct a new RotarySensor object
         *
         * @param config Configuration for the rotary sensor
         */
        RotarySensor(const Config& config);

        /**
         * @brief Get the rotary sensor position over an axis
         *
         * @return Current angular position of the sensor in radians
         */
        float get_position() const;

    private:
        union CommandFrame {
            struct Fields {
                uint8_t  do_not_care : 1;
                uint8_t  rw : 1;
                uint16_t address : 14;
                uint8_t  crc : 8;
            };

            Fields   fields;
            uint32_t raw;
        };

        union DataFrame {
            struct Fields {
                uint8_t  warning : 1;
                uint8_t  error : 1;
                uint16_t data : 14;
                uint8_t  crc : 8;
            };

            Fields   fields;
            uint32_t raw;
        };

        /**
         * @brief Write a register to the rotary sensor
         *
         * @param command_frame Command frame to send trough SPI
         * @param data_frame Data frame to send trough SPI
         */
        void write_register(CommandFrame& command_frame, DataFrame& data_frame);

        /**
         * @brief SPI for the rotary sensor configuration
         */
        hal::Spi spi;

        /**
         * @brief Encoder for getting the rotary sensor data
         */
        hal::Encoder encoder;

        /**
         * @brief CRC for the rotary sensor configuration
         */
        hal::Crc crc;

        /**
         * @brief Resolution of the rotary sensor
         */
        uint32_t resolution;
};
}  // namespace proxy

#endif // MICRAS_PROXY_ROTARY_SENSOR_HPP
