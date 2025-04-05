/**
 * @file
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <cstdint>

#include "micras/hal/crc.hpp"
#include "micras/hal/encoder.hpp"
#include "micras/hal/spi.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring rotary sensor data.
 */
class RotarySensor {
public:
#include "micras/proxy/rotary_sensor_reg.hpp"

    /**
     * @brief Rotary sensor configuration struct.
     */
    struct Config {
        hal::Spi::Config     spi;
        hal::Encoder::Config encoder;
        hal::Crc::Config     crc;
        uint32_t             resolution;
        Registers            registers;
    };

    union CommandFrame {
        struct __attribute__((__packed__)) Fields {
            uint8_t  crc         : 8;
            uint16_t address     : 14;
            uint8_t  rw          : 1;
            uint8_t  do_not_care : 1;
        };

        Fields   fields;
        uint32_t raw;
    };

    union DataFrame {
        struct __attribute__((__packed__)) Fields {
            uint8_t  crc     : 8;
            uint16_t data    : 14;
            uint8_t  error   : 1;
            uint8_t  warning : 1;
        };

        Fields   fields;
        uint32_t raw;
    };

    /**
     * @brief Construct a new RotarySensor object.
     *
     * @param config Configuration for the rotary sensor.
     */
    explicit RotarySensor(const Config& config);

    /**
     * @brief Get the rotary sensor position over an axis.
     *
     * @return Current angular position of the sensor in radians.
     */
    float get_position() const;

    /**
     * @brief Read a register to the rotary sensor.
     *
     * @param command_frame Command frame to send trough SPI.
     */
    uint16_t read_register(uint16_t address);

    /**
     * @brief Write a register to the rotary sensor.
     *
     * @param command_frame Command frame to send trough SPI.
     * @param data_frame Data frame to send trough SPI.
     */
    void write_register(CommandFrame& command_frame, DataFrame& data_frame);

private:
    /**
     * @brief SPI for the rotary sensor configuration.
     */
    hal::Spi spi;

    /**
     * @brief Encoder for getting the rotary sensor data.
     */
    hal::Encoder encoder;

    /**
     * @brief CRC for the rotary sensor configuration.
     */
    hal::Crc crc;

    /**
     * @brief Resolution of the rotary sensor.
     */
    uint32_t resolution;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ROTARY_SENSOR_HPP
