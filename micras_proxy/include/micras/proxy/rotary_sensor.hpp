/**
 * @file
 */

#ifndef MICRAS_PROXY_ROTARY_SENSOR_HPP
#define MICRAS_PROXY_ROTARY_SENSOR_HPP

#include <array>
#include <cstdint>
#include <optional>

#include "micras/hal/crc.hpp"
#include "micras/hal/encoder.hpp"
#include "micras/hal/spi.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring rotary sensor data.
 *
 * @note The angle is decoded from the quadrature output of the sensor by a hardware timer, while
 * SPI is used only to configure the sensor at construction and to read back that configuration.
 */
class RotarySensor {
public:
#include "micras/proxy/rotary_sensor_reg.hpp"

    /**
     * @brief Rotary sensor configuration struct.
     *
     * @note There is no resolution field: the pulses per revolution are whatever the ABIRES field
     * of the sensor says after configuration, so the scale factor is read back from the hardware
     * instead of being asserted here. Two numbers that have to agree are one number too many.
     */
    struct Config {
        hal::Spi::Config     spi;
        hal::Encoder::Config encoder;
        hal::Crc::Config     crc;
        Registers            registers;
    };

    /**
     * @brief Frame sent to the sensor to address one of its registers.
     */
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

    /**
     * @brief Frame carrying a register value in either direction.
     */
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
     * @brief Get the number of counts the quadrature decoder produces per revolution.
     *
     * @return Counts per revolution, as read back from the sensor.
     */
    uint32_t get_resolution() const;

    /**
     * @brief Check if the sensor was configured and its configuration read back successfully.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

    /**
     * @brief Read a register from the rotary sensor.
     *
     * @param address Address of the register.
     * @return The register value, or nothing if a transfer failed or a frame was rejected.
     */
    std::optional<uint16_t> read_register(uint16_t address);

    /**
     * @brief Write a register to the rotary sensor.
     *
     * @param address Address of the register.
     * @param data Value to write.
     * @return True if both frames were transferred, false otherwise.
     */
    bool write_register(uint16_t address, uint16_t data);

private:
    /**
     * @brief Number of bytes of a sensor SPI frame.
     */
    static constexpr uint8_t frame_size{3};

    /**
     * @brief Number of edges the quadrature decoder counts per pulse of a single channel.
     */
    static constexpr uint32_t edges_per_pulse{4};

    /**
     * @brief Pulses per revolution selected by each value of the ABIRES field, zero when reserved.
     */
    static constexpr std::array<uint16_t, 8> pulses_per_revolution{{1024, 512, 256, 2048, 4096, 0, 0, 0}};

    /**
     * @brief Serialize a frame into the wire order of the sensor, filling in its CRC.
     *
     * @param raw The 24 bit frame, without a valid CRC.
     * @return The frame as three bytes, most significant first, with the CRC in the last one.
     */
    std::array<uint8_t, frame_size> serialize(uint32_t raw);

    /**
     * @brief Exchange one frame with the sensor.
     *
     * @param frame The frame to send.
     * @return The frame received while sending, or nothing if the transfer failed.
     */
    std::optional<uint32_t> exchange_frame(const std::array<uint8_t, frame_size>& frame);

    /**
     * @brief Read back the ABIRES field and derive the counts per revolution from it.
     *
     * @return Counts per revolution, or nothing if the read failed or reported a reserved value.
     */
    std::optional<uint32_t> read_resolution();

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
     * @brief Counts per revolution of the quadrature decoder, read back from the sensor.
     */
    uint32_t resolution{};

    /**
     * @brief Flag to check if the sensor was configured successfully.
     */
    bool initialized{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ROTARY_SENSOR_HPP
