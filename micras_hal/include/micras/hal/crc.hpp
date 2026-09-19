/**
 * @file
 */

#ifndef MICRAS_HAL_CRC_HPP
#define MICRAS_HAL_CRC_HPP

#include <crc.h>
#include <cstdint>
#include <span>

namespace micras::hal {
/**
 * @brief Class to handle the cyclic redundancy check peripheral on STM32 microcontrollers.
 */
class Crc {
public:
    /**
     * @brief CRC configuration struct.
     */
    struct Config {
        CRC_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new Crc object.
     *
     * @param config Configuration for the CRC.
     */
    explicit Crc(const Config& config);

    /**
     * @brief Calculate the CRC value over a buffer of bytes.
     *
     * @note The polynomial, width and initial value come from the peripheral configuration, and the
     * calculation unit is reset to that initial value on every call.
     *
     * @param data Data to calculate the CRC over.
     * @return CRC value.
     */
    uint32_t calculate(std::span<const uint8_t> data);

private:
    /**
     * @brief CRC handle.
     */
    CRC_HandleTypeDef* handle;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_CRC_HPP
