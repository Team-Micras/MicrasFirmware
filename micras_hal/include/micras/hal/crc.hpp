/**
 * @file
 */

#ifndef MICRAS_HAL_CRC_HPP
#define MICRAS_HAL_CRC_HPP

#include <crc.h>

namespace micras::hal {
/**
 * @brief Class to handle the cyclic redundancy check peripheral on STM32 microcontrollers
 */
class Crc {
public:
    /**
     * @brief CRC configuration struct
     */
    struct Config {
        CRC_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new Crc object
     *
     * @param config Configuration for the CRC
     */
    explicit Crc(const Config& config);

    /**
     * @brief Calculate the CRC value
     *
     * @param data Data to calculate the CRC
     * @param size Size of the buffer
     *
     * @return uint32_t CRC value
     */
    uint32_t calculate(uint32_t data[], uint32_t size);  // NOLINT(*-avoid-c-arrays)

private:
    /**
     * @brief CRC handle
     */
    CRC_HandleTypeDef* handle;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_CRC_HPP
