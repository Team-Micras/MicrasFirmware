/**
 * @file crc.hpp
 *
 * @brief STM32 CRC HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __CRC_HPP__
#define __CRC_HPP__

#include <crc.h>

namespace hal {
/**
 * @brief Class to handle CRC peripheral on STM32 microcontrollers
 */
class Crc {
    public:
        /**
         * @brief CRC configuration struct
         */
        struct Config {
            TIM_HandleTypeDef* handle;
        };

        /**
         * @brief Construct a new Crc object
         *
         * @param config Configuration for the CRC
         */
        Crc(Config& config);

        /**
         * @brief Calculate the CRC value
         *
         * @param data Data to calculate the CRC
         * @param size Size of the buffer
         *
         * @return CRC value
         */
        uint32_t Crc::calculate(uint32_t data[], uint32_t size);

    private:
        /**
         * @brief CRC handle
         */
        CRC_HandleTypeDef* handle;
};
}  // namespace hal

#endif // __CRC_HPP__