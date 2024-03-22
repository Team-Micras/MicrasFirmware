/**
 * @file mcu.hpp
 *
 * @brief MCU related
 *
 * @date 03/2024
 */

#ifndef __MCU_HPP__
#define __MCU_HPP__

#include <cstdint>

extern "C"
{
/**
 * @brief Initializes System Clock
 * @note  Defined by cube
 */
void SystemClock_Config(void);
}

namespace hal {
/**
 * @brief Microcontroller unit class
 */
class Mcu {
    public:
        /**
         * @brief Initializes MCU and some peripherals
         */
        static void init(void);
};
};  // namespace hal
#endif // __MCU_HPP__
