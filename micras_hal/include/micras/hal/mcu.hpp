/**
 * @file
 */

#ifndef MICRAS_HAL_MCU_HPP
#define MICRAS_HAL_MCU_HPP

#include <cstdint>

namespace micras::hal {
/**
 * @brief Microcontroller unit class.
 */
class Mcu {
public:
    /**
     * @brief Deleted constructor for static class.
     */
    Mcu() = delete;

    /**
     * @brief Initialize MCU and some peripherals.
     */
    static void init();
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_MCU_HPP
