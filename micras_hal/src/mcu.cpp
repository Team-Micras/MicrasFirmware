/**
 * @file
 */

#include <crc.h>
#include <dma.h>
#include <gpio.h>

#include "micras/hal/mcu.hpp"

extern "C" {
/**
 * @brief Initialize System Clock.
 *
 * @note  Defined by cube.
 */
void SystemClock_Config();
}

namespace micras::hal {
void Mcu::init() {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CRC_Init();
}
}  // namespace micras::hal
