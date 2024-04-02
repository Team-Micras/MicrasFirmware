/**
 * @file mcu.cpp
 *
 * @brief MCU related
 */

#include <crc.h>
#include <dma.h>
#include <gpio.h>
#include <main.h>

#include "hal/mcu.hpp"

extern "C" {
/**
 * @brief Initializes System Clock
 * @note  Defined by cube
 */
void SystemClock_Config();
}

namespace hal {
void Mcu::init() {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CRC_Init();
}
}  // namespace hal
