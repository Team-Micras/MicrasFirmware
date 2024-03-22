/**
 * @file mcu.cpp
 *
 * @brief MCU related
 */

#include <gpio.h>
#include <main.h>

#include "hal/mcu.hpp"

namespace hal {
void Mcu::init(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
}
}
