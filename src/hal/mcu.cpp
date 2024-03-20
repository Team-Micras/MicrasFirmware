/**
 * @file mcu.cpp
 *
 * @brief MCU related
 */

#include <stdint.h>

#include "hal/mcu.hpp"

#include "gpio.h"
#include "main.h"

/*****************************************
 * Public Function Body Definitions
 *****************************************/

namespace hal {
void mcu::init(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
}

void mcu::sleep(uint32_t ms) {
    HAL_Delay(ms);
}

void mcu::reset_timer(uint32_t& ticks) {
    ticks = HAL_GetTick();
}

uint32_t mcu::get_timer_ms(uint32_t ticks) {
    return HAL_GetTick() - ticks;
}
}
