/**
 * @file
 *
 * @brief Fake STM32CubeMX gpio.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_GPIO_H
#define MICRAS_SIM_CUBE_GPIO_H

#include "main.h"

/**
 * @brief Configure what MX_GPIO_Init configures in cube/Src/gpio.c.
 */
void MX_GPIO_Init();

#endif  // MICRAS_SIM_CUBE_GPIO_H
