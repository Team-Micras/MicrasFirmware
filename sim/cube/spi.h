/**
 * @file
 *
 * @brief Fake STM32CubeMX spi.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_SPI_H
#define MICRAS_SIM_CUBE_SPI_H

#include "main.h"

/**
 * @brief Handle hspi3, as cube/Src/spi.c defines it.
 */
extern SPI_HandleTypeDef hspi3;

/**
 * @brief Configure what MX_SPI3_Init configures in cube/Src/spi.c.
 */
void MX_SPI3_Init();

#endif  // MICRAS_SIM_CUBE_SPI_H
