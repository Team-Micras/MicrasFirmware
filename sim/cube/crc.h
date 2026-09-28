/**
 * @file
 *
 * @brief Fake STM32CubeMX crc.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_CRC_H
#define MICRAS_SIM_CUBE_CRC_H

#include "main.h"

/**
 * @brief Handle hcrc, as cube/Src/crc.c defines it.
 */
extern CRC_HandleTypeDef hcrc;

/**
 * @brief Configure what MX_CRC_Init configures in cube/Src/crc.c.
 */
void MX_CRC_Init();

#endif  // MICRAS_SIM_CUBE_CRC_H
