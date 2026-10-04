/**
 * @file
 *
 * @brief Fake STM32CubeMX fmac.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_FMAC_H
#define MICRAS_SIM_CUBE_FMAC_H

#include "main.h"

/**
 * @brief Handle hfmac, as cube/Src/fmac.c defines it.
 */
extern FMAC_HandleTypeDef hfmac;

/**
 * @brief Configure what MX_FMAC_Init configures in cube/Src/fmac.c.
 */
void MX_FMAC_Init();

#endif  // MICRAS_SIM_CUBE_FMAC_H
