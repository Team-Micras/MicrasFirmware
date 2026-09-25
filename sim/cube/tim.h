/**
 * @file
 *
 * @brief Fake STM32CubeMX tim.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_TIM_H
#define MICRAS_SIM_CUBE_TIM_H

#include "main.h"

/**
 * @brief Handle htim1, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim1;

/**
 * @brief Handle htim2, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim2;

/**
 * @brief Handle htim3, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim3;

/**
 * @brief Handle htim4, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim4;

/**
 * @brief Handle htim5, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim5;

/**
 * @brief Handle htim8, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim8;

/**
 * @brief Handle htim12, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim12;

/**
 * @brief Handle htim15, as cube/Src/tim.c defines it.
 */
extern TIM_HandleTypeDef htim15;

/**
 * @brief Configure what MX_TIM1_Init configures in cube/Src/tim.c.
 */
void MX_TIM1_Init();

/**
 * @brief Configure what MX_TIM2_Init configures in cube/Src/tim.c.
 */
void MX_TIM2_Init();

/**
 * @brief Configure what MX_TIM3_Init configures in cube/Src/tim.c.
 */
void MX_TIM3_Init();

/**
 * @brief Configure what MX_TIM4_Init configures in cube/Src/tim.c.
 */
void MX_TIM4_Init();

/**
 * @brief Configure what MX_TIM5_Init configures in cube/Src/tim.c.
 */
void MX_TIM5_Init();

/**
 * @brief Configure what MX_TIM8_Init configures in cube/Src/tim.c.
 */
void MX_TIM8_Init();

/**
 * @brief Configure what MX_TIM12_Init configures in cube/Src/tim.c.
 */
void MX_TIM12_Init();

/**
 * @brief Configure what MX_TIM15_Init configures in cube/Src/tim.c.
 */
void MX_TIM15_Init();

#endif  // MICRAS_SIM_CUBE_TIM_H
