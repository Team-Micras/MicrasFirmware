/**
 * @file
 *
 * @brief Fake STM32CubeMX adc.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_ADC_H
#define MICRAS_SIM_CUBE_ADC_H

#include "main.h"

/**
 * @brief Handle hadc1, as cube/Src/adc.c defines it.
 */
extern ADC_HandleTypeDef hadc1;

/**
 * @brief Handle hadc2, as cube/Src/adc.c defines it.
 */
extern ADC_HandleTypeDef hadc2;

/**
 * @brief Handle hadc3, as cube/Src/adc.c defines it.
 */
extern ADC_HandleTypeDef hadc3;

/**
 * @brief Configure what MX_ADC1_Init configures in cube/Src/adc.c.
 */
void MX_ADC1_Init();

/**
 * @brief Configure what MX_ADC2_Init configures in cube/Src/adc.c.
 */
void MX_ADC2_Init();

/**
 * @brief Configure what MX_ADC3_Init configures in cube/Src/adc.c.
 */
void MX_ADC3_Init();

#endif  // MICRAS_SIM_CUBE_ADC_H
