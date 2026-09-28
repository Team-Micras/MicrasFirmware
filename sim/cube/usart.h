/**
 * @file
 *
 * @brief Fake STM32CubeMX usart.h of the Micras v1 board: its handles and init functions.
 */

#ifndef MICRAS_SIM_CUBE_USART_H
#define MICRAS_SIM_CUBE_USART_H

#include "main.h"

/**
 * @brief Handle huart4, as cube/Src/usart.c defines it.
 */
extern UART_HandleTypeDef huart4;

/**
 * @brief Configure what MX_UART4_Init configures in cube/Src/usart.c.
 */
void MX_UART4_Init();

#endif  // MICRAS_SIM_CUBE_USART_H
