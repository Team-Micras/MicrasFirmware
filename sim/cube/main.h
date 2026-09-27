/**
 * @file
 *
 * @brief Fake STM32CubeMX main.h of the Micras v1 board, for the simulator.
 *
 * @note Hand written, because micras_v1.ioc does not hold every value the host
 *       backend needs. Every value names the
 *       generated line it mirrors, in MicrasFirmware/cube/Inc and cube/Src. A
 *       CubeMX change that renames a handle or a pin is a compile error here; a
 *       changed prescaler or period shows as a wrong frequency at initialisation.
 */

#ifndef MICRAS_SIM_CUBE_MAIN_H
#define MICRAS_SIM_CUBE_MAIN_H

#include <cstdint>

#include "stm32_host.h"

/*****************************************
 * Microcontroller
 *****************************************/

/**
 * @brief Core clock after SystemClock_Config: HSI 64 MHz / PLLM 4 * (PLLN 34 + FRACN 3072 / 8192) / PLLP 1.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Flash geometry of the STM32H725xG: one bank of eight 128 KB sectors, 256-bit flash words.
 */
///@{
#define FLASH_NB_32BITWORD_IN_FLASHWORD 8U
inline constexpr uint32_t FLASH_SECTOR_SIZE{0x00020000U};
inline constexpr uint32_t FLASH_SECTOR_TOTAL{8U};
///@}

/*****************************************
 * GPIO ports and the pins target.hpp names (cube/Inc/main.h)
 *****************************************/

///@{
extern GPIO_TypeDef GPIOA_instance;
extern GPIO_TypeDef GPIOB_instance;
extern GPIO_TypeDef GPIOC_instance;
extern GPIO_TypeDef GPIOD_instance;
///@}

///@{
#define GPIOA (&GPIOA_instance)
#define GPIOB (&GPIOB_instance)
#define GPIOC (&GPIOC_instance)
#define GPIOD (&GPIOD_instance)
///@}

///@{
#define Encoder_Right_CSn_Pin GPIO_PIN_14
#define Encoder_Right_CSn_GPIO_Port GPIOC
#define Encoder_Left_CSn_Pin GPIO_PIN_3
#define Encoder_Left_CSn_GPIO_Port GPIOA
#define Button_Pin GPIO_PIN_7
#define Button_GPIO_Port GPIOA
#define Switch_0_Pin GPIO_PIN_5
#define Switch_0_GPIO_Port GPIOC
#define Switch_1_Pin GPIO_PIN_0
#define Switch_1_GPIO_Port GPIOB
#define Switch_2_Pin GPIO_PIN_2
#define Switch_2_GPIO_Port GPIOB
#define Switch_3_Pin GPIO_PIN_10
#define Switch_3_GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_12
#define LED_Red_GPIO_Port GPIOB
#define Fan_Enable_Pin GPIO_PIN_14
#define Fan_Enable_GPIO_Port GPIOB
#define Motors_Enable_Pin GPIO_PIN_10
#define Motors_Enable_GPIO_Port GPIOA
#define IMU_SPI_CSn_Pin GPIO_PIN_2
#define IMU_SPI_CSn_GPIO_Port GPIOD
///@}

#endif  // MICRAS_SIM_CUBE_MAIN_H
