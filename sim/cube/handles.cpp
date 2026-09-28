/**
 * @file
 *
 * @brief The Micras v1 handles and init functions, with the values CubeMX generates.
 *
 * @note Each MX_*_Init below sets the values its namesake sets in
 *       MicrasFirmware/cube/Src: MX_TIMn_Init the prescaler, counter mode and
 *       period of tim.c, MX_ADCn_Init the NbrOfConversion of adc.c, MX_CRC_Init
 *       the whole Init block of crc.c, MX_SPI3_Init the SPI mode and baud rate
 *       prescaler of spi.c and MX_UART4_Init the baud rate of usart.c. Only what the host backend reads
 *       is set, plus the handle states the drivers check before initialising.
 */

#include <adc.h>
#include <crc.h>
#include <dma.h>
#include <fmac.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>

#include "micras/hal/host/board.hpp"

namespace {
/**
 * @brief Frequency every timer counts at: APB1 and APB2 run at 137.5 MHz, and
 *        their timers at twice that (main.c, SystemClock_Config).
 */
constexpr uint32_t timer_clock{275000000};

/**
 * @brief Frequency SPI3's kernel clock runs at: PLL3 from the 64 MHz HSI, M 32,
 *        N 125 and P 2 (spi.c, HAL_SPI_MspInit).
 */
constexpr uint32_t spi3_kernel_clock{125000000};

/**
 * @brief The registers of the timers and of SPI3.
 */
///@{
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables): register blocks the firmware writes through handles.
TIM_TypeDef tim1_registers{};
TIM_TypeDef tim2_registers{};
TIM_TypeDef tim3_registers{};
TIM_TypeDef tim4_registers{};
TIM_TypeDef tim5_registers{};
TIM_TypeDef tim8_registers{};
TIM_TypeDef tim12_registers{};
TIM_TypeDef tim15_registers{};
SPI_TypeDef spi3_registers{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

///@}

/**
 * @brief Configure a timer base, as HAL_TIM_Base_Init would.
 *
 * @param handle Timer handle.
 * @param registers Registers of the timer.
 * @param name Name of the handle, for messages.
 * @param prescaler Prescaler.
 * @param counter_mode Counter mode.
 * @param period Autoreload value.
 */
void init_timer(
    TIM_HandleTypeDef& handle, TIM_TypeDef& registers, const char* name, uint32_t prescaler, uint32_t counter_mode,
    uint32_t period
) {
    handle.Instance = &registers;
    handle.Init = {.Prescaler = prescaler, .CounterMode = counter_mode, .Period = period};
    registers.PSC = prescaler;
    registers.ARR = period;
    registers.CR1 = counter_mode;
    registers.kernel_clock = timer_clock;
    handle.State = HAL_TIM_STATE_READY;
    micras::hal::host::Board::name_handle(&handle, name);
}

/**
 * @brief Configure an ADC, as HAL_ADC_Init would.
 *
 * @param handle ADC handle.
 * @param name Name of the handle, for messages.
 * @param conversions Number of conversions in a sequence.
 */
void init_adc(ADC_HandleTypeDef& handle, const char* name, uint32_t conversions) {
    handle.Init.NbrOfConversion = conversions;
    handle.State = HAL_ADC_STATE_READY;
    micras::hal::host::Board::name_handle(&handle, name);
}
}  // namespace

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables): the handles are globals in the generated code too.
uint32_t SystemCoreClock{550000000};

GPIO_TypeDef GPIOA_instance{'A'};
GPIO_TypeDef GPIOB_instance{'B'};
GPIO_TypeDef GPIOC_instance{'C'};
GPIO_TypeDef GPIOD_instance{'D'};

ADC_HandleTypeDef  hadc1{};
ADC_HandleTypeDef  hadc2{};
ADC_HandleTypeDef  hadc3{};
CRC_HandleTypeDef  hcrc{};
FMAC_HandleTypeDef hfmac{};
SPI_HandleTypeDef  hspi3{};
TIM_HandleTypeDef  htim1{};
TIM_HandleTypeDef  htim2{};
TIM_HandleTypeDef  htim3{};
TIM_HandleTypeDef  htim4{};
TIM_HandleTypeDef  htim5{};
TIM_HandleTypeDef  htim8{};
TIM_HandleTypeDef  htim12{};
TIM_HandleTypeDef  htim15{};
UART_HandleTypeDef huart4{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

extern "C" {
void SystemClock_Config() { }

void PeriphCommonClock_Config() { }
}

void MX_GPIO_Init() {
    using micras::hal::host::Board;
    Board::name_gpio(Encoder_Right_CSn_GPIO_Port, Encoder_Right_CSn_Pin, "Encoder_Right_CSn");
    Board::name_gpio(Encoder_Left_CSn_GPIO_Port, Encoder_Left_CSn_Pin, "Encoder_Left_CSn");
    Board::name_gpio(Button_GPIO_Port, Button_Pin, "Button");
    Board::name_gpio(Switch_0_GPIO_Port, Switch_0_Pin, "Switch_0");
    Board::name_gpio(Switch_1_GPIO_Port, Switch_1_Pin, "Switch_1");
    Board::name_gpio(Switch_2_GPIO_Port, Switch_2_Pin, "Switch_2");
    Board::name_gpio(Switch_3_GPIO_Port, Switch_3_Pin, "Switch_3");
    Board::name_gpio(LED_Red_GPIO_Port, LED_Red_Pin, "LED_Red");
    Board::name_gpio(Fan_Enable_GPIO_Port, Fan_Enable_Pin, "Fan_Enable");
    Board::name_gpio(Motors_Enable_GPIO_Port, Motors_Enable_Pin, "Motors_Enable");
    Board::name_gpio(IMU_SPI_CSn_GPIO_Port, IMU_SPI_CSn_Pin, "IMU_SPI_CSn");
}

void MX_DMA_Init() { }

void MX_CRC_Init() {
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
    hcrc.Init.GeneratingPolynomial = 29;
    hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
    hcrc.Init.InitValue = 0xC4;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    micras::hal::host::Board::name_handle(&hcrc, "hcrc");
}

void MX_ADC1_Init() {
    init_adc(hadc1, "hadc1", 4);
}

void MX_ADC2_Init() {
    init_adc(hadc2, "hadc2", 2);
}

void MX_ADC3_Init() {
    init_adc(hadc3, "hadc3", 1);
}

void MX_SPI3_Init() {
    spi3_registers.kernel_clock = spi3_kernel_clock;
    hspi3.Instance = &spi3_registers;
    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi3.State = HAL_SPI_STATE_READY;
    micras::hal::host::Board::name_handle(&hspi3, "hspi3");
}

void MX_TIM1_Init() {
    init_timer(htim1, tim1_registers, "htim1", 0, TIM_COUNTERMODE_UP, 2749);
}

void MX_TIM2_Init() {
    init_timer(htim2, tim2_registers, "htim2", 0, TIM_COUNTERMODE_UP, 4294967295);
}

void MX_TIM3_Init() {
    init_timer(htim3, tim3_registers, "htim3", 0, TIM_COUNTERMODE_UP, 2749);
}

void MX_TIM4_Init() {
    init_timer(htim4, tim4_registers, "htim4", 274, TIM_COUNTERMODE_CENTERALIGNED1, 250);
}

void MX_TIM5_Init() {
    init_timer(htim5, tim5_registers, "htim5", 0, TIM_COUNTERMODE_UP, 4294967295);
}

void MX_TIM8_Init() {
    init_timer(htim8, tim8_registers, "htim8", 4, TIM_COUNTERMODE_UP, 76);
}

void MX_TIM12_Init() {
    init_timer(htim12, tim12_registers, "htim12", 0, TIM_COUNTERMODE_UP, 2749);
}

void MX_TIM15_Init() {
    init_timer(htim15, tim15_registers, "htim15", 274, TIM_COUNTERMODE_UP, 65535);
}

void MX_UART4_Init() {
    huart4.Init.BaudRate = 115200;
    huart4.gState = HAL_UART_STATE_READY;
    huart4.RxState = HAL_UART_STATE_READY;
    micras::hal::host::Board::name_handle(&huart4, "huart4");
}

void MX_FMAC_Init() { }
