/**
 * @file
 */

#include <algorithm>
#include <crc.h>
#include <cstdint>
#include <dma.h>
#include <gpio.h>
#include <span>

#include "micras/hal/gpio.hpp"
#include "micras/hal/mcu.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/hal/timer.hpp"

extern "C" {
/**
 * @brief Initialize System Clock.
 *
 * @note  Defined by cube.
 */
void SystemClock_Config();

/**
 * @brief Initialize the kernel clocks of the peripherals that do not run from an APB clock.
 *
 * @note  Defined by cube.
 */
void PeriphCommonClock_Config();
}

namespace micras::hal {
/**
 * @brief Independent watchdog instance, spelled IWDG1 on the parts that have more than one.
 */
static IWDG_TypeDef* watchdog_instance() {
#ifdef IWDG1
    return IWDG1;
#else
    return IWDG;
#endif
}

/**
 * @brief Key values of the independent watchdog key register.
 */
///@{
static constexpr uint32_t watchdog_key_reload{0xAAAA};
static constexpr uint32_t watchdog_key_write{0x5555};
static constexpr uint32_t watchdog_key_start{0xCCCC};
///@}

/**
 * @brief Smallest divider the watchdog prescaler can apply to the low speed oscillator.
 */
static constexpr uint32_t watchdog_min_divider{4};

/**
 * @brief Largest value the watchdog prescaler register can hold.
 */
static constexpr uint32_t watchdog_max_prescaler{6};

/**
 * @brief Largest value the 12 bit watchdog reload register can hold.
 */
static constexpr uint32_t watchdog_max_reload{0xFFF};

/**
 * @brief Time to wait for the watchdog registers to take effect before giving up.
 *
 * @note Bounded so that a low speed oscillator that never starts cannot hang the boot.
 */
static constexpr uint32_t watchdog_timeout_us{1000};

void Mcu::init() {
    SCB_EnableICache();

    HAL_Init();

    SystemClock_Config();
    PeriphCommonClock_Config();

    Timer::init();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CRC_Init();
}

void Mcu::emergency_stop(std::span<const Pwm::Config> pwm_outputs, std::span<const Gpio::Config> enable_gpios) {
    for (const auto& pwm_output : pwm_outputs) {
        __HAL_TIM_SET_COMPARE(pwm_output.handle, pwm_output.timer_channel, 0);
    }

    for (const auto& enable_gpio : enable_gpios) {
        HAL_GPIO_WritePin(enable_gpio.port, enable_gpio.pin, GPIO_PIN_RESET);
    }
}

void Mcu::set_watchdog_timeout(uint32_t timeout_ms) {
    uint32_t prescaler = 0;
    uint32_t ticks = timeout_ms * LSI_VALUE / (watchdog_min_divider * 1000);

    while (ticks > watchdog_max_reload + 1 and prescaler < watchdog_max_prescaler) {
        prescaler++;
        ticks /= 2;
    }

    const uint32_t reload = std::clamp<uint32_t>(ticks, 1, watchdog_max_reload + 1) - 1;

    watchdog_instance()->KR = watchdog_key_start;
    watchdog_instance()->KR = watchdog_key_write;
    watchdog_instance()->PR = prescaler;
    watchdog_instance()->RLR = reload;

    const uint32_t start = Timer::get_counter();
    const uint32_t limit = Timer::to_cycles(watchdog_timeout_us);

    while ((watchdog_instance()->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0) {
        if (Timer::get_counter() - start > limit) {
            break;
        }
    }

    refresh_watchdog();
}

void Mcu::refresh_watchdog() {
    watchdog_instance()->KR = watchdog_key_reload;
}
}  // namespace micras::hal
