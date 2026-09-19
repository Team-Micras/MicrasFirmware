/**
 * @file
 */

#include <cstdint>

#include <main.h>
#include "micras/hal/timer.hpp"

namespace micras::hal {
/**
 * @brief Key that unlocks write access to the debug components on the Cortex-M7.
 */
static constexpr uint32_t software_lock_key{0xC5ACCE55};

uint32_t Timer::cycles_per_microsecond{1};

void Timer::init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    if ((DWT->LSR & ITM_LSR_Present_Msk) != 0 and (DWT->LSR & ITM_LSR_Access_Msk) != 0) {
        DWT->LAR = software_lock_key;
    }

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_microsecond = SystemCoreClock / 1000000;
}

uint32_t Timer::get_counter() {
    return DWT->CYCCNT;
}

uint32_t Timer::get_counter_ms() {
    return HAL_GetTick();
}

uint32_t Timer::to_microseconds(uint32_t cycles) {
    return cycles / cycles_per_microsecond;
}

uint32_t Timer::to_cycles(uint32_t microseconds) {
    return microseconds * cycles_per_microsecond;
}
}  // namespace micras::hal
