#ifndef MICRAS_HAL_TIMER_HPP
#define MICRAS_HAL_TIMER_HPP
#include <cstdint>

namespace micras::hal {
class Timer {
public:
    static inline uint32_t counter = 0;                   // cycle counter, which the test winds
    static inline uint32_t cycles_per_microsecond = 550;  // the core clock at 550 MHz

    static uint32_t get_counter() { return counter; }

    static uint32_t to_microseconds(uint32_t cycles) { return cycles / cycles_per_microsecond; }

    static uint32_t to_cycles(uint32_t microseconds) { return microseconds * cycles_per_microsecond; }
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_TIMER_HPP
