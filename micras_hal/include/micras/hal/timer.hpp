/**
 * @file
 */

#ifndef MICRAS_HAL_TIMER_HPP
#define MICRAS_HAL_TIMER_HPP

#include <cstdint>

namespace micras::hal {
/**
 * @brief Class providing the timebases of the microcontroller.
 *
 * @note The microsecond timebase comes from the Cortex-M cycle counter instead of a hardware timer:
 * it costs no peripheral, resolves single cycles and is present on every Cortex-M3, M4 and M7, so
 * the same implementation ports unchanged. Cortex-M0 and M0+ have no cycle counter and would need a
 * timer here.
 */
class Timer {
public:
    /**
     * @brief Deleted constructor for static class.
     */
    Timer() = delete;

    /**
     * @brief Start the cycle counter and compute the conversion to microseconds.
     *
     * @note Must be called after the system clock is configured, since the conversion depends on
     * the core clock frequency.
     */
    static void init();

    /**
     * @brief Get the current value of the free running cycle counter.
     *
     * @note Wraps every 2^32 core clock cycles, which bounds the longest measurable interval to
     * about 7.8 s at 550 MHz. Differences taken with unsigned arithmetic are correct across the
     * wrap, so no special case is needed below that bound.
     *
     * @return Current value of the cycle counter.
     */
    static uint32_t get_counter();

    /**
     * @brief Get the current timer counter.
     *
     * @return Current timer counter in milliseconds.
     */
    static uint32_t get_counter_ms();

    /**
     * @brief Convert a number of core clock cycles to microseconds.
     *
     * @param cycles Number of core clock cycles.
     * @return Equivalent time in microseconds.
     */
    static uint32_t to_microseconds(uint32_t cycles);

    /**
     * @brief Convert a time in microseconds to core clock cycles.
     *
     * @param microseconds Time in microseconds.
     * @return Equivalent number of core clock cycles.
     */
    static uint32_t to_cycles(uint32_t microseconds);

private:
    /**
     * @brief Core clock cycles in one microsecond, resolved by init.
     */
    static uint32_t cycles_per_microsecond;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_TIMER_HPP
