/**
 * @file
 */

#ifndef MICRAS_PROXY_MICROSECOND_CLOCK_HPP
#define MICRAS_PROXY_MICROSECOND_CLOCK_HPP

#include <cstdint>

namespace micras::proxy {
/**
 * @brief Class for a free running clock in microseconds, which wraps only every 2^32 us.
 *
 * @details The cycle counter wraps every 2^32 core clock cycles, about 7.8 s at 550 MHz, so a time
 * converted from it wraps as often. This clock adds the cycles elapsed since it was last read to the
 * time it keeps instead, carrying the cycles that do not make a whole microsecond to the next read,
 * so it neither drifts nor wraps before 2^32 us, about 71.6 minutes.
 *
 * @note The clock has to be read at least once every wrap of the cycle counter, which the control
 * loop does many times over.
 */
class MicrosecondClock {
public:
    /**
     * @brief Construct a new MicrosecondClock object, which starts at zero.
     */
    MicrosecondClock();

    /**
     * @brief Get the time since the clock was constructed.
     *
     * @return The time in microseconds, wrapping every 2^32 us.
     */
    uint32_t now_us();

private:
    /**
     * @brief Value of the cycle counter at the last read.
     */
    uint32_t last_counter;

    /**
     * @brief Cycles counted at the last read that did not make a whole microsecond.
     */
    uint32_t remainder_cycles{};

    /**
     * @brief Time kept by the clock, in microseconds.
     */
    uint32_t time_us{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_MICROSECOND_CLOCK_HPP
