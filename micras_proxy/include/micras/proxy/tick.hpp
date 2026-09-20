/**
 * @file
 */

#ifndef MICRAS_PROXY_TICK_HPP
#define MICRAS_PROXY_TICK_HPP

#include <cstdint>

namespace micras::proxy {
/**
 * @brief Class that paces a loop at a fixed period.
 *
 * @details Every tick is scheduled from the previous one and not from when the wait was called, so
 * the period does not drift with the time the loop body takes, and a body that overruns is reported
 * rather than absorbed. The deadlines are kept in cycles of the core clock, which resolves a
 * period of a hundred microseconds to a few parts per million, where a timebase in microseconds
 * would jitter by a percent.
 *
 * @note The loop stays a plain loop: the wait spins on the cycle counter and no interrupt is
 * involved. A simulator implements the wait by advancing its physics by one period.
 */
class Tick {
public:
    /**
     * @brief Configuration struct for the tick.
     */
    struct Config {
        uint32_t period_us;
    };

    /**
     * @brief Construct a new Tick object, whose first tick is one period from now.
     *
     * @param config Configuration for the tick.
     */
    explicit Tick(const Config& config);

    /**
     * @brief Wait for the next tick.
     *
     * @return The number of periods since the previous tick, which is one unless the caller took
     * longer than a period, in which case the ticks that were missed are included.
     */
    uint32_t wait();

    /**
     * @brief Get the time since the last tick.
     *
     * @return The time in microseconds, which is how long the body of the loop has taken so far.
     */
    uint32_t elapsed_time_us() const;

private:
    /**
     * @brief Period in cycles of the core clock.
     */
    uint32_t period;

    /**
     * @brief Value of the cycle counter at the last tick.
     */
    uint32_t last_tick;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_TICK_HPP
