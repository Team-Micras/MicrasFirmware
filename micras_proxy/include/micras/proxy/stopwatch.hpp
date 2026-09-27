/**
 * @file
 */

#ifndef MICRAS_PROXY_STOPWATCH_HPP
#define MICRAS_PROXY_STOPWATCH_HPP

#include <cstdint>

namespace micras::proxy {
/**
 * @brief Class to measure the time elapsed between two events.
 *
 * @note The millisecond and microsecond timebases are independent, so an object can be reset in one
 * unit and read in the other without the two interfering.
 */
class Stopwatch {
public:
    /**
     * @brief Construct a new Stopwatch object, starting both timebases.
     */
    Stopwatch();

    /**
     * @brief Reset the milliseconds timer counter.
     */
    void reset_ms();

    /**
     * @brief Reset the microseconds timer counter.
     */
    void reset_us();

    /**
     * @brief Get the time elapsed since the last reset.
     *
     * @return Time elapsed in miliseconds.
     */
    uint32_t elapsed_time_ms() const;

    /**
     * @brief Get the time elapsed since the last reset.
     *
     * @return Time elapsed in microseconds.
     */
    uint32_t elapsed_time_us() const;

    /**
     * @brief Sleep for a given amount of time.
     *
     * @param time Time to sleep in milliseconds.
     */
    static void sleep_ms(uint32_t time);

    /**
     * @brief Sleep for a given amount of time.
     *
     * @param time Time to sleep in microseconds.
     */
    static void sleep_us(uint32_t time);

private:
    /**
     * @brief Value of the millisecond counter at the last reset.
     */
    uint32_t counter_ms{};

    /**
     * @brief Value of the cycle counter at the last reset.
     */
    uint32_t counter_cycles{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_STOPWATCH_HPP
