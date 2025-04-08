/**
 * @file
 */

#ifndef MICRAS_PROXY_STOPWATCH_HPP
#define MICRAS_PROXY_STOPWATCH_HPP

#include "micras/hal/timer.hpp"

namespace micras::proxy {
/**
 * @brief Class to measure the time elapsed between two events.
 */
class Stopwatch {
public:
    /**
     * @brief Stopwatch configuration struct.
     */
    struct Config {
        hal::Timer::Config timer;
    };

    /**
     * @brief Construct a new Stopwatch object.
     */
    Stopwatch();

    /**
     * @brief Construct a new Stopwatch object.
     *
     * @param config Configuration for the timer.
     */
    explicit Stopwatch(const Config& config);

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
    void sleep_us(uint32_t time) const;

private:
    hal::Timer timer;

    /**
     * @brief Stopwatch counter.
     */
    uint32_t counter{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_STOPWATCH_HPP
