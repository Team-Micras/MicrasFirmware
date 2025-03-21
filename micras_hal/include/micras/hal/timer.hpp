/**
 * @file
 */

#ifndef MICRAS_HAL_TIMER_HPP
#define MICRAS_HAL_TIMER_HPP

#include <cstdint>
#include <tim.h>

namespace micras::hal {
/**
 * @brief Class to handle timer peripheral on STM32 microcontrollers.
 */
class Timer {
public:
    /**
     * @brief Timer configuration struct.
     */
    struct Config {
        void (*init_function)();
        TIM_HandleTypeDef* handle;
    };

    /**
     * @brief Construct a new Timer object.
     */
    Timer();

    /**
     * @brief Construct a new Timer object.
     *
     * @param config Configuration for the timer.
     */
    explicit Timer(const Config& config);

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
    /**
     * @brief Get the current timer counter.
     *
     * @return Current timer counter in milliseconds.
     */
    static uint32_t get_counter_ms();

    /**
     * @brief Get the current timer counter.
     *
     * @return Current timer counter in microseconds.
     */
    uint32_t get_counter_us() const;

    /**
     * @brief Timer handle.
     */
    TIM_HandleTypeDef* handle{};

    /**
     * @brief Timer counter.
     */
    uint32_t counter{};

    /**
     * @brief Flag to enable microseconds.
     */
    bool enable_microseconds{false};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_TIMER_HPP
