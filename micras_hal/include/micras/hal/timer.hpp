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
    Timer() = default;

    /**
     * @brief Construct a new Timer object.
     *
     * @param config Configuration for the timer.
     */
    explicit Timer(const Config& config);

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

private:
    /**
     * @brief Timer handle.
     */
    TIM_HandleTypeDef* handle{};

    /**
     * @brief Flag to enable microseconds.
     */
    bool enable_microseconds{false};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_TIMER_HPP
