/**
 * @file
 */

#ifndef MICRAS_HAL_PWM_HPP
#define MICRAS_HAL_PWM_HPP

#include <cstdint>

#include <main.h>

namespace micras::hal {
/**
 * @brief Class to handle PWM peripheral on STM32 microcontrollers.
 */
class Pwm {
public:
    /**
     * @brief PWM configuration struct.
     *
     * @note An inverted output is active for the last part of the period instead of the first. On a
     * center aligned timer that centers its pulse on the overflow instead of the underflow, which
     * is how two groups of channels of one timer are made to take turns. The duty cycle keeps
     * meaning the fraction of the period the output is active for, so zero is always off.
     */
    struct Config {
        void (*init_function)();
        TIM_HandleTypeDef* handle;
        uint32_t           timer_channel;
        bool               inverted;
    };

    /**
     * @brief Construct a new Pwm object, with its output at a duty cycle of zero.
     *
     * @note The compare register is preloaded, so a value written to it waits for the next update
     * of the timer. The first one is written around the preload: an inverted output would otherwise
     * start fully on, since zero is what the register holds until then.
     *
     * @param config Configuration for the PWM.
     */
    explicit Pwm(const Config& config);

    /**
     * @brief Set the PWM duty cycle.
     *
     * @param duty_cycle Duty cycle value in percent, which is clamped to the range from 0 to 100.
     */
    void set_duty_cycle(float duty_cycle);

    /**
     * @brief Set the PWM frequency.
     *
     * @note Changing the pwm frequency will modify the autoreload and reset the counter,
     * but the compare value will stay the same, so the duty cycle will be different from the previously set value.
     * There is a minimum and maximum frequency that can be set by changing the autoreload register.
     * The minimum frequency is base_freq / (2^timer_resolution * (prescaler + 1)).
     * The maximum frequency is base_freq / (prescaler + 1).
     *
     * @param frequency Frequency value in Hz.
     */
    void set_frequency(uint32_t frequency);

    /**
     * @brief Check if the PWM was successfully started.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Timer handle.
     */
    TIM_HandleTypeDef* handle;

    /**
     * @brief Channel number of the timer.
     */
    uint32_t channel;

    /**
     * @brief Whether the output is active for the last part of the period.
     */
    bool inverted;

    /**
     * @brief Flag to check if the PWM was started.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_PWM_HPP
