/**
 * @file
 */

#ifndef MICRAS_HAL_PWM_HPP
#define MICRAS_HAL_PWM_HPP

#include <cstdint>
#include <tim.h>

namespace micras::hal {
/**
 * @brief Class to handle PWM peripheral on STM32 microcontrollers.
 */
class Pwm {
public:
    /**
     * @brief PWM configuration struct.
     */
    struct Config {
        void (*init_function)();
        TIM_HandleTypeDef* handle;
        uint32_t           timer_channel;
    };

    /**
     * @brief Construct a new Pwm object.
     *
     * @param config Configuration for the PWM.
     */
    explicit Pwm(const Config& config);

    /**
     * @brief Set the PWM duty cycle.
     *
     * @param duty_cycle Duty cycle value.
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

private:
    /**
     * @brief Timer handle.
     */
    TIM_HandleTypeDef* handle;

    /**
     * @brief Channel number of the timer.
     */
    uint32_t channel;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_PWM_HPP
