/**
 * @file pwm.hpp
 *
 * @brief STM32 PWM HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __PWM_HPP__
#define __PWM_HPP__

#include <cstdint>
#include <tim.h>

namespace hal {
/**
 * @brief Class to handle PWM peripheral on STM32 microcontrollers
 */
class Pwm {
    public:
        /**
         * @brief PWM configuration struct
         */
        struct Config {
            TIM_HandleTypeDef* handle;
            void               (* init_function)(void);
            uint32_t           timer_channel;
        };

        /**
         * @brief Construct a new Pwm object
         *
         * @param config Configuration for the PWM
         */
        Pwm(Config& config);

        /**
         * @brief Set the PWM duty cycle
         *
         * @param duty_cycle Duty cycle value
         */
        void set_duty_cycle(float duty_cycle);

        /**
         * @brief Set the PWM frequency
         *
         * @param frequency Frequency value in Hz
         */
        void set_frequency(uint32_t frequency);

    private:
        /**
         * @brief Timer handle
         */
        TIM_HandleTypeDef* handle;

        /**
         * @brief Channel number of the timer
         */
        uint32_t channel;
};
}  // namespace hal

#endif // __PWM_HPP__
