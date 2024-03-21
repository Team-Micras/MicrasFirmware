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

#include "hal/timer.hpp"

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
            Timer::Config timer;
            uint32_t      timer_channel;
        };

        /**
         * @brief Construct a new Pwm object
         *
         * @param pwm_config Configuration for the PWM
         */
        Pwm(const Config& pwm_config);

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
         * @brief Timer object
         */
        Timer timer;

        /**
         * @brief Channel number of the timer
         */
        uint32_t channel;
};
}  // namespace hal

#endif // __PWM_HPP__
