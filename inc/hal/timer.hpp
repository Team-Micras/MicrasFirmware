/**
 * @file timer.hpp
 *
 * @brief STM32 Timer HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <cstdint>
#include <tim.h>

namespace hal {
/**
 * @brief Class to handle timer peripheral on STM32 microcontrollers
 */
class Timer {
    public:
        /**
         * @brief Timer configuration struct
         */
        struct Config {
            TIM_HandleTypeDef* handle;
            void               (* init_function)(void);
        };

        /**
         * @brief Constructor for the Timer class
         *
         * @param timer_config Configuration for timer handler
         */
        Timer(Config& timer_config);

        /**
         * @brief Initialize timer base generation
         */
        void base_start();

        /**
         * @brief Stop timer base generation
         */
        void base_stop();

        /**
         * @brief Set timer counter
         *
         * @param counter Counter value
         */
        void set_counter(uint32_t counter);

        /**
         * @brief Get timer counter
         *
         * @return uint32_t Counter value
         */
        uint32_t get_counter();

        /**
         * @brief Start PWM generation
         *
         * @param channel Timer channel to be enabled
         */
        void pwm_start(uint32_t channel);

        /**
         * @brief Stop PWM generation
         *
         * @param channel Timer channel to be disabled
         */
        void pwm_stop(uint32_t channel);

        /**
         * @brief Set timer prescaler
         *
         * @param prescaler Prescaler new value
         */
        void set_prescaler(uint32_t prescaler);

        /**
         * @brief Get the timer prescaler
         *
         * @return uint32_t The value of the prescaler
         */
        uint32_t get_prescaler();

        /**
         * @brief Set the timer Capture Compare Register value
         *
         * @param channel Timer channel to be configured
         * @param compare Compare register new value
         */
        void set_compare(uint32_t channel, uint32_t compare);

        /**
         * @brief Get the Autoreload register value
         *
         * @return uint32_t The value of the Autoreload register
         */
        uint32_t get_autoreload();

        /**
         * @brief Set the Autoreload register value
         *
         * @param autoreload New value for the Autoreload register
         */
        void set_autoreload(uint32_t autoreload);

        /**
         * @brief Get the timer clock frequency
         *
         * @return uint32_t Timer clock frequency
         */
        uint32_t get_timer_clock_freq();

    private:
        /**
         * @brief Timer handler
         */
        TIM_HandleTypeDef* handle;
};
}  // namespace hal

#endif // __TIMER_HPP__
