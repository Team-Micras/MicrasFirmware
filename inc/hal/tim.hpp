/**
 * @file tim.hpp
 *
 * @brief STM32 TIM HAL wrapper.
 *
 * @date 03/2024
 */

#ifndef __TIM_HPP__
#define __TIM_HPP__

#include <cstdint>
#include <functional>

#include "tim.h"

namespace hal {
/**
 * @brief Class to handle TIM peripheral on STM32 microcontrollers.
 */
class Tim {
    public:
        /**
         * @brief TIM configuration struct.
         */
        struct Config {
            TIM_HandleTypeDef*        handle;
            std::function<void(void)> init_function;
        };

        /**
         * @brief Constructor for the Tim class.
         *
         * @param tim_config Configuration for timer handler
         */
        Tim(Config& tim_config);

        /**
         * @brief Initialize timer base generation.
         */
        void base_start();

        /**
         * @brief Stop timer base generation.
         */
        void base_stop();

        /**
         * @brief Set timer counter.
         *
         * @param counter Counter value.
         */
        void set_counter(uint32_t counter);

        /**
         * @brief Get timer counter.
         *
         * @return uint32_t Counter value.
         */
        uint32_t get_counter();

        /**
         * @brief Start PWM generation.
         *
         * @param channel Tim channel to be enabled.
         */
        void pwm_start(uint32_t channel);

        /**
         * @brief Stop PWM generation.
         *
         * @param channel Tim channel to be disabled.
         */
        void pwm_stop(uint32_t channel);

        /**
         * @brief Set timer prescaler.
         *
         * @param prescaler Prescaler new value.
         */
        void set_prescaler(uint32_t prescaler);

        /**
         * @brief Set the TIM Capture Compare Register value.
         *
         * @param channel TIM channel to be configured.
         * @param compare Compare register new value.
         */
        void set_compare(uint32_t channel, uint32_t compare);

        /**
         * @brief Get the Autoreload register value.
         *
         * @return uint32_t The value of the Autoreload register.
         */
        uint32_t get_autoreload();

    private:
        /**
         * @brief Timer handler.
         */
        TIM_HandleTypeDef* handle;
};
}  // namespace hal

#endif // __TIM_HPP__
