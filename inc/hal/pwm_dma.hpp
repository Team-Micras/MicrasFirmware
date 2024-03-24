/**
 * @file pwm_dma.hpp
 *
 * @brief STM32 PWM DMA HAL wrapper
 *
 * @date 03/2024
 */

#ifndef __PWM_DMA_HPP__
#define __PWM_DMA_HPP__

#include <cstdint>
#include <tim.h>

#include "hal/timer.hpp"

namespace hal {
/**
 * @brief Class to handle PWM peripheral on STM32 microcontrollers using DMA
 */
class PwmDma {
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
         * @brief Construct a new PwmDma object
         *
         * @param pwm_config Configuration for the PWM
         */
        PwmDma(Config& pwm_config);

        /**
         * @brief Start PWM and DMA transfer
         *
         * @param buffer Buffer to transfer
         * @param size Size of the buffer
         */
        void start_dma(uint32_t buffer[], uint32_t size);

        /**
         * @brief Stop PWM and DMA transfer
         */
        void stop_dma();

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

extern "C" {
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    HAL_TIM_PWM_Stop_DMA(htim, hal::Timer::get_channel_from_active(HAL_TIM_GetActiveChannel(htim)));
}
}
#endif // __PWM_DMA_HPP__
