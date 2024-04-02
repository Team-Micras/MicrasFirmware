/**
 * @file encoder.hpp
 *
 * @brief STM32 encoder HAL wrapper
 *
 * @date 03/2024
 */

#ifndef MICRAS_HAL_ENCODER_HPP
#define MICRAS_HAL_ENCODER_HPP

#include <tim.h>

namespace hal {
/**
 * @brief Class to handle encoder peripheral on STM32 microcontrollers
 */
class Encoder {
    public:
        /**
         * @brief Encoder configuration struct
         */
        struct Config {
            TIM_HandleTypeDef* handle;
            void               (* init_function)(void);
            uint32_t           timer_channel;
        };

        /**
         * @brief Construct a new Encoder object
         *
         * @param config Configuration for the encoder
         */
        Encoder(Config& config);

        /**
         * @brief Get the counter value
         *
         * @return int32_t Current value of the counter
         */
        int32_t get_counter() const;

    private:
        /**
         * @brief Timer handle
         */
        TIM_HandleTypeDef* handle;
};
}  // namespace hal

#endif // MICRAS_HAL_ENCODER_HPP
