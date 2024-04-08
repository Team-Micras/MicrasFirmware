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

namespace micras::hal {
/**
 * @brief Class to handle encoder peripheral on STM32 microcontrollers
 */
class Encoder {
public:
    /**
     * @brief Encoder configuration struct
     */
    struct Config {
        void (*init_function)();
        TIM_HandleTypeDef* handle;
        uint32_t           timer_channel;
    };

    /**
     * @brief Construct a new Encoder object
     *
     * @param config Configuration for the encoder
     */
    explicit Encoder(const Config& config);

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
}  // namespace micras::hal

#endif  // MICRAS_HAL_ENCODER_HPP
