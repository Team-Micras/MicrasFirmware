/**
 * @file
 */

#ifndef MICRAS_HAL_ENCODER_HPP
#define MICRAS_HAL_ENCODER_HPP

#include <tim.h>

namespace micras::hal {
/**
 * @brief Class to handle encoder peripheral on STM32 microcontrollers.
 */
class Encoder {
public:
    /**
     * @brief Encoder configuration struct.
     */
    struct Config {
        void (*init_function)();
        TIM_HandleTypeDef* handle;
        uint32_t           timer_channel;
    };

    /**
     * @brief Construct a new Encoder object.
     *
     * @param config Configuration for the encoder.
     */
    explicit Encoder(const Config& config);

    /**
     * @brief Get the counter value.
     *
     * @return Current value of the counter.
     */
    int32_t get_counter() const;

private:
    /**
     * @brief Timer handle.
     */
    TIM_HandleTypeDef* handle;

    /**
     * @brief Start value of the timer counter.
     */
    uint32_t start_count;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ENCODER_HPP
