/**
 * @file pwm_dma.hpp
 *
 * @brief STM32 PWM DMA HAL wrapper
 *
 * @date 03/2024
 */

#ifndef MICRAS_HAL_PWM_DMA_HPP
#define MICRAS_HAL_PWM_DMA_HPP

#include <cstdint>
#include <tim.h>

namespace micras::hal {
/**
 * @brief Class to handle PWM peripheral on STM32 microcontrollers using DMA
 */
class PwmDma {
public:
    /**
     * @brief PWM configuration struct
     */
    struct Config {
        void (*init_function)();
        TIM_HandleTypeDef* handle;
        uint32_t           timer_channel;
    };

    /**
     * @brief Construct a new PwmDma object
     *
     * @param config Configuration for the PWM
     */
    explicit PwmDma(const Config& config);

    /**
     * @brief Start PWM and DMA transfer
     *
     * @param buffer Buffer to transfer
     * @param size Size of the buffer
     */
    void start_dma(uint32_t buffer[], uint32_t size);  // NOLINT(*-avoid-c-arrays)

    /**
     * @brief Start PWM and DMA transfer
     *
     * @param buffer Buffer to transfer
     * @param size Size of the buffer
     */
    void start_dma(uint16_t buffer[], uint32_t size);  // NOLINT(*-avoid-c-arrays)

    /**
     * @brief Stop PWM and DMA transfer
     */
    void stop_dma();

    /**
     * @brief Get the compare value for ad duty cycle
     *
     * @param duty_cycle Duty cycle to get the compare value for
     * @return uint32_t Compare value for the duty cycle
     */
    uint32_t get_compare(float duty_cycle) const;

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
}  // namespace micras::hal

#endif  // MICRAS_HAL_PWM_DMA_HPP
