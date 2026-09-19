/**
 * @file
 */

#ifndef MICRAS_HAL_MCU_HPP
#define MICRAS_HAL_MCU_HPP

#include <cstdint>
#include <span>

#include "micras/hal/gpio.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::hal {
/**
 * @brief Microcontroller unit class.
 */
class Mcu {
public:
    /**
     * @brief Deleted constructor for static class.
     */
    Mcu() = delete;

    /**
     * @brief Initialize MCU and some peripherals.
     */
    static void init();

    /**
     * @brief Silence every actuator as directly as possible.
     *
     * @note Written to be callable from a fault or abort handler, so it touches only the timer and
     * GPIO registers and relies on no object state.
     *
     * @param pwm_outputs Every PWM output to bring to a null duty cycle.
     * @param enable_gpios Every driver enable pin to deassert.
     */
    static void emergency_stop(std::span<const Pwm::Config> pwm_outputs, std::span<const Gpio::Config> enable_gpios);

    /**
     * @brief Start the independent watchdog, or change the timeout of a running one.
     *
     * @note Once started the watchdog cannot be stopped by software, so anything that hangs for
     * longer than the timeout resets the microcontroller, which brings every driver enable pin and
     * every PWM output back to its reset state. The timeout can still be widened around an
     * operation that legitimately stalls the core for longer than the control loop budget, such as
     * erasing a flash sector.
     *
     * @param timeout_ms Time without a refresh that triggers a reset, in milliseconds.
     */
    static void set_watchdog_timeout(uint32_t timeout_ms);

    /**
     * @brief Refresh the independent watchdog.
     */
    static void refresh_watchdog();
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_MCU_HPP
