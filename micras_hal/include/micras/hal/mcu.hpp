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
     * @brief Signature of the initialization function of a peripheral.
     */
    using InitFunction = void (*)();

    /**
     * @brief Configuration struct for the microcontroller.
     *
     * @note The initialization functions are taken rather than named, so that this package refers
     * to no symbol the application generates. The two clock functions are separate fields because
     * the order matters: the clock tree has to be running before the timebase is resolved from the
     * core clock frequency, and before any peripheral is initialized.
     */
    struct Config {
        InitFunction                  clock_init;
        InitFunction                  peripheral_clock_init;
        std::span<const InitFunction> peripheral_inits;
    };

    /**
     * @brief Deleted constructor for static class.
     */
    Mcu() = delete;

    /**
     * @brief Initialize the microcontroller, the timebase and the peripherals that no wrapper owns.
     *
     * @note Every peripheral held by a wrapper is initialized by that wrapper instead, through the
     * initialization function of its own configuration, so peripheral_inits carries only what is
     * left over: the pin configuration, the DMA controllers and anything else without an owner.
     *
     * @param config Initialization functions of the board, which may leave peripheral_clock_init
     * null on a part where every peripheral runs from a bus clock.
     */
    static void init(const Config& config);

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
