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
     *
     * @note The core clock of some parts is only within their datasheet above a threshold, 520 MHz
     * on the STM32H72x and STM32H73x, when an option byte says so, and that option byte comes
     * cleared from the factory. A board whose clock tree needs it sets cpu_frequency_boost, the
     * option byte is programmed once per chip with STM32CubeProgrammer, and a chip without it is
     * reported by is_cpu_frequency_supported.
     */
    struct Config {
        InitFunction                  clock_init;
        InitFunction                  peripheral_clock_init;
        std::span<const InitFunction> peripheral_inits;
        bool                          cpu_frequency_boost;
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
     * @note The watchdog is frozen while the core is halted by a debugger, since otherwise any
     * breakpoint would reset the microcontroller as soon as the timeout ran out. Without a debugger
     * the setting has no effect.
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

    /**
     * @brief Check whether the last reset was caused by the independent watchdog.
     *
     * @note The reset flags are read and cleared by init, so this describes the last reset only,
     * and a watchdog reset followed by a reset from the button reads as a normal start.
     *
     * @return True if the watchdog reset the microcontroller, false otherwise.
     */
    static bool was_reset_by_watchdog();

    /**
     * @brief Check whether the core runs within its datasheet.
     *
     * @note The option byte is only read, never programmed, so that nothing but a deliberate step on
     * the bench changes it.
     *
     * @return False if the clock tree of the board needs the CPU frequency boost and the option byte
     * that allows it is not set, true otherwise.
     */
    static bool is_cpu_frequency_supported();

private:
    /**
     * @brief Whether the last reset was caused by the independent watchdog.
     */
    static bool watchdog_reset;

    /**
     * @brief Whether the core runs within its datasheet.
     */
    static bool cpu_frequency_supported;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_MCU_HPP
