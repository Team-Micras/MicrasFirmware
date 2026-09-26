/**
 * @file
 */

#include <csignal>

#include "micras/hal/gpio.hpp"
#include "micras/hal/mcu.hpp"
#include "micras/micras.hpp"
#include "target.hpp"

/**
 * @brief Bring every actuator to a safe state.
 *
 * @note Reached through the abort handler, which is where a failed assertion, a container access
 * out of range or an invalid state id ends up, and through the hard fault handler. The watchdog
 * covers the cases that never get here, such as a hang with interrupts disabled, by resetting the
 * microcontroller. A reset only stops the program, though: the pins float until they are configured
 * again, so whether the drivers are off in between is up to the pull resistors of the board.
 */
static void emergency_stop() {
    micras::hal::Mcu::emergency_stop(micras::emergency_pwm_configs, micras::emergency_gpio_configs);
    micras::hal::Gpio{micras::led_config.gpio}.write(true);
}

static void signal_handler(int signal) {
    if (signal == SIGABRT) {
        emergency_stop();
    }
}

extern "C" {
/**
 * @brief Handler of the hard fault exception, which replaces the generated one.
 *
 * @note The generated handler spins with every output at its last duty cycle until the watchdog
 * resets the microcontroller. This one holds every actuator off while it waits for that reset,
 * which the reset itself does not guarantee. The STM32CubeMX project is set not to generate one.
 */
// NOLINTNEXTLINE(readability-identifier-naming) the name is fixed by the vector table
void HardFault_Handler() {
    emergency_stop();

    while (true) { }
}
}

/**
 * @note The robot is a static local rather than an automatic one, so that its few kilobytes live in
 * .bss instead of on the stack, while it is still constructed after the microcontroller is ready.
 */
int main() {
    std::signal(SIGABRT, signal_handler);

    micras::hal::Mcu::init(micras::mcu_config);
    static micras::Micras micras;

    while (true) {
        micras.update();
    }

    return 0;
}
