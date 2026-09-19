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
 * out of range or an invalid state id ends up. The watchdog covers the cases that never get here,
 * such as a fault handler that spins with interrupts disabled: it resets the microcontroller, and a
 * reset returns every driver enable pin and every PWM output to its inactive state.
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

int main() {
    std::signal(SIGABRT, signal_handler);

    micras::hal::Mcu::init(micras::mcu_config);
    micras::Micras micras;

    while (true) {
        micras.update();
    }

    return 0;
}
