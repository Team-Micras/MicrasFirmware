/**
 * @file
 */

#include <csignal>

#include "micras/micras.hpp"
#include "micras/hal/mcu.hpp"

void signal_handler(int signal) {
    if (signal == SIGABRT) {
        HAL_GPIO_WritePin(micras::led_config.gpio.port, micras::led_config.gpio.pin, GPIO_PinState::GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(
            micras::locomotion_config.left_motor.backwards_pwm.handle,
            micras::locomotion_config.left_motor.backwards_pwm.timer_channel, 0
        );
        __HAL_TIM_SET_COMPARE(
            micras::locomotion_config.left_motor.forward_pwm.handle,
            micras::locomotion_config.left_motor.forward_pwm.timer_channel, 0
        );
        __HAL_TIM_SET_COMPARE(
            micras::locomotion_config.right_motor.backwards_pwm.handle,
            micras::locomotion_config.right_motor.backwards_pwm.timer_channel, 0
        );
        __HAL_TIM_SET_COMPARE(
            micras::locomotion_config.right_motor.forward_pwm.handle,
            micras::locomotion_config.right_motor.forward_pwm.timer_channel, 0
        );

        __HAL_TIM_SET_COMPARE(micras::fan_config.pwm.handle, micras::fan_config.pwm.timer_channel, 0);

        __HAL_TIM_SET_COMPARE(
            micras::wall_sensors_config.led_0_pwm.handle, micras::wall_sensors_config.led_0_pwm.timer_channel, 0
        );
        __HAL_TIM_SET_COMPARE(
            micras::wall_sensors_config.led_1_pwm.handle, micras::wall_sensors_config.led_1_pwm.timer_channel, 0
        );

        __HAL_TIM_SET_COMPARE(micras::buzzer_config.pwm.handle, micras::buzzer_config.pwm.timer_channel, 0);
    }
}

int main() {
    std::signal(SIGABRT, signal_handler);

    micras::hal::Mcu::init();
    micras::Micras micras;

    while (true) {
        micras.update();
    }

    return 0;
}
