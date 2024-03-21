/**
 * @file micras_controller.cpp
 *
 * @brief Micras Controller class implementation
 *
 * @date 03/2024
 */

#include "controller/micras_controller.hpp"
#include "target.hpp"

MicrasController::MicrasController() : button{button_config}, buzzer{buzzer_config},
    current_sensors{current_sensors_config},
    dip_switch{dip_switch_config}, distance_sensors{distance_sensors_config}, fan{fan_config}, imu{imu_config},
    led{led_config}, locomotion{locomotion_config} {
}

void MicrasController::run() {
    if (button.get_status() == proxy::Button::SHORT_PRESS) {
        led.turn_on();
    } else {
        led.turn_off();
    }
}
