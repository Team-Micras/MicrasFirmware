/**
 * @file micras_controller.cpp
 *
 * @brief Micras Controller class implementation
 *
 * @date 03/2024
 */

#include "controller/micras_controller.hpp"
#include "target.hpp"

MicrasController::MicrasController() :
    argb{argb_config},
    battery{battery_config},
    button{button_config},
    buzzer{buzzer_config},
    dip_switch{dip_switch_config},
    distance_sensors{distance_sensors_config},
    fan{fan_config},
    imu{imu_config},
    led{led_config},
    locomotion{locomotion_config},
    rotary_sensor_left{rotary_sensor_left_config},
    rotary_sensor_right{rotary_sensor_right_config},
    torque_sensors{torque_sensors_config} { }

void MicrasController::run() {
    if (button.is_pressed()) {
        led.turn_on();
    } else {
        led.turn_off();
    }
}
