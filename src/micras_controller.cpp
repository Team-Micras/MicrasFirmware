/**
 * @file micras_controller.cpp
 *
 * @brief Micras Controller class implementation
 *
 * @date 03/2024
 */

#include "micras/micras_controller.hpp"
#include "target.hpp"

namespace micras {
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
    torque_sensors{torque_sensors_config},
    odometry{rotary_sensor_left, rotary_sensor_right, imu, timer_config, odometry_config},
    mapping{distance_sensors, mapping_config},
    look_at_point{look_at_point_config},
    go_to_point{go_to_point_config},
    current_action{mapping.get_action(odometry_config.initial_pose)} { }

void MicrasController::run() {
    distance_sensors.update();
    fan.update();
    imu.update_data();
    torque_sensors.update();
    odometry.update();

    const micras::nav::State& state = odometry.get_state();
    mapping.update(state.pose);
    nav::Twist command{};

    switch (current_action.type) {
        case micras::nav::Mapping<maze_width, maze_height>::Action::Type::LOOK_AT:
            if (look_at_point.finished(state.pose, current_action.point)) {
                current_action = mapping.get_action(state.pose);
            }

            command = look_at_point.action(state.pose, current_action.point);
            break;

        case micras::nav::Mapping<maze_width, maze_height>::Action::Type::GO_TO:
            if (go_to_point.finished(state, current_action.point)) {
                current_action = mapping.get_action(state.pose);
            }

            command = go_to_point.action(state, current_action.point);
            break;
    }

    locomotion.set_command(command.linear, command.angular);

    // const auto& [linear, angular] = mapping.correct_command(command);
    // locomotion.set_command(linear, angular);
}
}  // namespace micras
