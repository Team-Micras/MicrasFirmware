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
    timer{timer_config},
    argb{argb_config},
    battery{battery_config},
    button{button_config},
    buzzer{buzzer_config},
    dip_switch{dip_switch_config},
    wall_sensors{wall_sensors_config},
    fan{fan_config},
    imu{imu_config},
    led{led_config},
    locomotion{locomotion_config},
    rotary_sensor_left{rotary_sensor_left_config},
    rotary_sensor_right{rotary_sensor_right_config},
    // torque_sensors{torque_sensors_config},
    odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config},
    mapping{wall_sensors, mapping_config},
    look_at_point{look_at_point_config},
    go_to_point{wall_sensors, go_to_point_config, follow_wall_config},
    current_action{mapping.get_action(odometry_config.initial_pose)},
    interface{buzzer, argb, led, go_to_point, look_at_point, mapping, odometry, wall_sensors} {
    this->wall_sensors.turn_on();
}

void MicrasController::run() {
    float elapsed_time = timer.elapsed_time_us() / 1000000.0F;
    timer.reset_us();
    auto button_status = button.get_status();
    this->wall_sensors.update();

    buzzer.update();

    if (button_status == proxy::Button::Status::SHORT_PRESS) {
        this->argb.set_color({255, 173, 0});
        hal::Timer::sleep_ms(3000);
        this->go_to_point.calibrate();
        this->mapping.calibrate_side();
        this->wall_sensors.calibrate_left_wall();
        this->wall_sensors.calibrate_right_wall();
        this->wall_sensors.calibrate_front_free_space();

        this->argb.set_color({0, 255, 46});
        hal::Timer::sleep_ms(3000);
        this->mapping.calibrate_front();
        this->wall_sensors.calibrate_front_wall();

        this->argb.set_color({0, 82, 255});
        hal::Timer::sleep_ms(3000);
        this->mapping.calibrate_front();
        this->wall_sensors.calibrate_right_free_space();

        this->argb.set_color({255, 0, 209});
        hal::Timer::sleep_ms(3000);
        this->mapping.calibrate_front();
        this->wall_sensors.calibrate_left_free_space();

        this->wall_sensors.update_thresholds();
        this->argb.set_color({0, 0, 0});

        return;
    }

    if (button_status == proxy::Button::Status::LONG_PRESS) {
        locomotion.enable();
        this->argb.set_color({0, 0, 255});
        this->argb.set_color({0, 255, 0});
        hal::Timer::sleep_ms(3000);
        this->started = true;
        this->odometry.reset();
        this->look_at_point.reset();
        this->go_to_point.reset();
        return;
    }

    if (not this->started) {
        return;
    }

    this->fan.update();
    this->imu.update();
    this->odometry.update(elapsed_time);

    micras::nav::State state = odometry.get_state();
    this->mapping.update(state.pose);

    bool can_follow_wall = this->mapping.can_follow_wall(state.pose);

    nav::Twist command{};

    nav::State relative_state = {
        state.pose.position.rotate(current_action.side),
        core::assert_angle(state.pose.orientation + std::numbers::pi_v<float> / 2.0F * (1 - current_action.side)),
        state.velocity
    };

    switch (current_action.type) {
        case micras::nav::Mapping<maze_width, maze_height>::Action::Type::LOOK_AT:
            if (this->look_at_point.finished(relative_state, current_action.point)) {
                current_action = this->mapping.get_action(state.pose);
                this->look_at_point.reset();
                return;
            }

            this->argb.set_color({0, 0, 255});

            command = this->look_at_point.action(relative_state, current_action.point, elapsed_time);
            break;

        case micras::nav::Mapping<maze_width, maze_height>::Action::Type::GO_TO:
            if (can_follow_wall) {
                this->argb.set_color({0, 255, 0});
            } else {
                this->argb.set_color({255, 0, 0});
            }

            if (this->go_to_point.finished(relative_state, current_action.point)) {
                current_action = this->mapping.get_action(state.pose);
                this->go_to_point.reset();
                return;
            }

            command = this->go_to_point.action(relative_state, current_action.point, can_follow_wall, elapsed_time);

            state.pose = this->mapping.correct_pose(state.pose, can_follow_wall);
            this->odometry.set_state(state);
            break;
    }

    locomotion.set_command(command.linear, command.angular);

    while (timer.elapsed_time_us() < loop_time_us) { }
}
}  // namespace micras
