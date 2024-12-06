/**
 * @file
 */

#include "micras/micras_controller.hpp"
#include "target.hpp"

namespace micras {
MicrasController::MicrasController() :
    loop_timer{timer_config},
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
    maze_storage{maze_storage_config},
    odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config},
    mapping{wall_sensors, mapping_config},
    look_at_point{look_at_point_config},
    go_to_point{wall_sensors, go_to_point_config, follow_wall_config} { }

void MicrasController::update() {
    float elapsed_time = loop_timer.elapsed_time_us() / 1000000.0F;
    loop_timer.reset_us();

    auto button_status = button.get_status();

    this->wall_sensors.update();
    this->buzzer.update();
    this->imu.update();
    this->fan.update();

    switch (this->status) {
        case Status::INIT:
            if (not this->imu.check_whoami()) {
                this->status = Status::ERROR;
            }

            this->status = Status::IDLE;
            break;

        case Status::IDLE:
            if (button_status != proxy::Button::Status::NO_PRESS) {
                this->status = Status::WAIT;
                this->wait_timer.reset_ms();
                this->wall_sensors.turn_on();
            }

            if (button_status == proxy::Button::Status::SHORT_PRESS) {
                this->locomotion.enable();
                this->objective = core::Objective::EXPLORE;
                this->next_status = Status::RUN;
            }

            if (button_status == proxy::Button::Status::LONG_PRESS) {
                this->locomotion.enable();
                this->objective = core::Objective::SOLVE;
                this->next_status = Status::RUN;
                this->maze_storage.sync("maze", this->mapping);

                if (this->dip_switch.get_switch_state(Switch::DIAGONAL)) {
                    this->mapping.diagonalize_best_route();
                }

                if (this->dip_switch.get_switch_state(Switch::FAN)) {
                    this->fan.set_speed(50.0F);
                }
            }

            if (button_status == proxy::Button::Status::EXTRA_LONG_PRESS) {
                this->next_status = Status::CALIBRATE;
            }

            break;

        case Status::WAIT:
            if (this->wait_timer.elapsed_time_ms() > 3000) {
                this->status = this->next_status;
                this->odometry.reset();
                this->look_at_point.reset();
                this->go_to_point.reset();
                this->imu.calibrate();
                this->current_action = this->mapping.get_action(this->odometry.get_state().pose, this->objective);
            }

            break;

        case Status::RUN:
            if (this->run(elapsed_time)) {
                if (this->current_action.type == nav::Mapping<maze_width, maze_height>::Action::Type::ERROR) {
                    this->status = Status::ERROR;
                    this->stop();
                    break;
                }

                switch (this->objective) {
                    case core::Objective::EXPLORE:
                        this->status = Status::WAIT;
                        this->wait_timer.reset_ms();
                        this->objective = core::Objective::RETURN;
                        break;

                    case core::Objective::RETURN:
                        this->objective = core::Objective::SOLVE;
                        this->maze_storage.create("maze", this->mapping);
                        this->maze_storage.save();
                        this->stop();
                        break;

                    default:
                        this->status = Status::IDLE;
                        this->stop();
                        break;
                }
            }

            break;

        case Status::CALIBRATE:
            if (this->calibration_type == CalibrationType::RIGHT_FREE_SPACE) {
                this->status = Status::IDLE;
            } else {
                this->status = Status::WAIT;
                this->wait_timer.reset_ms();
            }

            this->calibrate();
            break;

        case Status::ERROR:
            break;

        default:
            this->status = Status::ERROR;
            break;
    }

    while (loop_timer.elapsed_time_us() < loop_time_us) { }
}

bool MicrasController::run(float elapsed_time) {
    this->odometry.update(elapsed_time);

    micras::nav::State state = this->odometry.get_state();

    if (this->objective != core::Objective::SOLVE) {
        this->mapping.update(state.pose);
    }

    nav::Twist command{};

    nav::State relative_state = {
        {state.pose.position.rotate(this->current_action.direction),
         core::assert_angle(
             state.pose.orientation + std::numbers::pi_v<float> / 4.0F * (2 - this->current_action.direction)
         )},
        state.velocity
    };

    core::FollowWallType follow_wall_type = this->mapping.get_follow_wall_type(state.pose);

    bool stop = (this->objective != core::Objective::SOLVE) or not this->dip_switch.get_switch_state(Switch::STOP);

    switch (this->current_action.type) {
        case nav::Mapping<maze_width, maze_height>::Action::Type::LOOK_AT:
            if (this->look_at_point.finished(relative_state, this->current_action.point)) {
                this->current_action = this->mapping.get_action(state.pose, this->objective);

                if (this->current_action.type == nav::Mapping<maze_width, maze_height>::Action::Type::GO_TO and
                    this->mapping.can_align_back(state.pose) and this->objective != core::Objective::SOLVE) {
                    this->current_action.type = nav::Mapping<maze_width, maze_height>::Action::Type::ALIGN_BACK;
                    this->align_back_timer.reset_ms();
                }

                this->look_at_point.reset();
                return false;
            }

            command = this->look_at_point.action(relative_state, this->current_action.point, elapsed_time);
            break;

        case nav::Mapping<maze_width, maze_height>::Action::Type::GO_TO:
            if (this->go_to_point.finished(relative_state, this->current_action.point, stop)) {
                this->current_action = this->mapping.get_action(state.pose, this->objective);
                this->go_to_point.reset();

                return false;
            }

            if (this->current_action.direction % 2 == 1) {
                follow_wall_type = core::FollowWallType::NONE;
            } else {
                state.pose = this->mapping.correct_pose(state.pose, follow_wall_type);
                this->odometry.set_state(state);
            }

            command = this->go_to_point.action(
                relative_state, this->current_action.point, follow_wall_type, elapsed_time, stop
            );

            break;
        case nav::Mapping<maze_width, maze_height>::Action::Type::ALIGN_BACK:
            command = {-5.0F, 0.0F};

            if (this->align_back_timer.elapsed_time_ms() > 500) {
                state.pose = this->mapping.correct_pose(state.pose, core::FollowWallType::BACK);
                this->odometry.set_state(state);
                this->current_action = this->mapping.get_action(state.pose, this->objective);
                return false;
            }
            break;
        default:
            locomotion.stop();
            return true;
    }

    if (this->current_action.type == nav::Mapping<maze_width, maze_height>::Action::Type::LOOK_AT) {
        this->argb.set_color({0, 0, 255});
    } else if (this->current_action.type == nav::Mapping<maze_width, maze_height>::Action::Type::GO_TO) {
        switch (follow_wall_type) {
            case core::FollowWallType::NONE:
                this->argb.set_color({255, 0, 255});
                break;

            case core::FollowWallType::FRONT:
                this->argb.set_color({255, 255, 255});
                break;

            case core::FollowWallType::LEFT:
                this->argb.set_color({0, 255, 0});
                break;

            case core::FollowWallType::RIGHT:
                this->argb.set_color({255, 0, 0});
                break;

            case core::FollowWallType::PARALLEL:
                this->argb.set_color({255, 255, 0});
                break;
            default:
                break;
        }
    } else {
        this->argb.set_color({0, 255, 255});
    }

    locomotion.set_command(command.linear, command.angular);
    return false;
}

void MicrasController::calibrate() {
    switch (this->calibration_type) {
        case CalibrationType::SIDE_WALLS:
            this->go_to_point.calibrate();
            this->mapping.calibrate_side();
            this->wall_sensors.calibrate_left_wall();
            this->wall_sensors.calibrate_right_wall();
            this->wall_sensors.calibrate_front_free_space();
            this->calibration_type = CalibrationType::FRONT_WALL;
            break;

        case CalibrationType::FRONT_WALL:
            this->mapping.calibrate_front();
            this->wall_sensors.calibrate_front_wall();
            this->calibration_type = CalibrationType::LEFT_FREE_SPACE;
            break;

        case CalibrationType::LEFT_FREE_SPACE:
            this->wall_sensors.calibrate_left_free_space();
            this->calibration_type = CalibrationType::RIGHT_FREE_SPACE;
            break;

        case CalibrationType::RIGHT_FREE_SPACE:
            this->wall_sensors.calibrate_right_free_space();
            this->wall_sensors.update_thresholds();
            this->wall_sensors.turn_off();
            this->calibration_type = CalibrationType::SIDE_WALLS;
            break;
    }
}

void MicrasController::stop() {
    this->wall_sensors.turn_off();
    this->locomotion.disable();
    this->fan.stop();
}
}  // namespace micras
