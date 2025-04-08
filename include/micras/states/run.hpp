/**
 * @file
 */

#ifndef RUN_STATE_HPP
#define RUN_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class RunState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @param previous_state_id The id of the last executed state.
     *
     * @return The id of the next state.
     */
    uint8_t run(uint8_t /*previous_state_id*/) override {
        if (not this->run(this->micras.elapsed_time)) {
            return this->get_id();
        }

        if (this->micras.current_action.type == nav::Mapping::Action::Type::ERROR) {
            this->stop();
            return Micras::State::ERROR;
        }

        switch (this->micras.objective) {
            case core::Objective::EXPLORE:
                this->micras.objective = core::Objective::RETURN;
                return Micras::State::WAIT_FOR_RUN;

            case core::Objective::RETURN:
                this->micras.objective = core::Objective::SOLVE;
                this->micras.maze_storage.create("maze", this->micras.mapping);
                this->micras.maze_storage.save();
                this->stop();
                return Micras::State::IDLE;

            case core::Objective::SOLVE:
                this->stop();
                return Micras::State::IDLE;
        }

        return Micras::State::ERROR;
    }

private:
    /**
     * @brief Run the main algorithm of the robot.
     *
     * @param elapsed_time The elapsed time since the last update.
     * @return true if the robot is still running, false otherwise.
     */
    bool run(float elapsed_time) {
        this->micras.odometry.update(elapsed_time);

        micras::nav::State state = this->micras.odometry.get_state();

        if (this->micras.objective != core::Objective::SOLVE) {
            this->micras.mapping.update(state.pose);
        }

        nav::Twist command{};

        const nav::State relative_state = {
            {state.pose.position.rotate(this->micras.current_action.direction),
             core::assert_angle(
                 state.pose.orientation + std::numbers::pi_v<float> / 4.0F * (2 - this->micras.current_action.direction)
             )},
            state.velocity
        };

        core::FollowWallType follow_wall_type = this->micras.mapping.get_follow_wall_type(state.pose);

        const bool stop = (this->micras.objective != core::Objective::SOLVE) or
                          not this->micras.dip_switch.get_switch_state(Micras::Switch::STOP);

        switch (this->micras.current_action.type) {
            case nav::Mapping::Action::Type::LOOK_AT:
                if (this->micras.look_at_point.finished(relative_state, this->micras.current_action.point)) {
                    this->micras.current_action = this->micras.mapping.get_action(state.pose, this->micras.objective);

                    if (this->micras.current_action.type == nav::Mapping::Action::Type::GO_TO and
                        this->micras.mapping.can_align_back(state.pose) and
                        this->micras.objective != core::Objective::SOLVE) {
                        this->micras.current_action.type = nav::Mapping::Action::Type::ALIGN_BACK;
                        this->align_back_timer.reset_ms();
                    }

                    this->micras.look_at_point.reset();
                    return false;
                }

                command =
                    this->micras.look_at_point.action(relative_state, this->micras.current_action.point, elapsed_time);
                break;

            case nav::Mapping::Action::Type::GO_TO:
                if (this->micras.go_to_point.finished(relative_state, this->micras.current_action.point, stop)) {
                    this->micras.current_action = this->micras.mapping.get_action(state.pose, this->micras.objective);
                    this->micras.go_to_point.reset();

                    return false;
                }

                if (this->micras.current_action.direction % 2 == 1) {
                    follow_wall_type = core::FollowWallType::NONE;
                } else {
                    state.pose = this->micras.mapping.correct_pose(state.pose, follow_wall_type);
                    this->micras.odometry.set_state(state);
                }

                command = this->micras.go_to_point.action(
                    relative_state, this->micras.current_action.point, follow_wall_type, elapsed_time, stop
                );

                break;
            case nav::Mapping::Action::Type::ALIGN_BACK:
                command = {-5.0F, 0.0F};

                if (this->align_back_timer.elapsed_time_ms() > 500) {
                    state.pose = this->micras.mapping.correct_pose(state.pose, core::FollowWallType::BACK);
                    this->micras.odometry.set_state(state);
                    this->micras.current_action = this->micras.mapping.get_action(state.pose, this->micras.objective);
                    return false;
                }
                break;
            default:
                this->micras.locomotion.stop();
                return true;
        }

        if (this->micras.current_action.type == nav::Mapping::Action::Type::LOOK_AT) {
            this->micras.argb.set_color(proxy::Argb::Colors::blue);
        } else if (this->micras.current_action.type == nav::Mapping::Action::Type::GO_TO) {
            switch (follow_wall_type) {
                case core::FollowWallType::NONE:
                    this->micras.argb.set_color(proxy::Argb::Colors::magenta);
                    break;

                case core::FollowWallType::FRONT:
                    this->micras.argb.set_color(proxy::Argb::Colors::white);
                    break;

                case core::FollowWallType::LEFT:
                    this->micras.argb.set_color(proxy::Argb::Colors::green);
                    break;

                case core::FollowWallType::RIGHT:
                    this->micras.argb.set_color(proxy::Argb::Colors::red);
                    break;

                case core::FollowWallType::PARALLEL:
                    this->micras.argb.set_color(proxy::Argb::Colors::yellow);
                    break;
                default:
                    break;
            }
        } else {
            this->micras.argb.set_color(proxy::Argb::Colors::cyan);
        }

        this->micras.locomotion.set_command(command.linear, command.angular);
        return false;
    }

    /**
     * @brief Stop the robot.
     */
    void stop() {
        this->micras.wall_sensors->turn_off();
        this->micras.locomotion.disable();
        this->micras.fan.stop();
    }

    /**
     * @brief Timer for aligning the robot to the back wall.
     */
    hal::Timer align_back_timer;
};
}  // namespace micras

#endif  // RUN_STATE_HPP
