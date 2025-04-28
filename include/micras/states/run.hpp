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

        switch (this->micras.objective) {
            case core::Objective::EXPLORE:
                this->micras.objective = core::Objective::RETURN;
                return Micras::State::WAIT_FOR_RUN;

            case core::Objective::RETURN:
                this->micras.objective = core::Objective::SOLVE;
                this->micras.maze_storage.create("maze", this->micras.maze);
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
     * @return True if the robot is still running, false otherwise.
     */
    bool run(float elapsed_time) {
        this->micras.odometry.update(elapsed_time);

        const micras::nav::State& state = this->micras.odometry.get_state();
        core::Observation         observation{};

        if (this->micras.current_action->finished(this->micras.action_pose.get())) {
            if (this->finished) {
                this->finished = false;
                return true;
            }

            this->micras.speed_controller.reset();
            this->micras.action_pose.reset_reference();

            if (not this->micras.action_queuer.empty()) {
                this->micras.current_action = this->micras.action_queuer.pop();
            } else {
                const bool returning = (this->micras.objective == core::Objective::RETURN);
                const bool solving = (this->micras.objective == core::Objective::SOLVE);

                if (not solving) {
                    observation = this->micras.wall_sensors->get_observation();
                    this->micras.maze.update_walls(this->micras.grid_pose, observation);
                }

                micras::nav::GridPose next_goal{};

                if (solving or this->micras.maze.finished(this->micras.grid_pose.position, returning)) {
                    this->finished = true;
                    next_goal = this->micras.grid_pose.turned_back().front();
                } else {
                    if (returning) {
                        this->micras.maze.compute_best_route();
                    }

                    next_goal = this->micras.maze.get_next_goal(this->micras.grid_pose.position, returning);
                }

                this->micras.action_queuer.push(this->micras.grid_pose, next_goal.position);
                this->micras.current_action = this->micras.action_queuer.pop();
                this->micras.grid_pose = next_goal;
            }

            if (this->micras.current_action->allow_follow_wall()) {
                this->micras.follow_wall.reset();
            }
        }

        auto desired_twist = this->micras.current_action->get_twist(this->micras.action_pose.get());

        if (this->micras.current_action->allow_follow_wall()) {
            desired_twist.angular =
                this->micras.follow_wall.compute_angular_correction(elapsed_time, state.velocity.linear);
        }

        const auto [left_command, right_command] =
            this->micras.speed_controller.compute_commands(state.velocity, desired_twist, elapsed_time);
        this->micras.locomotion.set_wheel_command(left_command, right_command);

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
     * @brief Stopwatch for aligning the robot to the back wall.
     */
    proxy::Stopwatch align_back_stopwatch;

    /**
     * @brief Flag for when the robot has finished an objective.
     */
    bool finished{false};
};
}  // namespace micras

#endif  // RUN_STATE_HPP
