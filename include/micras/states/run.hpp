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

        if (this->micras.current_action->finished(state)) {
            this->micras.odometry.reset();

            if (not this->micras.action_queuer.empty()) {
                this->micras.current_action = this->micras.action_queuer.pop();
            } else {
                const bool returning = (this->micras.objective == core::Objective::RETURN);

                if (this->micras.objective != core::Objective::SOLVE) {
                    observation = this->micras.wall_sensors->get_observation();
                    this->micras.maze.update_walls(this->micras.grid_pose, observation);
                }

                if (this->micras.maze.finished(this->micras.grid_pose.position, returning)) {
                    return true;
                }

                auto next_goal = this->micras.maze.get_next_goal(this->micras.grid_pose.position, returning);

                this->micras.action_queuer.push(this->micras.grid_pose, next_goal.position);
                this->micras.current_action = this->micras.action_queuer.pop();
                this->micras.grid_pose = next_goal;
            }
        }

        auto desired_twist = this->micras.current_action->get_twist(state);

        // desired_twist.angular = this->micras.follow_wall.action(observation, elapsed_time, state.velocity.linear);

        const auto command = this->micras.speed_controller.action(state.velocity, desired_twist, elapsed_time);
        this->micras.locomotion.set_wheel_command(command.first, command.second);

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
};
}  // namespace micras

#endif  // RUN_STATE_HPP
