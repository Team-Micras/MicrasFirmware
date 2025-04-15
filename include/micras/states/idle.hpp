/**
 * @file
 */

#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class IdleState : public BaseState {
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
        if (this->micras.button_status != proxy::Button::Status::NO_PRESS) {
            this->micras.wall_sensors->turn_on();
        }

        if (this->micras.button_status == proxy::Button::Status::SHORT_PRESS) {
            this->micras.locomotion.enable();
            this->micras.objective = core::Objective::EXPLORE;
            this->micras.grid_pose = this->micras.maze.get_next_goal(this->micras.grid_pose.position, false);
            this->micras.current_action = this->micras.action_queuer.pop();
            return Micras::State::WAIT_FOR_RUN;
        }

        if (this->micras.button_status == proxy::Button::Status::LONG_PRESS) {
            this->micras.locomotion.enable();
            this->micras.objective = core::Objective::SOLVE;

            this->micras.maze_storage.sync("maze", this->micras.maze);

            if (this->micras.dip_switch.get_switch_state(Micras::Switch::FAN)) {
                this->micras.fan.set_speed(50.0F);
            }

            return Micras::State::WAIT_FOR_RUN;
        }

        if (this->micras.button_status == proxy::Button::Status::EXTRA_LONG_PRESS) {
            return Micras::State::WAIT_FOR_CALIBRATE;
        }

        return this->get_id();
    }
};
}  // namespace micras

#endif  // IDLE_STATE_HPP
