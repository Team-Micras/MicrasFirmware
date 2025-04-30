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
    uint8_t execute(uint8_t /*previous_state_id*/) override {
        if (not this->micras.run(this->micras.elapsed_time)) {
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
                this->micras.stop();
                return Micras::State::IDLE;

            case core::Objective::SOLVE:
                this->micras.stop();
                return Micras::State::IDLE;
        }

        return Micras::State::ERROR;
    }
};
}  // namespace micras

#endif  // RUN_STATE_HPP
