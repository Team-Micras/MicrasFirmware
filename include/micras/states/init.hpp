/**
 * @file
 */

#ifndef INIT_STATE_HPP
#define INIT_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class InitState : public BaseState {
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
        if (not this->micras.check_initialization()) {
            return Micras::State::ERROR;
        }

        return Micras::State::IDLE;
    }
};
}  // namespace micras

#endif  // INIT_STATE_HPP
