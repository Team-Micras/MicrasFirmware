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
    uint8_t run(uint8_t previous_state_id) override;
};
}  // namespace micras

#endif  // IDLE_STATE_HPP
