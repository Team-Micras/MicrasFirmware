/**
 * @file
 */

#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that waits for the user to choose what to do.
 */
class IdleState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override;
};
}  // namespace micras

#endif  // IDLE_STATE_HPP
