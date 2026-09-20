/**
 * @file
 */

#ifndef SAVE_STATE_HPP
#define SAVE_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that writes the maze to the non-volatile storage, with the motors stopped.
 */
class SaveState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override;
};
}  // namespace micras

#endif  // SAVE_STATE_HPP
