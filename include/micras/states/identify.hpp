/**
 * @file
 */

#ifndef IDENTIFY_STATE_HPP
#define IDENTIFY_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that measures the constants of the drive train.
 */
class IdentifyState : public BaseState {
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

#endif  // IDENTIFY_STATE_HPP
