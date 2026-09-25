/**
 * @file
 */

#ifndef INIT_STATE_HPP
#define INIT_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that checks if every device was initialized.
 */
class InitState : public BaseState {
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

#endif  // INIT_STATE_HPP
