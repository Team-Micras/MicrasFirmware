/**
 * @file
 */

#ifndef ERROR_STATE_HPP
#define ERROR_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that keeps the robot stopped after something went wrong.
 */
class ErrorState : public BaseState {
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

#endif  // ERROR_STATE_HPP
