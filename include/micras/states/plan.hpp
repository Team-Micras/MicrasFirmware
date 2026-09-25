/**
 * @file
 */

#ifndef PLAN_STATE_HPP
#define PLAN_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that plans the route of a fast run, with the motors stopped.
 */
class PlanState : public BaseState {
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

#endif  // PLAN_STATE_HPP
