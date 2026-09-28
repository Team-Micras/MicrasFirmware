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
     * @note Starts what a press of the button asked for. A press made while the robot was busy is
     * kept until the robot is idle again.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override;

    /**
     * @brief Get ready to explore the maze, as a short press of the button does.
     *
     * @return The id of the state that starts the search.
     */
    uint8_t explore();

    /**
     * @brief Get ready for the fastest run, as a long press of the button does.
     *
     * @return The id of the state that plans the run.
     */
    uint8_t solve();

    /**
     * @brief Choose the maintenance procedure the switches select, as an extra long press does.
     *
     * @return The id of the state that starts the procedure.
     */
    uint8_t calibrate();
};
}  // namespace micras

#endif  // IDLE_STATE_HPP
