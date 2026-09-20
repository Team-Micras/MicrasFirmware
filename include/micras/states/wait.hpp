/**
 * @file
 */

#ifndef WAIT_STATE_HPP
#define WAIT_STATE_HPP

#include <cstdint>

#include "micras/proxy/stopwatch.hpp"
#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that gives the user time to let go of the robot before it moves.
 */
class WaitState : public BaseState {
public:
    /**
     * @brief Construct a new WaitState object.
     *
     * @param id The id of the state.
     * @param micras The Micras object.
     * @param next_state The state to go to after the wait.
     * @param wait_time_ms The time to wait in milliseconds.
     */
    WaitState(State id, Micras& micras, State next_state, uint16_t wait_time_ms = 3000);

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

private:
    /**
     * @brief Stopwatch for the wait status.
     */
    proxy::Stopwatch wait_stopwatch;

    /**
     * @brief Id of the state to go after ending the wait.
     */
    uint8_t next_state_id;

    /**
     * @brief Time to wait in milliseconds.
     */
    uint16_t wait_time_ms;
};
}  // namespace micras

#endif  // WAIT_STATE_HPP
