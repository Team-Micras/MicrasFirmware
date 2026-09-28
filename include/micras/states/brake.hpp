/**
 * @file
 */

#ifndef BRAKE_STATE_HPP
#define BRAKE_STATE_HPP

#include <cstdint>

#include "micras/proxy/stopwatch.hpp"
#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that brings a moving robot to a standstill after a stop, before it is idle.
 *
 * @note The faults are still watched while braking. A brake that has not ended by the timeout
 * gives up and leaves the robot to coast, which the idle state does by disabling the drivers. The
 * presses of the button while braking are forgotten once it ends, so the idle state never starts a
 * run with a press made during the stop.
 */
class BrakeState : public BaseState {
public:
    /**
     * @brief Construct a new BrakeState object.
     *
     * @param id The id of the state.
     * @param micras The Micras object.
     * @param timeout_ms The longest the brake may take, in milliseconds.
     */
    BrakeState(State id, Micras& micras, uint16_t timeout_ms);

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
     * @brief Stopwatch for the timeout.
     */
    proxy::Stopwatch stopwatch;

    /**
     * @brief The longest the brake may take, in milliseconds.
     */
    uint16_t timeout_ms;
};
}  // namespace micras

#endif  // BRAKE_STATE_HPP
