/**
 * @file
 */

#ifndef BASE_STATE_HPP
#define BASE_STATE_HPP

#include <cstdint>

#include "micras/core/fsm.hpp"

namespace micras {
class Micras;

/**
 * @brief Identifiers of the states of the robot.
 */
enum class State : uint8_t {
    INIT = 0,                  // Initialization of the robot.
    IDLE = 1,                  // Waiting for the user to start the robot.
    WAIT_FOR_RUN = 2,          // Timer for entering the RUN state.
    RUN = 3,                   // Running the main algorithm.
    PLAN = 4,                  // Planning the route of a fast run.
    SAVE = 5,                  // Saving the maze to the non-volatile storage.
    WAIT_FOR_CALIBRATE = 6,    // Timer for entering the CALIBRATE state.
    CALIBRATE = 7,             // Calibrating the wall sensors.
    WAIT_FOR_IDENTIFY = 8,     // Timer for entering the IDENTIFY state.
    IDENTIFY = 9,              // Measuring the constants of the drive train.
    WAIT_FOR_GYROSCOPE = 10,   // Timer for entering the CALIBRATE_GYROSCOPE state.
    CALIBRATE_GYROSCOPE = 11,  // Measuring the scale factor of the gyroscope.
    ERROR = 12,                // Error state.
    NUMBER_OF_STATES = 13
};

/**
 * @brief Base of the states of the robot, which all act on the Micras object.
 */
class BaseState : public core::FsmState {
public:
    /**
     * @brief Construct a new BaseState object.
     *
     * @param id The id of the state.
     * @param micras The Micras object.
     */
    BaseState(State id, Micras& micras);

    /**
     * @brief Do nothing by default.
     */
    void on_entry() override;

protected:
    /**
     * @brief A reference to the Micras object.
     */
    Micras& micras;  // NOLINT(*-non-private-member-variables-in-classes, *-avoid-const-or-ref-data-members)
};
}  // namespace micras

#endif  // BASE_STATE_HPP
