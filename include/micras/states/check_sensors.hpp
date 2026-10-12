/**
 * @file
 */

#ifndef CHECK_SENSORS_STATE_HPP
#define CHECK_SENSORS_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that keeps every sensor on with the robot still, until the link stops it.
 *
 * @note For checking the sensors from the link: the wall sensors are off while the robot is idle.
 */
class CheckSensorsState : public BaseState {
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

#endif  // CHECK_SENSORS_STATE_HPP
