/**
 * @file
 */

#ifndef CALIBRATE_STATE_HPP
#define CALIBRATE_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class CalibrateState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @param previous_state_id The id of the last executed state.
     *
     * @return The id of the next state.
     */
    uint8_t run(uint8_t previous_state_id) override;

private:
    /**
     * @brief Calibrate the robot.
     */
    void calibrate();
};
}  // namespace micras

#endif  // CALIBRATE_STATE_HPP
