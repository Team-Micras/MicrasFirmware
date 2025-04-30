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
    uint8_t execute(uint8_t /*previous_state_id*/) override {
        if (this->micras.calibrate()) {
            return Micras::State::IDLE;
        }

        return Micras::State::WAIT_FOR_CALIBRATE;
    }
};
}  // namespace micras

#endif  // CALIBRATE_STATE_HPP
