/**
 * @file
 */

#ifndef ERROR_STATE_HPP
#define ERROR_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class ErrorState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @param previous_state_id The id of the last executed state.
     *
     * @return The id of the next state.
     */
    uint8_t run(uint8_t /*previous_state_id*/) override {
        this->micras.led.turn_on();

        return this->get_id();
    }
};
}  // namespace micras

#endif  // ERROR_STATE_HPP
