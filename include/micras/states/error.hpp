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
     * @brief Execute the entry function of this state.
     */
    void on_entry() override {
        this->micras.stop();
        this->micras.send_event(Interface::Event::ERROR);
    }

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override { return this->get_id(); }
};
}  // namespace micras

#endif  // ERROR_STATE_HPP
