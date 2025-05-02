/**
 * @file
 */

#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class IdleState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override {
        this->micras.stop();
        this->micras.reset();
    }

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override {
        if (this->micras.acknowledge_event(Interface::Event::EXPLORE)) {
            this->micras.set_objective(core::Objective::EXPLORE);

            return Micras::State::WAIT_FOR_RUN;
        }

        if (this->micras.acknowledge_event(Interface::Event::SOLVE)) {
            this->micras.set_objective(core::Objective::SOLVE);
            this->micras.load_best_route();

            return Micras::State::WAIT_FOR_RUN;
        }

        if (this->micras.acknowledge_event(Interface::Event::CALIBRATE)) {
            return Micras::State::WAIT_FOR_CALIBRATE;
        }

        return this->get_id();
    }
};
}  // namespace micras

#endif  // IDLE_STATE_HPP
