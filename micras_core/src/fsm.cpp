/**
 * @file
 */

#include "micras/core/fsm.hpp"

namespace micras::core {
FSM::State::State(uint8_t id) : id{id} { }

uint8_t FSM::State::get_id() const {
    return this->id;
}

FSM::FSM(uint8_t initial_state_id) : current_state_id{initial_state_id} { }

void FSM::add_state(std::unique_ptr<State> state) {
    this->states.emplace(state->get_id(), std::move(state));
}

void FSM::run() {
    const uint8_t next_state = this->states.at(this->current_state_id)->run(this->previous_state_id);
    this->previous_state_id = this->current_state_id;
    this->current_state_id = next_state;
}
}  // namespace micras::core
