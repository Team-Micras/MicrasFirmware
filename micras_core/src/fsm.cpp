/**
 * @file
 */

#include "micras/core/fsm.hpp"

namespace micras::core {
FSM::State::State(uint8_t id) : id{id} { }

uint8_t FSM::State::run() {
    return this->get_id();
}

uint8_t FSM::State::get_id() const {
    return this->id;
}

FSM::FSM(uint8_t initial_state_id) : current_state_id{initial_state_id} { }

void FSM::add_state(const State& state) {
    this->states.emplace(state.get_id(), state);
}

void FSM::run() {
    this->current_state_id = this->states.at(this->current_state_id).run();
}
}  // namespace micras::core
