/**
 * @file
 */

#include "micras/core/fsm.hpp"

namespace micras::core {
Fsm::State::State(uint8_t id) : id{id} { }

uint8_t Fsm::State::get_id() const {
    return this->id;
}

Fsm::Fsm(uint8_t initial_state_id) : current_state_id{initial_state_id} { }

void Fsm::add_state(std::unique_ptr<State> state) {
    this->states.emplace(state->get_id(), std::move(state));
}

void Fsm::update() {
    if (this->current_state_id != this->previous_state_id) {
        this->states.at(this->current_state_id)->on_entry();
    }

    const uint8_t next_state = this->states.at(this->current_state_id)->execute();

    this->previous_state_id = this->current_state_id;
    this->current_state_id = next_state;
}
}  // namespace micras::core
