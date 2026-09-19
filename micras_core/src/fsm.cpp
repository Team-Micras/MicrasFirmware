/**
 * @file
 */

#ifndef MICRAS_CORE_FSM_CPP
#define MICRAS_CORE_FSM_CPP

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <utility>

#include "micras/core/fsm.hpp"

namespace micras::core {
template <uint8_t num_of_states>
TFsm<num_of_states>::TFsm(uint8_t initial_state_id) : current_state_id{initial_state_id} { }

template <uint8_t num_of_states>
void TFsm<num_of_states>::add_state(std::unique_ptr<FsmState> state) {
    this->states.at(state->get_id()) = std::move(state);
}

template <uint8_t num_of_states>
void TFsm<num_of_states>::update() {
    if (this->current_state_id >= num_of_states or this->states.at(this->current_state_id) == nullptr) {
        std::abort();
    }

    FsmState& state = *this->states.at(this->current_state_id);

    if (this->current_state_id != this->previous_state_id) {
        state.on_entry();
    }

    const uint8_t next_state_id = state.execute();

    this->previous_state_id = this->current_state_id;
    this->current_state_id = next_state_id;
}
}  // namespace micras::core

#endif  // MICRAS_CORE_FSM_CPP
