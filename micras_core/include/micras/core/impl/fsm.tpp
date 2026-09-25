/**
 * @file
 */

#ifndef MICRAS_CORE_FSM_TPP
#define MICRAS_CORE_FSM_TPP

#include <cstdint>
#include <cstdlib>

namespace micras::core {
template <uint8_t num_of_states>
TFsm<num_of_states>::TFsm(uint8_t initial_state_id) : current_state_id{initial_state_id} { }

template <uint8_t num_of_states>
void TFsm<num_of_states>::add_state(FsmState& state) {
    this->states.at(state.get_id()) = &state;
}

template <uint8_t num_of_states>
uint8_t TFsm<num_of_states>::get_current_state_id() const {
    return this->current_state_id;
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

#endif  // MICRAS_CORE_FSM_TPP
