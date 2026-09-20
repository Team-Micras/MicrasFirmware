/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/wait.hpp"

namespace micras {
WaitState::WaitState(State id, Micras& micras, State next_state, uint16_t wait_time_ms) :
    BaseState{id, micras}, next_state_id{std::to_underlying(next_state)}, wait_time_ms{wait_time_ms} { }

void WaitState::on_entry() {
    this->wait_stopwatch.reset_ms();
    this->micras.prepare();
}

uint8_t WaitState::execute() {
    this->micras.rest();

    if (this->wait_stopwatch.elapsed_time_ms() > this->wait_time_ms) {
        return this->next_state_id;
    }

    return this->get_id();
}
}  // namespace micras
