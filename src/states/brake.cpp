/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/brake.hpp"

namespace micras {
BrakeState::BrakeState(State id, Micras& micras, uint16_t timeout_ms) :
    BaseState{id, micras}, timeout_ms{timeout_ms} { }

void BrakeState::on_entry() {
    this->stopwatch.reset_ms();
    this->micras.start_brake();
}

uint8_t BrakeState::execute() {
    if (this->micras.check_fault()) {
        return std::to_underlying(State::ERROR);
    }

    if (this->micras.brake() or this->stopwatch.elapsed_time_ms() > this->timeout_ms) {
        return std::to_underlying(State::IDLE);
    }

    return this->get_id();
}
}  // namespace micras
