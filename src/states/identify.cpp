/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/identify.hpp"

namespace micras {
void IdentifyState::on_entry() {
    this->micras.start_identification();
}

uint8_t IdentifyState::execute() {
    if (this->micras.check_fault()) {
        return std::to_underlying(State::ERROR);
    }

    return this->micras.identify() ? std::to_underlying(State::IDLE) : this->get_id();
}
}  // namespace micras
