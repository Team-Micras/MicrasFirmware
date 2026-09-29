/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/init.hpp"

namespace micras {
uint8_t InitState::execute() {
    if (not this->micras.check_initialization()) {
        this->micras.record_initialization_fault();
        return std::to_underlying(State::ERROR);
    }

    this->micras.place_at_start();
    return std::to_underlying(State::IDLE);
}
}  // namespace micras
