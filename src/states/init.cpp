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
        return std::to_underlying(State::ERROR);
    }

    return std::to_underlying(State::IDLE);
}
}  // namespace micras
