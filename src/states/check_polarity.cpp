/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/check_polarity.hpp"

namespace micras {
void CheckPolarityState::on_entry() {
    this->micras.start_polarity_check();
}

uint8_t CheckPolarityState::execute() {
    if (this->micras.is_stop_requested() or this->micras.check_polarity()) {
        return std::to_underlying(State::IDLE);
    }

    return this->get_id();
}
}  // namespace micras
