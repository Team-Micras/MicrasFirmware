/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/check_crosstalk.hpp"

namespace micras {
void CheckCrosstalkState::on_entry() {
    this->micras.start_crosstalk_check();
}

uint8_t CheckCrosstalkState::execute() {
    if (this->micras.is_stop_requested() or this->micras.check_crosstalk()) {
        return std::to_underlying(State::IDLE);
    }

    return this->get_id();
}
}  // namespace micras
