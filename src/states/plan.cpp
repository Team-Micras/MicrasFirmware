/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/plan.hpp"

namespace micras {
void PlanState::on_entry() {
    this->micras.start_plan();
}

uint8_t PlanState::execute() {
    if (not this->micras.plan()) {
        return this->get_id();
    }

    return std::to_underlying(this->micras.has_route() ? State::WAIT_FOR_RUN : State::ERROR);
}
}  // namespace micras
