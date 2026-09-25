/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "constants.hpp"
#include "micras/core/types.hpp"
#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/run.hpp"

namespace micras {
void RunState::on_entry() {
    this->micras.start_run();
}

uint8_t RunState::execute() {
    if (this->micras.check_crash()) {
        return std::to_underlying(State::ERROR);
    }

    switch (this->micras.run()) {
        case nav::Mission::Status::RUNNING:
            return this->get_id();

        case nav::Mission::Status::FAILED:
            return std::to_underlying(State::ERROR);

        case nav::Mission::Status::FINISHED:
            break;
    }

    if (this->micras.get_objective() == core::Objective::SOLVE) {
        return std::to_underlying(State::IDLE);
    }

    return std::to_underlying(State::SAVE);
}
}  // namespace micras
