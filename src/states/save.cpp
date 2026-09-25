/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/core/types.hpp"
#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/save.hpp"

namespace micras {
uint8_t SaveState::execute() {
    this->micras.stop();
    this->micras.save_maze();

    if (this->micras.get_objective() == core::Objective::EXPLORE) {
        this->micras.set_objective(core::Objective::RETURN);

        return std::to_underlying(State::WAIT_FOR_RUN);
    }

    this->micras.set_objective(core::Objective::SOLVE);

    return std::to_underlying(State::IDLE);
}
}  // namespace micras
