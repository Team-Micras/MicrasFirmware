/**
 * @file
 */

#include <utility>

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"
#include "micras/states/base.hpp"

namespace micras {
BaseState::BaseState(State id, Micras& micras) : FsmState{std::to_underlying(id)}, micras{micras} { }

void BaseState::on_entry() { }
}  // namespace micras
