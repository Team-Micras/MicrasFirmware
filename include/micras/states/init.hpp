/**
 * @file
 */

#ifndef INIT_STATE_HPP
#define INIT_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class InitState : public core::FSM::State {
public:
    InitState(uint8_t id, Micras& micras);

    uint8_t run() override;

private:
    Micras& micras;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};
}  // namespace micras

#endif  // INIT_STATE_HPP
