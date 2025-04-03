/**
 * @file
 */

#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class IdleState : public core::FSM::State {
public:
    IdleState(uint8_t id, Micras& micras);

    uint8_t run() override;

private:
    Micras& micras;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};
}  // namespace micras

#endif  // IDLE_STATE_HPP
