/**
 * @file
 */

#ifndef WAIT_STATE_HPP
#define WAIT_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class WaitState : public core::FSM::State {
public:
    WaitState(uint8_t id, Micras& micras, uint8_t next_state_id, uint16_t wait_time_ms = 3000);

    uint8_t run() override;

private:
    Micras& micras;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    uint8_t next_state_id;

    uint16_t wait_time_ms;
};
}  // namespace micras

#endif  // WAIT_STATE_HPP
