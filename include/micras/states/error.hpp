/**
 * @file
 */

#ifndef ERROR_STATE_HPP
#define ERROR_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class ErrorState : public core::FSM::State {
public:
    ErrorState(uint8_t id, Micras& micras);

    uint8_t run() override;

private:
    Micras& micras;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};
}  // namespace micras

#endif  // ERROR_STATE_HPP
