/**
 * @file
 */

#ifndef BASE_STATE_HPP
#define BASE_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class BaseState : public core::FSM::State {
public:
    /**
     * @brief Construct a new BaseState object.
     *
     * @param id The id of the state.
     * @param micras The Micras object.
     */
    BaseState(uint8_t id, Micras& micras) : State(id), micras{micras} {};

    /**
     * @brief Do nothing by default.
     */
    void on_entry() override { }

protected:
    /**
     * @brief A reference to the Micras object.
     */
    Micras& micras;  // NOLINT(*-non-private-member-variables-in-classes, *-avoid-const-or-ref-data-members)
};
}  // namespace micras

#endif  // BASE_STATE_HPP
