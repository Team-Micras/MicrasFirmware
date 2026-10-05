/**
 * @file
 */

#ifndef CHECK_CROSSTALK_STATE_HPP
#define CHECK_CROSSTALK_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that lights the emitters of the wall sensors one at a time, then all and none.
 *
 * @note For measuring from the link how much of each emitter's light reaches each receiver, with
 * the robot still. The mode being lit is published, so that the readings can be told apart.
 */
class CheckCrosstalkState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override;
};
}  // namespace micras

#endif  // CHECK_CROSSTALK_STATE_HPP
