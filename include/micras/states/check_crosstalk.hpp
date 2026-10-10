/**
 * @file
 */

#ifndef CHECK_CROSSTALK_STATE_HPP
#define CHECK_CROSSTALK_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that publishes how much of each emitter's light reaches each receiver, one emitter
 * at a time, after a mode with every emitter off and before one with the readings as they are.
 *
 * @note For measuring from the link, with the robot still. The emitters take turns anyway, so in
 * the mode of an emitter the intensity of each receiver is read from the scan where that emitter
 * alone is lit. The mode is published, so that the readings can be told apart.
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
