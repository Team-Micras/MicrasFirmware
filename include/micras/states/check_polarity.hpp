/**
 * @file
 */

#ifndef CHECK_POLARITY_STATE_HPP
#define CHECK_POLARITY_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that drives each wheel forward and then backward, one wheel at a time.
 *
 * @note For checking from the link that each motor turns its own wheel the way it is told, and that
 * the encoder of that wheel and the gyroscope see it turn that way. Meant for the robot on a stand,
 * with its wheels in the air.
 */
class CheckPolarityState : public BaseState {
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

#endif  // CHECK_POLARITY_STATE_HPP
