/**
 * @file
 */

#ifndef RUN_STATE_HPP
#define RUN_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class RunState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @param previous_state_id The id of the last executed state.
     *
     * @return The id of the next state.
     */
    uint8_t run(uint8_t previous_state_id) override;

private:
    /**
     * @brief Run the main algorithm of the robot.
     *
     * @param elapsed_time The elapsed time since the last update.
     * @return true if the robot is still running, false otherwise.
     */
    bool run(float elapsed_time);

    /**
     * @brief Stop the robot.
     */
    void stop();

    /**
     * @brief Timer for aligning the robot to the back wall.
     */
    hal::Timer align_back_timer;
};
}  // namespace micras

#endif  // RUN_STATE_HPP
