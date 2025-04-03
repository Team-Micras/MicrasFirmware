/**
 * @file
 */

#ifndef RUN_STATE_HPP
#define RUN_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class RunState : public core::FSM::State {
public:
    RunState(uint8_t id, Micras& micras);

    uint8_t run() override;

private:
    Micras& micras;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

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
};
}  // namespace micras

#endif  // RUN_STATE_HPP
