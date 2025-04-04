/**
 * @file
 */

#ifndef WAIT_STATE_HPP
#define WAIT_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class WaitState : public BaseState {
public:
    /**
     * @brief Construct a new WaitState object.
     *
     * @param id The id of the state.
     * @param micras The Micras object.
     * @param next_state_id The id of the state to go after ending the wait.
     * @param wait_time_ms The time to wait in milliseconds.
     */
    WaitState(uint8_t id, Micras& micras, uint8_t next_state_id, uint16_t wait_time_ms = 3000) :
        BaseState{id, micras}, next_state_id{next_state_id}, wait_time_ms{wait_time_ms} { }

    /**
     * @brief Execute this state.
     *
     * @param previous_state_id The id of the last executed state.
     *
     * @return The id of the next state.
     */
    uint8_t run(uint8_t previous_state_id) override {
        if (previous_state_id != this->get_id()) {
            this->wait_timer.reset_ms();
        }

        if (this->wait_timer.elapsed_time_ms() > this->wait_time_ms) {
            this->micras.odometry.reset();
            this->micras.look_at_point.reset();
            this->micras.go_to_point.reset();
            this->micras.imu.calibrate();
            this->micras.current_action =
                this->micras.mapping.get_action(this->micras.odometry.get_state().pose, this->micras.objective);

            return this->next_state_id;
        }

        return this->get_id();
    }

private:
    /**
     * @brief Timer for the wait status.
     */
    hal::Timer wait_timer;

    /**
     * @brief Id of the state to go after ending the wait.
     */
    uint8_t next_state_id;

    /**
     * @brief Time to wait in milliseconds.
     */
    uint16_t wait_time_ms;
};
}  // namespace micras

#endif  // WAIT_STATE_HPP
