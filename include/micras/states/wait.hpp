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
     * @brief Execute the entry function of this state.
     */
    void on_entry() override {
        this->micras.stop();
        this->wait_stopwatch.reset_ms();
    }

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override {
        if (this->wait_stopwatch.elapsed_time_ms() > this->wait_time_ms) {
            return this->next_state_id;
        }

        return this->get_id();
    }

private:
    /**
     * @brief Stopwatch for the wait status.
     */
    proxy::Stopwatch wait_stopwatch;

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
