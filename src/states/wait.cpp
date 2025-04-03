/**
 * @file
 */

#include "micras/states/wait.hpp"

namespace micras {
WaitState::WaitState(uint8_t id, Micras& micras, uint8_t next_state_id, uint16_t wait_time_ms) :
    State(id), micras{micras}, next_state_id{next_state_id}, wait_time_ms{wait_time_ms} { }

uint8_t WaitState::run() {
    if (this->micras.wait_timer.elapsed_time_ms() > this->wait_time_ms) {
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
}  // namespace micras
