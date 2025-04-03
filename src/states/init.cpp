/**
 * @file
 */

#include "micras/states/init.hpp"

namespace micras {
uint8_t InitState::run(uint8_t /*previous_state_id*/) {
    if (not this->micras.imu.check_whoami()) {
        return Micras::State::ERROR;
    }

    return Micras::State::IDLE;
}
}  // namespace micras
